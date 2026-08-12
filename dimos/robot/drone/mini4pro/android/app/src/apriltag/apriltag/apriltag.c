/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include "apriltag.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "common/image_u8.h"
#include "common/image_u8_parallel.h"
#include "common/image_u8x3.h"
#include "common/zarray.h"
#include "common/matd.h"
#include "common/homography.h"
#include "common/timeprofile.h"
#include "common/math_util.h"
#include "common/g2d.h"
#include "common/debug_print.h"

#include "apriltag_math.h"

#include "common/postscript_utils.h"

#ifdef _WIN32
static inline void srandom(unsigned int seed)
{
        srand(seed);
}

static inline long int random(void)
{
        return rand();
}
#endif

#define APRILTAG_U64_ONE ((uint64_t) 1)

extern zarray_t *apriltag_quad_thresh(apriltag_detector_t *td, image_u8_t *im);

// Regresses a model of the form:
// intensity(x,y) = C0*x + C1*y + CC2
// The J matrix is the:
//    J = [ x1 y1 1 ]
//        [ x2 y2 1 ]
//        [ ...     ]
//  The A matrix is J'J

struct graymodel
{
    double A[3][3];
    double B[3];
    double C[3];
};

static void graymodel_init(struct graymodel *gm)
{
    memset(gm, 0, sizeof(struct graymodel));
}

static void graymodel_add(struct graymodel *gm, double x, double y, double gray)
{
    // update upper right entries of A = J'J
    gm->A[0][0] += x*x;
    gm->A[0][1] += x*y;
    gm->A[0][2] += x;
    gm->A[1][1] += y*y;
    gm->A[1][2] += y;
    gm->A[2][2] += 1;

    // update B = J'gray
    gm->B[0] += x * gray;
    gm->B[1] += y * gray;
    gm->B[2] += gray;
}

static void graymodel_solve(struct graymodel *gm)
{
    mat33_sym_solve((double*) gm->A, gm->B, gm->C);
}

static double graymodel_interpolate(struct graymodel *gm, double x, double y)
{
    return gm->C[0]*x + gm->C[1]*y + gm->C[2];
}

static inline int popcount64(uint64_t x)
{
    x -= (x >> 1) & 0x5555555555555555ULL;
    x = (x & 0x3333333333333333ULL) + ((x >> 2) & 0x3333333333333333ULL);
    x = (x + (x >> 4)) & 0x0f0f0f0f0f0f0f0fULL;
    return (x * 0x0101010101010101ULL) >> 56;
}

/**
 * Assuming we are drawing the image one quadrant at a time, what would the rotated image look like?
 * Special care is taken to handle the case where there is a middle pixel of the image.
 */
static uint64_t rotate90(uint64_t w, int numBits)
{
    int p = numBits;
    uint64_t l = 0;
    if (numBits % 4 == 1) {
	p = numBits - 1;
	l = 1;
    }
    w = ((w >> l) << (p/4 + l)) | (w >> (3 * p/ 4 + l) << l) | (w & l);
    w &= ((APRILTAG_U64_ONE << numBits) - 1);
    return w;
}

static void quad_destroy(struct quad *quad)
{
    if (!quad)
        return;

    matd_destroy(quad->H);
    matd_destroy(quad->Hinv);
    free(quad);
}

static struct quad *quad_copy(struct quad *quad)
{
    struct quad *q = calloc(1, sizeof(struct quad));
    memcpy(q, quad, sizeof(struct quad));
    if (quad->H)
        q->H = matd_copy(quad->H);
    if (quad->Hinv)
        q->Hinv = matd_copy(quad->Hinv);
    return q;
}

struct quick_decode_result
{
    uint64_t rcode;   // the queried code
    uint16_t id;      // the tag ID (a small integer)
    uint8_t hamming;  // how many errors corrected?
    uint8_t rotation; // number of rotations [0, 3]
};

#define NUM_CHUNKS 4

struct quick_decode
{
    int nbits;
    int chunk_size;
    int capacity;
    int chunk_mask;
    int shifts[NUM_CHUNKS];

    // chunk_offsets is a map from chunk value to a range of locations in chunk_ids.
    // Together with chunk_ids, this allows a lookup of all codes matching a chunk value.
    uint16_t* chunk_offsets[NUM_CHUNKS];

    // chunk_ids is an array of indices into the codes table
    uint16_t* chunk_ids[NUM_CHUNKS];

    int maxhamming;
    int ncodes;
};

static void quick_decode_uninit(apriltag_family_t *fam)
{
    if (!fam->impl)
        return;

    struct quick_decode *qd = (struct quick_decode*) fam->impl;
    for (int i = 0; i < NUM_CHUNKS; i++) {
        free(qd->chunk_offsets[i]);
        free(qd->chunk_ids[i]);
    }
    free(qd);
    fam->impl = NULL;
}

static void quick_decode_init(apriltag_family_t *family, int maxhamming)
{
    assert(family->impl == NULL);
    assert(family->ncodes < 65536);

    if (maxhamming > 3) {
        debug_print("\"maxhamming\" beyond 3 not supported\n");
        errno = EINVAL;
        return;
    }

    struct quick_decode *qd = calloc(1, sizeof(struct quick_decode));
    if (!qd) {
        debug_print("Memory allocation failed\n");
        return;
    }
    family->impl = qd;

    qd->maxhamming = maxhamming;
    qd->ncodes = family->ncodes;
    qd->nbits = family->nbits;

    qd->chunk_size = (qd->nbits + (NUM_CHUNKS - 1)) / NUM_CHUNKS;
    qd->capacity = 1 << qd->chunk_size;
    qd->chunk_mask = qd->capacity - 1;

    for (int i = 0; i < NUM_CHUNKS; i++) {
        qd->shifts[i] = i * qd->chunk_size;
    }

    for (int i = 0; i < NUM_CHUNKS; i++) {
        qd->chunk_offsets[i] = calloc(qd->capacity + 1, sizeof(uint16_t));
        if (!qd->chunk_offsets[i]) {
            debug_print("Memory allocation failed\n");
            goto fail;
        }
        qd->chunk_ids[i] = calloc(qd->ncodes, sizeof(uint16_t));
        if (!qd->chunk_ids[i]) {
            debug_print("Memory allocation failed\n");
            goto fail;
        }
    }

    // Count frequencies
    for (int i = 0; i < qd->ncodes; i++) {
        uint64_t code = family->codes[i];
        for (int j = 0; j < NUM_CHUNKS; j++) {
            int val = (code >> qd->shifts[j]) & qd->chunk_mask;
            qd->chunk_offsets[j][val + 1]++;
        }
    }

    // Prefix sum
    for (int i = 0; i < NUM_CHUNKS; i++) {
        for (int j = 0; j < qd->capacity; j++) {
            qd->chunk_offsets[i][j + 1] += qd->chunk_offsets[i][j];
        }
    }

    // Populate ids
    uint16_t *cursors[NUM_CHUNKS];
    memset(cursors, 0, sizeof(cursors));
    for (int i = 0; i < NUM_CHUNKS; i++) {
        cursors[i] = malloc((qd->capacity + 1) * sizeof(uint16_t));
        if (cursors[i] == NULL) {
            debug_print("Memory allocation failed\n");
            for (int j = 0; j < NUM_CHUNKS; j++)
                free(cursors[j]);
            goto fail;
        }
        memcpy(cursors[i], qd->chunk_offsets[i], (qd->capacity + 1) * sizeof(uint16_t));
    }

    for (int i = 0; i < qd->ncodes; i++) {
        uint64_t code = family->codes[i];
        for (int j = 0; j < NUM_CHUNKS; j++) {
            int val = (code >> qd->shifts[j]) & qd->chunk_mask;
            int write_pos = cursors[j][val];
            qd->chunk_ids[j][write_pos] = i;
            cursors[j][val]++;
        }
    }

    for (int i = 0; i < NUM_CHUNKS; i++) {
        free(cursors[i]);
    }

    return;

fail:
    quick_decode_uninit(family);
}

// returns a result with hamming set to 255 if no decode was found.
static void quick_decode_codeword(apriltag_family_t *tf, uint64_t rcode,
                                  struct quick_decode_result *res)
{
    struct quick_decode *qd = (struct quick_decode*) tf->impl;

    // qd might be null if detector_add_family_bits() failed
    for (int ridx = 0; qd != NULL && ridx < 4; ridx++) {

        for (int i = 0; i < NUM_CHUNKS; i++) {
            int val = (rcode >> qd->shifts[i]) & qd->chunk_mask;
            int start = qd->chunk_offsets[i][val];
            int end = qd->chunk_offsets[i][val + 1];

            for (int j = start; j < end; j++) {
                uint16_t id = qd->chunk_ids[i][j];
                uint64_t correct_code = tf->codes[id];
                int hamming = popcount64(correct_code ^ rcode);

                if (hamming <= qd->maxhamming) {
                    res->rcode = rcode;
                    res->id = id;
                    res->hamming = hamming;
                    res->rotation = ridx;
                    return;
                }
            }
        }

        rcode = rotate90(rcode, tf->nbits);
    }

    res->rcode = 0;
    res->id = 65535;
    res->hamming = 255;
    res->rotation = 0;
}

static inline int detection_compare_function(const void *_a, const void *_b)
{
    apriltag_detection_t *a = *(apriltag_detection_t**) _a;
    apriltag_detection_t *b = *(apriltag_detection_t**) _b;

    return a->id - b->id;
}

void apriltag_detector_remove_family(apriltag_detector_t *td, apriltag_family_t *fam)
{
    quick_decode_uninit(fam);
    zarray_remove_value(td->tag_families, &fam, 0);
}

void apriltag_detector_add_family_bits(apriltag_detector_t *td, apriltag_family_t *fam, int bits_corrected)
{
    zarray_add(td->tag_families, &fam);

    if (!fam->impl)
        quick_decode_init(fam, bits_corrected);
}

void apriltag_detector_clear_families(apriltag_detector_t *td)
{
    for (int i = 0; i < zarray_size(td->tag_families); i++) {
        apriltag_family_t *fam;
        zarray_get(td->tag_families, i, &fam);
        quick_decode_uninit(fam);
    }
    zarray_clear(td->tag_families);
}

apriltag_detector_t *apriltag_detector_create()
{
    apriltag_detector_t *td = (apriltag_detector_t*) calloc(1, sizeof(apriltag_detector_t));

    td->nthreads = 1;
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;

    td->qtp.max_nmaxima = 10;
    td->qtp.min_cluster_pixels = 24;

    td->qtp.max_line_fit_mse = 10.0;
    td->qtp.cos_critical_rad = cos(10 * M_PI / 180);
    td->qtp.deglitch = false;
    td->qtp.min_white_black_diff = 5;

    td->tag_families = zarray_create(sizeof(apriltag_family_t*));

    pthread_mutex_init(&td->mutex, NULL);

    td->tp = timeprofile_create();

    td->refine_edges = true;
    td->decode_sharpening = 0.25;


    td->debug = false;

    // NB: defer initialization of td->wp so that the user can
    // override td->nthreads.

    return td;
}

void apriltag_detector_destroy(apriltag_detector_t *td)
{
    timeprofile_destroy(td->tp);
    workerpool_destroy(td->wp);

    apriltag_detector_clear_families(td);

    zarray_destroy(td->tag_families);
    free(td);
}

struct quad_decode_task
{
    int i0, i1;
    zarray_t *quads;
    apriltag_detector_t *td;

    image_u8_t *im;
    zarray_t *detections;

    image_u8_t *im_samples;
};

struct evaluate_quad_ret
{
    int64_t rcode;
    double  score;
    matd_t  *H, *Hinv;

    int decode_status;
    struct quick_decode_result res;
};

static matd_t* homography_compute2(double c[4][4]) {
    double A[] =  {
            c[0][0], c[0][1], 1,       0,       0, 0, -c[0][0]*c[0][2], -c[0][1]*c[0][2], c[0][2],
                  0,       0, 0, c[0][0], c[0][1], 1, -c[0][0]*c[0][3], -c[0][1]*c[0][3], c[0][3],
            c[1][0], c[1][1], 1,       0,       0, 0, -c[1][0]*c[1][2], -c[1][1]*c[1][2], c[1][2],
                  0,       0, 0, c[1][0], c[1][1], 1, -c[1][0]*c[1][3], -c[1][1]*c[1][3], c[1][3],
            c[2][0], c[2][1], 1,       0,       0, 0, -c[2][0]*c[2][2], -c[2][1]*c[2][2], c[2][2],
                  0,       0, 0, c[2][0], c[2][1], 1, -c[2][0]*c[2][3], -c[2][1]*c[2][3], c[2][3],
            c[3][0], c[3][1], 1,       0,       0, 0, -c[3][0]*c[3][2], -c[3][1]*c[3][2], c[3][2],
                  0,       0, 0, c[3][0], c[3][1], 1, -c[3][0]*c[3][3], -c[3][1]*c[3][3], c[3][3],
    };

    double epsilon = 1e-10;

    // Eliminate.
    for (int col = 0; col < 8; col++) {
        // Find best row to swap with.
        double max_val = 0;
        int max_val_idx = -1;
        for (int row = col; row < 8; row++) {
            double val = fabs(A[row*9 + col]);
            if (val > max_val) {
                max_val = val;
                max_val_idx = row;
            }
        }

        if (max_val_idx < 0) {
            return NULL;
        }

        if (max_val < epsilon) {
            debug_print("WRN: Matrix is singular.\n");
            return NULL;
        }

        // Swap to get best row.
        if (max_val_idx != col) {
            for (int i = col; i < 9; i++) {
                double tmp = A[col*9 + i];
                A[col*9 + i] = A[max_val_idx*9 + i];
                A[max_val_idx*9 + i] = tmp;
            }
        }

        // Do eliminate.
        for (int i = col + 1; i < 8; i++) {
            double f = A[i*9 + col]/A[col*9 + col];
            A[i*9 + col] = 0;
            for (int j = col + 1; j < 9; j++) {
                A[i*9 + j] -= f*A[col*9 + j];
            }
        }
    }

    // Back solve.
    for (int col = 7; col >=0; col--) {
        double sum = 0;
        for (int i = col + 1; i < 8; i++) {
            sum += A[col*9 + i]*A[i*9 + 8];
        }
        A[col*9 + 8] = (A[col*9 + 8] - sum)/A[col*9 + col];
    }
    return matd_create_data(3, 3, (double[]) { A[8], A[17], A[26], A[35], A[44], A[53], A[62], A[71], 1 });
}

// returns non-zero if an error occurs (i.e., H has no inverse)
static int quad_update_homographies(struct quad *quad)
{
    //zarray_t *correspondences = zarray_create(sizeof(float[4]));

    double corr_arr[4][4];

    for (int i = 0; i < 4; i++) {
        corr_arr[i][0] = (i==0 || i==3) ? -1 : 1;
        corr_arr[i][1] = (i==0 || i==1) ? -1 : 1;
        corr_arr[i][2] = quad->p[i][0];
        corr_arr[i][3] = quad->p[i][1];
    }

    if (quad->H)
        matd_destroy(quad->H);
    if (quad->Hinv)
        matd_destroy(quad->Hinv);

    // XXX Tunable
    quad->H = homography_compute2(corr_arr);
    if (quad->H != NULL) {
        quad->Hinv = matd_inverse(quad->H);
        if (quad->Hinv != NULL) {
	    // Success!
            return 0;
        }
        matd_destroy(quad->H);
        quad->H = NULL;
    }
    return -1;
}

static double value_for_pixel(image_u8_t *im, double px, double py) {
    int x1 = floor(px - 0.5);
    int x2 = ceil(px - 0.5);
    double x = px - 0.5 - x1;
    int y1 = floor(py - 0.5);
    int y2 = ceil(py - 0.5);
    double y = py - 0.5 - y1;
    if (x1 < 0 || x2 >= im->width || y1 < 0 || y2 >= im->height) {
        return -1;
    }
    return im->buf[y1*im->stride + x1]*(1-x)*(1-y) +
            im->buf[y1*im->stride + x2]*x*(1-y) +
            im->buf[y2*im->stride + x1]*(1-x)*y +
            im->buf[y2*im->stride + x2]*x*y;
}

static void sharpen(apriltag_detector_t* td, double* values, int size) {
    double *sharpened = malloc(sizeof(double)*size*size);
    double kernel[9] = {
        0, -1, 0,
        -1, 4, -1,
        0, -1, 0
    };

    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            sharpened[y*size + x] = 0;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    if ((y + i - 1) < 0 || (y + i - 1) > size - 1 || (x + j - 1) < 0 || (x + j - 1) > size - 1) {
                        continue;
                    }
                    sharpened[y*size + x] += values[(y + i - 1)*size + (x + j - 1)]*kernel[i*3 + j];
                }
            }
        }
    }


    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            values[y*size + x] = values[y*size + x] + td->decode_sharpening*sharpened[y*size + x];
        }
    }

    free(sharpened);
}

// returns the decision margin. Return < 0 if the detection should be rejected.
static float quad_decode(apriltag_detector_t* td, apriltag_family_t *family, image_u8_t *im, struct quad *quad, struct quick_decode_result *res, image_u8_t *im_samples)
{
    // decode the tag binary contents by sampling the pixel
    // closest to the center of each bit cell.

    // We will compute a threshold by sampling known white/black cells around this tag.
    // This sampling is achieved by considering a set of samples along lines.
    //
    // coordinates are given in bit coordinates. ([0, fam->border_width]).
    //
    // { initial x, initial y, delta x, delta y, WHITE=1 }
    float patterns[] = {
        // left white column
        -0.5, 0.5,
        0, 1,
        1,

        // left black column
        0.5, 0.5,
        0, 1,
        0,

        // right white column
        family->width_at_border + 0.5, .5,
        0, 1,
        1,

        // right black column
        family->width_at_border - 0.5, .5,
        0, 1,
        0,

        // top white row
        0.5, -0.5,
        1, 0,
        1,

        // top black row
        0.5, 0.5,
        1, 0,
        0,

        // bottom white row
        0.5, family->width_at_border + 0.5,
        1, 0,
        1,

        // bottom black row
        0.5, family->width_at_border - 0.5,
        1, 0,
        0

        // XXX double-counts the corners.
    };

    struct graymodel whitemodel, blackmodel;
    graymodel_init(&whitemodel);
    graymodel_init(&blackmodel);

    for (long unsigned int pattern_idx = 0; pattern_idx < sizeof(patterns)/(5*sizeof(float)); pattern_idx ++) {
        float *pattern = &patterns[pattern_idx * 5];

        int is_white = pattern[4];

        for (int i = 0; i < family->width_at_border; i++) {
            double tagx01 = (pattern[0] + i*pattern[2]) / (family->width_at_border);
            double tagy01 = (pattern[1] + i*pattern[3]) / (family->width_at_border);

            double tagx = 2*(tagx01-0.5);
            double tagy = 2*(tagy01-0.5);

            double px, py;
            homography_project(quad->H, tagx, tagy, &px, &py);

            // don't round
            int ix = px;
            int iy = py;
            if (ix < 0 || iy < 0 || ix >= im->width || iy >= im->height)
                continue;

            int v = im->buf[iy*im->stride + ix];

            if (im_samples) {
                im_samples->buf[iy*im_samples->stride + ix] = (1-is_white)*255;
            }

            if (is_white)
                graymodel_add(&whitemodel, tagx, tagy, v);
            else
                graymodel_add(&blackmodel, tagx, tagy, v);
        }
    }

    if (family->width_at_border > 1) {
        graymodel_solve(&whitemodel);
        graymodel_solve(&blackmodel);
    } else {
        graymodel_solve(&whitemodel);
        blackmodel.C[0] = 0;
        blackmodel.C[1] = 0;
        blackmodel.C[2] = blackmodel.B[2]/4;
    }

    // XXX Tunable
    if ((graymodel_interpolate(&whitemodel, 0, 0) - graymodel_interpolate(&blackmodel, 0, 0) < 0) != family->reversed_border) {
        return -1;
    }

    // compute the average decision margin (how far was each bit from
    // the decision boundary?
    //
    // we score this separately for white and black pixels and return
    // the minimum average threshold for black/white pixels. This is
    // to penalize thresholds that are too close to an extreme.
    float black_score = 0, white_score = 0;
    float black_score_count = 1, white_score_count = 1;

    double *values = calloc(family->total_width*family->total_width, sizeof(double));

    int min_coord = (family->width_at_border - family->total_width)/2;
    for (uint32_t i = 0; i < family->nbits; i++) {
        int bity = family->bit_y[i];
        int bitx = family->bit_x[i];

        double tagx01 = (bitx + 0.5) / (family->width_at_border);
        double tagy01 = (bity + 0.5) / (family->width_at_border);

        // scale to [-1, 1]
        double tagx = 2*(tagx01-0.5);
        double tagy = 2*(tagy01-0.5);

        double px, py;
        homography_project(quad->H, tagx, tagy, &px, &py);

        double v = value_for_pixel(im, px, py);

        if (v == -1) {
            continue;
        }

        double thresh = (graymodel_interpolate(&blackmodel, tagx, tagy) + graymodel_interpolate(&whitemodel, tagx, tagy)) / 2.0;
        values[family->total_width*(bity - min_coord) + bitx - min_coord] = v - thresh;

        if (im_samples) {
            int ix = px;
            int iy = py;
            im_samples->buf[iy*im_samples->stride + ix] = (v < thresh) * 255;
        }
    }

    sharpen(td, values, family->total_width);

    uint64_t rcode = 0;
    for (uint32_t i = 0; i < family->nbits; i++) {
        int bity = family->bit_y[i];
        int bitx = family->bit_x[i];
        rcode = (rcode << 1);
        double v = values[(bity - min_coord)*family->total_width + bitx - min_coord];

        if (v > 0) {
            white_score += v;
            white_score_count++;
            rcode |= 1;
        } else {
            black_score -= v;
            black_score_count++;
        }
    }

    quick_decode_codeword(family, rcode, res);
    free(values);
    return fmin(white_score / white_score_count, black_score / black_score_count);
}

static void refine_edges(apriltag_detector_t *td, image_u8_t *im_orig, struct quad *quad)
{
    double lines[4][4]; // for each line, [Ex Ey nx ny]

    for (int edge = 0; edge < 4; edge++) {
        int a = edge, b = (edge + 1) & 3; // indices of the end points.

        // compute the normal to the current line estimate
        double nx = quad->p[b][1] - quad->p[a][1];
        double ny = -quad->p[b][0] + quad->p[a][0];
        double mag = sqrt(nx*nx + ny*ny);
        nx /= mag;
        ny /= mag;

        if (quad->reversed_border) {
            nx = -nx;
            ny = -ny;
        }

        // we will now fit a NEW line by sampling points near
        // our original line that have large gradients. On really big tags,
        // we're willing to sample more to get an even better estimate.
        int nsamples = imax(16, mag / 8); // XXX tunable

        // stats for fitting a line...
        double Mx = 0, My = 0, Mxx = 0, Mxy = 0, Myy = 0, N = 0;

        for (int s = 0; s < nsamples; s++) {
            // compute a point along the line... Note, we're avoiding
            // sampling *right* at the corners, since those points are
            // the least reliable.
            double alpha = (1.0 + s) / (nsamples + 1);
            double x0 = alpha*quad->p[a][0] + (1-alpha)*quad->p[b][0];
            double y0 = alpha*quad->p[a][1] + (1-alpha)*quad->p[b][1];

            // search along the normal to this line, looking at the
            // gradients along the way. We're looking for a strong
            // response.
            double Mn = 0;
            double Mcount = 0;

            // XXX tunable: how far to search?  We want to search far
            // enough that we find the best edge, but not so far that
            // we hit other edges that aren't part of the tag. We
            // shouldn't ever have to search more than quad_decimate,
            // since otherwise we would (ideally) have started our
            // search on another pixel in the first place. Likewise,
            // for very small tags, we don't want the range to be too
            // big.

            int range = td->quad_decimate + 1;

            // To reduce the overhead of bilinear interpolation, we can
            // reduce the number of steps per unit.
            int steps_per_unit = 4;
            double step_length = 1.0 / steps_per_unit;
            int max_steps = 2 * steps_per_unit * range + 1;
            double delta = 0.5;

            // XXX tunable step size.
            for (int step = 0; step < max_steps; ++step) {
                double n = -range + step_length * step;
                // Because of the guaranteed winding order of the
                // points in the quad, we will start inside the white
                // portion of the quad and work our way outward.
                //
                // sample to points (x1,y1) and (x2,y2) XXX tunable:
                // how far +/- to look? Small values compute the
                // gradient more precisely, but are more sensitive to
                // noise.
                double grange = 1;

                double x1 = x0 + (n + grange)*nx - delta;
                double y1 = y0 + (n + grange)*ny - delta;
                double x1i_d, y1i_d, a1, b1;
                a1 = modf(x1, &x1i_d);
                b1 = modf(y1, &y1i_d);
                int x1i = x1i_d, y1i = y1i_d;

                if (x1i < 0 || x1i + 1 >= im_orig->width || y1i < 0 || y1i + 1 >= im_orig->height)
                    continue;

                double x2 = x0 + (n - grange)*nx - delta;
                double y2 = y0 + (n - grange)*ny - delta;
                double x2i_d, y2i_d, a2, b2;
                a2 = modf(x2, &x2i_d);
                b2 = modf(y2, &y2i_d);
                int x2i = x2i_d, y2i = y2i_d;

                if (x2i < 0 || x2i + 1 >= im_orig->width || y2i < 0 || y2i + 1 >= im_orig->height)
                    continue;

                // interpolate
                double g1 = (1 - a1) * (1 - b1) * im_orig->buf[y1i*im_orig->stride + x1i] +
                                  a1 * (1 - b1) * im_orig->buf[y1i*im_orig->stride + x1i + 1] +
                            (1 - a1) *    b1    * im_orig->buf[(y1i + 1)*im_orig->stride + x1i] +
                                  a1 *    b1    * im_orig->buf[(y1i + 1)*im_orig->stride + x1i + 1];
                double g2 = (1 - a2) * (1 - b2) * im_orig->buf[y2i*im_orig->stride + x2i] +
                                  a2 * (1 - b2) * im_orig->buf[y2i*im_orig->stride + x2i + 1] +
                            (1 - a2) *    b2    * im_orig->buf[(y2i + 1)*im_orig->stride + x2i] +
                                  a2 *    b2    * im_orig->buf[(y2i + 1)*im_orig->stride + x2i + 1];
                if (g1 < g2) // reject points whose gradient is "backwards". They can only hurt us.
                    continue;

                double weight = (g2 - g1)*(g2 - g1); // XXX tunable. What shape for weight=f(g2-g1)?

                // compute weighted average of the gradient at this point.
                Mn += weight*n;
                Mcount += weight;
            }

            // what was the average point along the line?
            if (Mcount == 0)
                continue;

            double n0 = Mn / Mcount;

            // where is the point along the line?
            double bestx = x0 + n0*nx;
            double besty = y0 + n0*ny;

            // update our line fit statistics
            Mx += bestx;
            My += besty;
            Mxx += bestx*bestx;
            Mxy += bestx*besty;
            Myy += besty*besty;
            N++;
        }

        // fit a line
        double Ex = Mx / N, Ey = My / N;
        double Cxx = Mxx / N - Ex*Ex;
        double Cxy = Mxy / N - Ex*Ey;
        double Cyy = Myy / N - Ey*Ey;

        // TODO: Can replace this with same code as in fit_line.
        double normal_theta = .5 * atan2f(-2*Cxy, (Cyy - Cxx));
        nx = cosf(normal_theta);
        ny = sinf(normal_theta);
        lines[edge][0] = Ex;
        lines[edge][1] = Ey;
        lines[edge][2] = nx;
        lines[edge][3] = ny;
    }

    // now refit the corners of the quad
    for (int i = 0; i < 4; i++) {

        // solve for the intersection of lines (i) and (i+1)&3.
        double A00 =  lines[i][3],  A01 = -lines[(i+1)&3][3];
        double A10 =  -lines[i][2],  A11 = lines[(i+1)&3][2];
        double B0 = -lines[i][0] + lines[(i+1)&3][0];
        double B1 = -lines[i][1] + lines[(i+1)&3][1];

        double det = A00 * A11 - A10 * A01;

        // inverse.
        if (fabs(det) > 0.001) {
            // solve
            double W00 = A11 / det, W01 = -A01 / det;

            double L0 = W00*B0 + W01*B1;

            // Compute intersection. Note that line i represents the line from corner i to (i+1)&3, so
	    // the intersection of line i with line (i+1)&3 represents corner (i+1)&3.
            quad->p[(i+1)&3][0] = lines[i][0] + L0*A00;
            quad->p[(i+1)&3][1] = lines[i][1] + L0*A10;
        } else {
            // this is a bad sign. We'll just keep the corner we had.
//            debug_print("bad det: %15f %15f %15f %15f %15f\n", A00, A11, A10, A01, det);
        }
    }
}

static void quad_decode_task(void *_u)
{
    struct quad_decode_task *task = (struct quad_decode_task*) _u;
    apriltag_detector_t *td = task->td;
    image_u8_t *im = task->im;

    for (int quadidx = task->i0; quadidx < task->i1; quadidx++) {
        struct quad *quad_original;
        zarray_get_volatile(task->quads, quadidx, &quad_original);

        // refine edges is not dependent upon the tag family, thus
        // apply this optimization BEFORE the other work.
        //if (td->quad_decimate > 1 && td->refine_edges) {
        if (td->refine_edges) {
            refine_edges(td, im, quad_original);
        }

        // make sure the homographies are computed...
        if (quad_update_homographies(quad_original) != 0)
            continue;

        for (int famidx = 0; famidx < zarray_size(td->tag_families); famidx++) {
            apriltag_family_t *family;
            zarray_get(td->tag_families, famidx, &family);

            if (family->reversed_border != quad_original->reversed_border) {
                continue;
            }

            // since the geometry of tag families can vary, start any
            // optimization process over with the original quad.
            struct quad *quad = quad_copy(quad_original);

            struct quick_decode_result res;

            float decision_margin = quad_decode(td, family, im, quad, &res, task->im_samples);

            if (decision_margin >= 0 && res.hamming < 255) {
                apriltag_detection_t *det = calloc(1, sizeof(apriltag_detection_t));

                det->family = family;
                det->id = res.id;
                det->hamming = res.hamming;
                det->decision_margin = decision_margin;

                double theta = res.rotation * M_PI / 2.0;
                double c = cos(theta), s = sin(theta);

                // Fix the rotation of our homography to properly orient the tag
                matd_t *R = matd_create(3,3);
                MATD_EL(R, 0, 0) = c;
                MATD_EL(R, 0, 1) = -s;
                MATD_EL(R, 1, 0) = s;
                MATD_EL(R, 1, 1) = c;
                MATD_EL(R, 2, 2) = 1;

                det->H = matd_op("M*M", quad->H, R);

                matd_destroy(R);

                homography_project(det->H, 0, 0, &det->c[0], &det->c[1]);

                // [-1, -1], [1, -1], [1, 1], [-1, 1], Desired points
                // [-1, 1], [1, 1], [1, -1], [-1, -1], FLIP Y
                // adjust the points in det->p so that they correspond to
                // counter-clockwise around the quad, starting at -1,-1.
                for (int i = 0; i < 4; i++) {
                    int tcx = (i == 1 || i == 2) ? 1 : -1;
                    int tcy = (i < 2) ? 1 : -1;

                    double p[2];

                    homography_project(det->H, tcx, tcy, &p[0], &p[1]);

                    det->p[i][0] = p[0];
                    det->p[i][1] = p[1];
                }

                pthread_mutex_lock(&td->mutex);
                zarray_add(task->detections, &det);
                pthread_mutex_unlock(&td->mutex);
            }

            quad_destroy(quad);
        }
    }
}

void apriltag_detection_destroy(apriltag_detection_t *det)
{
    if (det == NULL)
        return;

    matd_destroy(det->H);
    free(det);
}

static int prefer_smaller(int pref, double q0, double q1)
{
    if (pref)     // already prefer something? exit.
        return pref;

    if (q0 < q1)
        return -1; // we now prefer q0
    if (q1 < q0)
        return 1; // we now prefer q1

    // no preference
    return 0;
}

zarray_t *apriltag_detector_detect(apriltag_detector_t *td, image_u8_t *im_orig)
{
    if (zarray_size(td->tag_families) == 0) {
        zarray_t *s = zarray_create(sizeof(apriltag_detection_t*));
        debug_print("No tag families enabled\n");
        return s;
    }

    if (im_orig->width < 8 || im_orig->height < 8) {
        zarray_t *s = zarray_create(sizeof(apriltag_detection_t*));
        debug_print("Image too small (%d x %d)\n", im_orig->width, im_orig->height);
        return s;
    }

    if (td->wp == NULL || td->nthreads != workerpool_get_nthreads(td->wp)) {
        workerpool_destroy(td->wp);
        td->wp = workerpool_create(td->nthreads);
        if (td->wp == NULL) {
            // creating workerpool failed - return empty zarray
            return zarray_create(sizeof(apriltag_detection_t*));
        }
    }

    timeprofile_clear(td->tp);
    timeprofile_stamp(td->tp, "init");

    ///////////////////////////////////////////////////////////
    // Step 1. Detect quads according to requested image decimation
    // and blurring parameters.
    image_u8_t *quad_im = im_orig;
    if (td->quad_decimate > 1) {
        quad_im = image_u8_decimate(im_orig, td->quad_decimate);

        timeprofile_stamp(td->tp, "decimate");
    }

    if (td->quad_sigma != 0) {
        // compute a reasonable kernel width by figuring that the
        // kernel should go out 2 std devs.
        //
        // max sigma          ksz
        // 0.499              1  (disabled)
        // 0.999              3
        // 1.499              5
        // 1.999              7

        float sigma = fabsf((float) td->quad_sigma);

        int ksz = 4 * sigma; // 2 std devs in each direction
        if ((ksz & 1) == 0)
            ksz++;

        if (ksz > 1) {

            if (td->quad_sigma > 0) {
                // Apply a blur
                image_u8_gaussian_blur_parallel(td->wp, quad_im, sigma, ksz);
            } else {
                // SHARPEN the image by subtracting the low frequency components.
                image_u8_t *orig = image_u8_copy(quad_im);
                image_u8_gaussian_blur_parallel(td->wp, quad_im, sigma, ksz);

                for (int y = 0; y < orig->height; y++) {
                    for (int x = 0; x < orig->width; x++) {
                        int vorig = orig->buf[y*orig->stride + x];
                        int vblur = quad_im->buf[y*quad_im->stride + x];

                        int v = 2*vorig - vblur;
                        if (v < 0)
                            v = 0;
                        if (v > 255)
                            v = 255;

                        quad_im->buf[y*quad_im->stride + x] = (uint8_t) v;
                    }
                }
                image_u8_destroy(orig);
            }
        }
    }

    timeprofile_stamp(td->tp, "blur/sharp");

    if (td->debug)
        image_u8_write_pnm(quad_im, "debug_preprocess.pnm");

    zarray_t *quads = apriltag_quad_thresh(td, quad_im);

    // adjust centers of pixels so that they correspond to the
    // original full-resolution image.
    if (td->quad_decimate > 1) {
        for (int i = 0; i < zarray_size(quads); i++) {
            struct quad *q;
            zarray_get_volatile(quads, i, &q);

            for (int j = 0; j < 4; j++) {
                q->p[j][0] *= td->quad_decimate;
                q->p[j][1] *= td->quad_decimate;
            }
        }
    }

    if (quad_im != im_orig)
        image_u8_destroy(quad_im);

    zarray_t *detections = zarray_create(sizeof(apriltag_detection_t*));

    td->nquads = zarray_size(quads);

    timeprofile_stamp(td->tp, "quads");

    if (td->debug) {
        image_u8_t *im_quads = image_u8_copy(im_orig);
        image_u8_darken(im_quads);
        image_u8_darken(im_quads);

        srandom(0);

        for (int i = 0; i < zarray_size(quads); i++) {
            struct quad *quad;
            zarray_get_volatile(quads, i, &quad);

            const int bias = 100;
            int color = bias + (random() % (255-bias));

            image_u8_draw_line(im_quads, quad->p[0][0], quad->p[0][1], quad->p[1][0], quad->p[1][1], color, 1);
            image_u8_draw_line(im_quads, quad->p[1][0], quad->p[1][1], quad->p[2][0], quad->p[2][1], color, 1);
            image_u8_draw_line(im_quads, quad->p[2][0], quad->p[2][1], quad->p[3][0], quad->p[3][1], color, 1);
            image_u8_draw_line(im_quads, quad->p[3][0], quad->p[3][1], quad->p[0][0], quad->p[0][1], color, 1);
        }

        image_u8_write_pnm(im_quads, "debug_quads_raw.pnm");
        image_u8_destroy(im_quads);
    }

    ////////////////////////////////////////////////////////////////
    // Step 2. Decode tags from each quad.
    if (1) {
        image_u8_t *im_samples = td->debug ? image_u8_copy(im_orig) : NULL;

        int chunksize = 1 + zarray_size(quads) / (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);

        struct quad_decode_task *tasks = malloc(sizeof(struct quad_decode_task)*(zarray_size(quads) / chunksize + 1));

        int ntasks = 0;
        for (int i = 0; i < zarray_size(quads); i+= chunksize) {
            tasks[ntasks].i0 = i;
            tasks[ntasks].i1 = imin(zarray_size(quads), i + chunksize);
            tasks[ntasks].quads = quads;
            tasks[ntasks].td = td;
            tasks[ntasks].im = im_orig;
            tasks[ntasks].detections = detections;

            tasks[ntasks].im_samples = im_samples;

            workerpool_add_task(td->wp, quad_decode_task, &tasks[ntasks]);
            ntasks++;
        }

        workerpool_run(td->wp);

        free(tasks);

        if (im_samples != NULL) {
            image_u8_write_pnm(im_samples, "debug_samples.pnm");
            image_u8_destroy(im_samples);
        }
    }

    if (td->debug) {
        image_u8_t *im_quads = image_u8_copy(im_orig);
        image_u8_darken(im_quads);
        image_u8_darken(im_quads);

        srandom(0);

        for (int i = 0; i < zarray_size(quads); i++) {
            struct quad *quad;
            zarray_get_volatile(quads, i, &quad);

            const int bias = 100;
            int color = bias + (random() % (255-bias));

            image_u8_draw_line(im_quads, quad->p[0][0], quad->p[0][1], quad->p[1][0], quad->p[1][1], color, 1);
            image_u8_draw_line(im_quads, quad->p[1][0], quad->p[1][1], quad->p[2][0], quad->p[2][1], color, 1);
            image_u8_draw_line(im_quads, quad->p[2][0], quad->p[2][1], quad->p[3][0], quad->p[3][1], color, 1);
            image_u8_draw_line(im_quads, quad->p[3][0], quad->p[3][1], quad->p[0][0], quad->p[0][1], color, 1);

        }

        image_u8_write_pnm(im_quads, "debug_quads_fixed.pnm");
        image_u8_destroy(im_quads);
    }

    timeprofile_stamp(td->tp, "decode+refinement");

    ////////////////////////////////////////////////////////////////
    // Step 3. Reconcile detections--- don't report the same tag more
    // than once. (Allow non-overlapping duplicate detections.)
    if (1) {
        zarray_t *poly0 = g2d_polygon_create_zeros(4);
        zarray_t *poly1 = g2d_polygon_create_zeros(4);

        for (int i0 = 0; i0 < zarray_size(detections); i0++) {

            apriltag_detection_t *det0;
            zarray_get(detections, i0, &det0);

            for (int k = 0; k < 4; k++)
                zarray_set(poly0, k, det0->p[k], NULL);

            for (int i1 = i0+1; i1 < zarray_size(detections); i1++) {

                apriltag_detection_t *det1;
                zarray_get(detections, i1, &det1);

                if (det0->id != det1->id || det0->family != det1->family)
                    continue;

                for (int k = 0; k < 4; k++)
                    zarray_set(poly1, k, det1->p[k], NULL);

                if (g2d_polygon_overlaps_polygon(poly0, poly1)) {
                    // the tags overlap. Delete one, keep the other.

                    int pref = 0; // 0 means undecided which one we'll keep.
                    pref = prefer_smaller(pref, det0->hamming, det1->hamming);     // want small hamming
                    pref = prefer_smaller(pref, -det0->decision_margin, -det1->decision_margin);      // want bigger margins

                    // if we STILL don't prefer one detection over the other, then pick
                    // any deterministic criterion.
                    for (int i = 0; i < 4; i++) {
                        pref = prefer_smaller(pref, det0->p[i][0], det1->p[i][0]);
                        pref = prefer_smaller(pref, det0->p[i][1], det1->p[i][1]);
                    }

                    if (pref == 0) {
                        // at this point, we should only be undecided if the tag detections
                        // are *exactly* the same. How would that happen?
                        debug_print("uh oh, no preference for overlappingdetection\n");
                    }

                    if (pref < 0) {
                        // keep det0, destroy det1
                        apriltag_detection_destroy(det1);
                        zarray_remove_index(detections, i1, 1);
                        i1--; // retry the same index
                        goto retry1;
                    } else {
                        // keep det1, destroy det0
                        apriltag_detection_destroy(det0);
                        zarray_remove_index(detections, i0, 1);
                        i0--; // retry the same index.
                        goto retry0;
                    }
                }

              retry1: ;
            }

          retry0: ;
        }

        zarray_destroy(poly0);
        zarray_destroy(poly1);
    }

    timeprofile_stamp(td->tp, "reconcile");

    ////////////////////////////////////////////////////////////////
    // Produce final debug output
    if (td->debug) {

        image_u8_t *darker = image_u8_copy(im_orig);
        image_u8_darken(darker);
        image_u8_darken(darker);

        // assume letter, which is 612x792 points.
        FILE *f = fopen("debug_output.ps", "w");
        fprintf(f, "%%!PS\n\n");
        double scale = fmin(612.0/darker->width, 792.0/darker->height);
        fprintf(f, "%f %f scale\n", scale, scale);
        fprintf(f, "0 %d translate\n", darker->height);
        fprintf(f, "1 -1 scale\n");
        postscript_image(f, darker);

        image_u8_destroy(darker);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            float rgb[3];
            int bias = 100;

            for (int j = 0; j < 3; j++) {
                rgb[j] = bias + (random() % (255-bias));
            }

            fprintf(f, "%f %f %f setrgbcolor\n", rgb[0]/255.0f, rgb[1]/255.0f, rgb[2]/255.0f);
            fprintf(f, "%f %f moveto %f %f lineto %f %f lineto %f %f lineto %f %f lineto stroke\n",
                    det->p[0][0], det->p[0][1],
                    det->p[1][0], det->p[1][1],
                    det->p[2][0], det->p[2][1],
                    det->p[3][0], det->p[3][1],
                    det->p[0][0], det->p[0][1]);
        }

        fprintf(f, "showpage\n");
        fclose(f);
    }

    if (td->debug) {
        image_u8_t *darker = image_u8_copy(im_orig);
        image_u8_darken(darker);
        image_u8_darken(darker);

        image_u8x3_t *out = image_u8x3_create(darker->width, darker->height);
        for (int y = 0; y < im_orig->height; y++) {
            for (int x = 0; x < im_orig->width; x++) {
                out->buf[y*out->stride + 3*x + 0] = darker->buf[y*darker->stride + x];
                out->buf[y*out->stride + 3*x + 1] = darker->buf[y*darker->stride + x];
                out->buf[y*out->stride + 3*x + 2] = darker->buf[y*darker->stride + x];
            }
        }

        image_u8_destroy(darker);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            float rgb[3];
            int bias = 100;

            for (int j = 0; j < 3; j++) {
                rgb[j] = bias + (random() % (255-bias));
            }

            for (int j = 0; j < 4; j++) {
                int k = (j + 1) & 3;
                image_u8x3_draw_line(out,
                                     det->p[j][0], det->p[j][1], det->p[k][0], det->p[k][1],
                                     (uint8_t[]) { rgb[0], rgb[1], rgb[2] });
            }
        }

        image_u8x3_write_pnm(out, "debug_output.pnm");
        image_u8x3_destroy(out);
    }

    // deallocate
    if (td->debug) {
        FILE *f = fopen("debug_quads.ps", "w");
        fprintf(f, "%%!PS\n\n");

        image_u8_t *darker = image_u8_copy(im_orig);
        image_u8_darken(darker);
        image_u8_darken(darker);

        // assume letter, which is 612x792 points.
        double scale = fmin(612.0/darker->width, 792.0/darker->height);
        fprintf(f, "%f %f scale\n", scale, scale);
        fprintf(f, "0 %d translate\n", darker->height);
        fprintf(f, "1 -1 scale\n");

        postscript_image(f, darker);

        image_u8_destroy(darker);

        for (int i = 0; i < zarray_size(quads); i++) {
            struct quad *q;
            zarray_get_volatile(quads, i, &q);

            float rgb[3];
            int bias = 100;

            for (int j = 0; j < 3; j++) {
                rgb[j] = bias + (random() % (255-bias));
            }

            fprintf(f, "%f %f %f setrgbcolor\n", rgb[0]/255.0f, rgb[1]/255.0f, rgb[2]/255.0f);
            fprintf(f, "%f %f moveto %f %f lineto %f %f lineto %f %f lineto %f %f lineto stroke\n",
                    q->p[0][0], q->p[0][1],
                    q->p[1][0], q->p[1][1],
                    q->p[2][0], q->p[2][1],
                    q->p[3][0], q->p[3][1],
                    q->p[0][0], q->p[0][1]);
        }

        fprintf(f, "showpage\n");
        fclose(f);
    }

    timeprofile_stamp(td->tp, "debug output");

    for (int i = 0; i < zarray_size(quads); i++) {
        struct quad *quad;
        zarray_get_volatile(quads, i, &quad);
        matd_destroy(quad->H);
        matd_destroy(quad->Hinv);
    }

    zarray_destroy(quads);

    zarray_sort(detections, detection_compare_function);
    timeprofile_stamp(td->tp, "cleanup");

    return detections;
}


// Call this method on each of the tags returned by apriltag_detector_detect
void apriltag_detections_destroy(zarray_t *detections)
{
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        apriltag_detection_destroy(det);
    }

    zarray_destroy(detections);
}

image_u8_t *apriltag_to_image(apriltag_family_t *fam, uint32_t idx)
{
    assert(fam != NULL);
    assert(idx < fam->ncodes);

    uint64_t code = fam->codes[idx];

    image_u8_t *im = image_u8_create(fam->total_width, fam->total_width);

    int white_border_width = fam->width_at_border + (fam->reversed_border ? 0 : 2);
    int white_border_start = (fam->total_width - white_border_width)/2;
    // Make 1px white border
    for (int i = 0; i < white_border_width - 1; i += 1) {
        im->buf[white_border_start*im->stride + white_border_start + i] = 255;
        im->buf[(white_border_start + i)*im->stride + fam->total_width - 1 - white_border_start] = 255;
        im->buf[(fam->total_width - 1 - white_border_start)*im->stride + white_border_start + i + 1] = 255;
        im->buf[(white_border_start + 1 + i)*im->stride + white_border_start] = 255;
    }

    int border_start = (fam->total_width - fam->width_at_border)/2;
    for (uint32_t i = 0; i < fam->nbits; i++) {
        if (code & (APRILTAG_U64_ONE << (fam->nbits - i - 1))) {
            im->buf[(fam->bit_y[i] + border_start)*im->stride + fam->bit_x[i] + border_start] = 255;
        }
    }
    return im;
}

void apriltag_detection_copy(apriltag_detection_t* src, apriltag_detection_t* dst)
{
    assert(src != NULL);
    assert(dst != NULL);

    if (dst->H) {
        matd_destroy(dst->H);
    }
    dst->H = matd_copy(src->H);

    dst->c[0] = src->c[0];
    dst->c[1] = src->c[1];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            dst->p[i][j] = src->p[i][j];
        }
    }

    dst->id = src->id;
    dst->family = src->family;
    dst->hamming = src->hamming;
    dst->decision_margin = src->decision_margin;
}

zarray_t* apriltag_detections_copy(zarray_t* detections)
{
    zarray_t* detections_copy = zarray_create(sizeof(apriltag_detection_t*));
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        apriltag_detection_t* det_copy = (apriltag_detection_t*)calloc(1, sizeof(apriltag_detection_t));
        apriltag_detection_copy(det, det_copy);
        zarray_add(detections_copy, &det_copy);
    }

    return detections_copy;
}

apriltag_detector_t *apriltag_detector_copy(apriltag_detector_t *src)
{
    apriltag_detector_t *dst = (apriltag_detector_t *)malloc(sizeof(apriltag_detector_t));
    // Shallow copy of all scalar fields
    *dst = *src;

    // Reinitialize pointer fields to independent default values to avoid shared ownership and double-free issues
    dst->tag_families = zarray_create(sizeof(apriltag_family_t *));
    dst->tp = timeprofile_create();
    dst->wp = workerpool_create(src->nthreads);

    return dst;
}
