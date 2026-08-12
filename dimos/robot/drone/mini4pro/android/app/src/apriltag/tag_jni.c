/*
 * JNI shim over the AprilTag reference C library, thin enough to be measured.
 *
 * It mirrors `vision/ArucoProbe.kt` deliberately, call for call: create a detector once, hand it
 * one 8-bit luminance frame at a time, get back the ids found and the longest tag edge in pixels.
 * The two probes must be interchangeable at the call site or the comparison in
 * docs/measurements/2026-07-27-apriltag-c-vs-opencv.md would be measuring the harness.
 *
 * ## The copy is deliberate, and it is what keeps the comparison fair
 *
 * `image_u8_t` could point straight at the Java array through GetPrimitiveArrayCritical and skip a
 * memcpy. It does not, for two reasons. The library's own `image_u8_create` aligns each row to 96
 * bytes and several of its inner loops are written expecting that; and OpenCV's probe pays exactly
 * the same copy in `Mat.put(0, 0, luma)`. Removing it here and not there would hand apriltag a
 * ~0.3 ms/frame advantage that has nothing to do with detection.
 *
 * ## The pose solve (added 2026-07-28), and what it is and is not
 *
 * Until 2026-07-28 no pose left this file and `apriltag_pose.c` was not even vendored. It is now
 * (apriltag/VENDOR.md), and per detection this shim runs upstream's own
 * `estimate_tag_pose_orthogonal_iteration` — homography initialisation, Orthogonal Iteration
 * refinement, and the Schweighofer–Pinz second minimum — whenever the caller supplies intrinsics
 * and a tag size. **The solve is apriltag's, not ours**: OpenCV left this build after a
 * native-lib crash (tools/native-known-good.txt) and no new native dependency is added here.
 *
 * What this file does *not* do is decide whether the solve deserves belief. Both ambiguity
 * errors (`err1`, `err2`) cross the boundary raw, precisely so the gates — pixel size and
 * ambiguity ratio, measured in docs/measurements/2026-07-28-pose-solve-stability.md — live in
 * Kotlin (`vision/TagPose`) where a JVM test can hold them. A threshold buried in C is a
 * threshold nobody can measure, which is the same argument `decision_margin` already carries.
 *
 * The intrinsics passed in are a **fitted** focal length and an **assumed** principal point
 * (image centre; the measured nadir pixel is 2.99° away and is deliberately not used here —
 * `vision/TagWorld.nadirPointX` says why it must not stand in for the principal point). The
 * solved pose inherits that assumption and every consumer sees `metric=false` beside it.
 *
 * ## What is not here
 *
 * No thresholds, no frame policy, no belief. The longest edge in pixels is the same size proxy
 * ArucoProbe reported, and it is comparable across the two detectors because it is computed from
 * the same four corners the same way.
 *
 * ## What was added when this moved into the shipped path (2026-07-27)
 *
 * The comparison only needed "was the tag there, and how big" — one id list and one number. A
 * *sighting* needs three more things per detection, and all three are already in
 * `apriltag_detection_t`, so this is a wider read of the same struct rather than new work:
 *
 *  - **the centre**, `det->c`, because where the tag is in the image is the entire content of a
 *    bearing. Without it a detection cannot become a position.
 *  - **the hamming distance**, `det->hamming`, because it is the per-detection half of the
 *    `maxhamming` question. Two false ids in 1978 frames is the one axis on which OpenCV was
 *    better, and a filter cannot be written against a budget it cannot see.
 *  - **the decision margin**, `det->decision_margin`, apriltag's own confidence in the decode.
 *    Recorded rather than thresholded here: what a useful cut-off is on this camera is not
 *    known, and a threshold buried in C is a threshold nobody can measure.
 *
 * Both output arrays are caller-owned and reused across frames, so a detect still allocates
 * nothing on the JVM side.
 */
#include <jni.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"

#define EXPORT __attribute__((visibility("default")))

typedef struct {
    apriltag_detector_t *td;
    apriltag_family_t *tf;
    /* Reused across frames. Allocating a 2 MP image per frame would measure the allocator. */
    image_u8_t *img;
} probe_t;

EXPORT JNIEXPORT jlong JNICALL
Java_com_dimensional_mini4pro_vision_AprilTagDetector_nativeCreate(
        JNIEnv *env, jclass clazz,
        jint nthreads, jfloat quadDecimate, jfloat quadSigma,
        jboolean refineEdges, jint maxHamming)
{
    (void) env; (void) clazz;
    probe_t *p = (probe_t *) calloc(1, sizeof(probe_t));
    if (!p) return 0;
    p->tf = tag36h11_create();
    p->td = apriltag_detector_create();
    /* maxhamming is the one knob with a safety consequence rather than a speed one: it is how many
     * corrected bit errors count as a detection. Upstream's default is 2. It is a parameter here
     * because it trades detection rate against false positives and that trade has to be visible. */
    apriltag_detector_add_family_bits(p->td, p->tf, maxHamming);
    p->td->nthreads = nthreads;
    p->td->quad_decimate = quadDecimate;
    p->td->quad_sigma = quadSigma;
    p->td->refine_edges = refineEdges == JNI_TRUE;
    p->td->debug = false;
    return (jlong) (intptr_t) p;
}

/*
 * The layout of one detection in `geomOut`. 26 doubles:
 *
 *   [0]  centre x                [1]  centre y            (pixels, y down)
 *   [2]  longest edge, pixels    [3]  decision margin
 *   [4..11]  the four corners, x0,y0 .. x3,y3, in apriltag's own order (`det->p`:
 *            counter-clockwise around the quad, starting at the corner that is the tag
 *            frame's (-1, +1)). Always written — they exist whether or not anything solves.
 *   [12..20] R, row-major        [21..23] t              (the BEST ambiguity solution)
 *   [24] err1 — the best solution's object-space error
 *   [25] err2 — the other minimum's, or +Infinity when Schweighofer–Pinz found none
 *
 * [12..25] are NaN when no solve ran (tagSizeM or fx not positive). `err1 <= err2` always:
 * the two are sorted here so "which slot is the winner" is not a convention the Kotlin side
 * has to remember. R maps the TAG frame into the CAMERA frame; both are apriltag's own
 * conventions (camera: x right, y down, z out the lens; tag: x right, y down on the printed
 * face, z into the tag) and `vision/TagPose` documents them where the quaternion is made.
 */
#define GEOM_STRIDE 26

/*
 * One frame.
 *
 * `idsOut` receives 2 ints per detection: the id, then the hamming distance corrected.
 * `geomOut` receives GEOM_STRIDE doubles per detection, laid out above. The number of
 * detections written is capped by whichever array runs out first, and the return value is that
 * count — never more than the caller's arrays hold.
 *
 * `fx`/`fy`/`cx`/`cy` are pixels, `tagSizeM` metres (the black square, side to side). Passing
 * `tagSizeM <= 0` skips the solve — corners still cross, poses are NaN — which is both the
 * legacy behaviour and the "solve is off" switch.
 */
EXPORT JNIEXPORT jint JNICALL
Java_com_dimensional_mini4pro_vision_AprilTagDetector_nativeDetect(
        JNIEnv *env, jclass clazz, jlong handle,
        jbyteArray luma, jint width, jint height,
        jdouble fx, jdouble fy, jdouble cx, jdouble cy, jdouble tagSizeM,
        jintArray idsOut, jdoubleArray geomOut)
{
    (void) clazz;
    probe_t *p = (probe_t *) (intptr_t) handle;
    if (!p) return -1;

    if (!p->img || p->img->width != width || p->img->height != height) {
        if (p->img) image_u8_destroy(p->img);
        p->img = image_u8_create(width, height);
        if (!p->img) return -1;
    }

    /* Row by row, because the library's stride is padded and the source is tightly packed. */
    jbyte *src = (*env)->GetPrimitiveArrayCritical(env, luma, NULL);
    if (!src) return -1;
    if (p->img->stride == width) {
        memcpy(p->img->buf, src, (size_t) width * (size_t) height);
    } else {
        for (int y = 0; y < height; y++) {
            memcpy(p->img->buf + (size_t) y * p->img->stride,
                   (uint8_t *) src + (size_t) y * width, (size_t) width);
        }
    }
    (*env)->ReleasePrimitiveArrayCritical(env, luma, src, JNI_ABORT);

    zarray_t *detections = apriltag_detector_detect(p->td, p->img);
    int n = zarray_size(detections);

    jint *ids = (*env)->GetIntArrayElements(env, idsOut, NULL);
    jdouble *geom = (*env)->GetDoubleArrayElements(env, geomOut, NULL);
    int idCap = (int) ((*env)->GetArrayLength(env, idsOut) / 2);
    int geomCap = (int) ((*env)->GetArrayLength(env, geomOut) / GEOM_STRIDE);
    int cap = idCap < geomCap ? idCap : geomCap;
    int kept = 0;
    for (int i = 0; i < n && kept < cap; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        /* Per detection, not per frame: a frame with two tags in it has two sizes, and the
         * comparison's single "largest tag" number could not say which id it belonged to. */
        double longest = 0.0;
        for (int c = 0; c < 4; c++) {
            int d = (c + 1) % 4;
            double dx = det->p[d][0] - det->p[c][0];
            double dy = det->p[d][1] - det->p[c][1];
            double len = hypot(dx, dy);
            if (len > longest) longest = len;
        }
        jdouble *g = geom + (size_t) kept * GEOM_STRIDE;
        ids[kept * 2] = det->id;
        ids[kept * 2 + 1] = det->hamming;
        g[0] = det->c[0];
        g[1] = det->c[1];
        g[2] = longest;
        g[3] = (double) det->decision_margin;
        /* Corners always: they are already in the struct, and the flight record wants them
         * whether or not a solve runs — tools/tagcorners currently re-detects them offline. */
        for (int c = 0; c < 4; c++) {
            g[4 + c * 2] = det->p[c][0];
            g[4 + c * 2 + 1] = det->p[c][1];
        }
        for (int k = 12; k < GEOM_STRIDE; k++) g[k] = NAN;
        if (tagSizeM > 0.0 && fx > 0.0 && fy > 0.0) {
            apriltag_detection_info_t info = {
                .det = det, .tagsize = tagSizeM, .fx = fx, .fy = fy, .cx = cx, .cy = cy,
            };
            double e1 = 0.0, e2 = 0.0;
            apriltag_pose_t p1 = {0}, p2 = {0};
            /* 50 iterations is upstream's own choice in estimate_tag_pose(); calling the
             * two-solution form instead is the whole point — the loser's error is the
             * ambiguity evidence the Kotlin gate needs. */
            estimate_tag_pose_orthogonal_iteration(&info, &e1, &p1, &e2, &p2, 50);
            /* p2.R is NULL and e2 HUGE_VAL when no second minimum exists. Sort so the best
             * solution's R/t and error are always in the same slots. */
            apriltag_pose_t *best = (p2.R && e2 < e1) ? &p2 : &p1;
            double errBest = (p2.R && e2 < e1) ? e2 : e1;
            double errOther = (p2.R && e2 < e1) ? e1 : (p2.R ? e2 : INFINITY);
            for (int r = 0; r < 3; r++) {
                for (int col = 0; col < 3; col++) {
                    g[12 + r * 3 + col] = MATD_EL(best->R, r, col);
                }
                g[21 + r] = MATD_EL(best->t, r, 0);
            }
            g[24] = errBest;
            g[25] = errOther;
            matd_destroy(p1.R);
            matd_destroy(p1.t);
            if (p2.R) {
                matd_destroy(p2.R);
                matd_destroy(p2.t);
            }
        }
        kept++;
    }
    (*env)->ReleaseIntArrayElements(env, idsOut, ids, 0);
    (*env)->ReleaseDoubleArrayElements(env, geomOut, geom, 0);

    apriltag_detections_destroy(detections);
    return kept;
}

EXPORT JNIEXPORT void JNICALL
Java_com_dimensional_mini4pro_vision_AprilTagDetector_nativeDestroy(
        JNIEnv *env, jclass clazz, jlong handle)
{
    (void) env; (void) clazz;
    probe_t *p = (probe_t *) (intptr_t) handle;
    if (!p) return;
    if (p->img) image_u8_destroy(p->img);
    if (p->td) apriltag_detector_destroy(p->td);
    if (p->tf) tag36h11_destroy(p->tf);
    free(p);
}
