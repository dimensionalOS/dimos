// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Minimal extern "C" surface over gtsam — exactly the API the gsc_pgo C++
// core (simple_pgo.cpp) uses: ISAM2 incremental update with factor removal,
// NonlinearFactorGraph with Pose3 prior/between factors, Values, and the
// diagonal / gaussian / robust-Huber noise models.
//
// Conventions:
//   - Rotation matrices are double[9], ROW-major. Translations double[3].
//   - Covariances are double[36], row-major 6x6 in gtsam Pose3 tangent order
//     [rot(3), trans(3)] — same convention the C++ core feeds
//     noiseModel::Gaussian::Covariance.
//   - Handles are opaque; each type gets its own incomplete struct so the
//     compiler catches a Graph/Values mix-up (they are still void*-shaped).
//   - No exceptions cross this boundary: every entry point is wrapped in a
//     catch-all. Constructors return NULL on failure; int-returning calls
//     return 0 on success, nonzero on failure.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gtsam_shim_isam2 gtsam_shim_isam2;
typedef struct gtsam_shim_graph gtsam_shim_graph;
typedef struct gtsam_shim_values gtsam_shim_values;
typedef struct gtsam_shim_noise gtsam_shim_noise;

// ---- Keys ------------------------------------------------------------------
// gtsam::Symbol('l', i) packing — used for location-landmark keys. Plain
// integer keys (keyframe indices) are used as-is, no helper needed.
uint64_t gtsam_shim_symbol_key(char chr, uint64_t index);

// ---- Noise models (opaque shared_ptr; free with gtsam_shim_noise_free) -----
// noiseModel::Diagonal::Variances(var[6]) — tangent order [rot(3), trans(3)].
gtsam_shim_noise* gtsam_shim_noise_diagonal_variances(const double variances[6]);
// noiseModel::Gaussian::Covariance(cov 6x6 row-major).
gtsam_shim_noise* gtsam_shim_noise_gaussian_covariance(const double covariance[36]);
// noiseModel::Robust(mEstimator::Huber(k), base). Does NOT consume `base`;
// the robust model holds its own reference.
gtsam_shim_noise* gtsam_shim_noise_robust_huber(double k, const gtsam_shim_noise* base);
void gtsam_shim_noise_free(gtsam_shim_noise* noise);

// ---- NonlinearFactorGraph ---------------------------------------------------
gtsam_shim_graph* gtsam_shim_graph_create(void);
void gtsam_shim_graph_destroy(gtsam_shim_graph* graph);
// resize(0) — what the C++ core does after committing a batch to iSAM2.
void gtsam_shim_graph_clear(gtsam_shim_graph* graph);
size_t gtsam_shim_graph_size(const gtsam_shim_graph* graph);
// PriorFactor<Pose3>(key, Pose3(R, t), noise)
int gtsam_shim_graph_add_prior_pose3(gtsam_shim_graph* graph, uint64_t key,
                                     const double r[9], const double t[3],
                                     const gtsam_shim_noise* noise);
// BetweenFactor<Pose3>(key1, key2, Pose3(R, t), noise)
int gtsam_shim_graph_add_between_pose3(gtsam_shim_graph* graph, uint64_t key1,
                                       uint64_t key2, const double r[9],
                                       const double t[3],
                                       const gtsam_shim_noise* noise);

// ---- Values -----------------------------------------------------------------
gtsam_shim_values* gtsam_shim_values_create(void);
void gtsam_shim_values_destroy(gtsam_shim_values* values);
void gtsam_shim_values_clear(gtsam_shim_values* values);
int gtsam_shim_values_insert_pose3(gtsam_shim_values* values, uint64_t key,
                                   const double r[9], const double t[3]);
// at<Pose3>(key) -> out_r/out_t. Returns false if the key is missing (or
// holds a non-Pose3 value).
bool gtsam_shim_values_at_pose3(const gtsam_shim_values* values, uint64_t key,
                                double out_r[9], double out_t[3]);

// ---- ISAM2 ------------------------------------------------------------------
// ISAM2Params{relinearizeThreshold=0.01, relinearizeSkip=1} — the exact
// configuration SimplePGO::SimplePGO uses.
gtsam_shim_isam2* gtsam_shim_isam2_create(void);
void gtsam_shim_isam2_destroy(gtsam_shim_isam2* isam2);

// update(graph, values, removeFactorIndices). On success writes a
// caller-owned copy of ISAM2Result::newFactorsIndices to
// *out_new_factor_indices (length *out_len; free with
// gtsam_shim_indices_free; NULL when empty). remove_indices may be NULL when
// n_remove is 0. Either out pointer may be NULL if the caller doesn't want
// the indices.
int gtsam_shim_isam2_update(gtsam_shim_isam2* isam2,
                            const gtsam_shim_graph* graph,
                            const gtsam_shim_values* values,
                            const uint64_t* remove_indices, size_t n_remove,
                            uint64_t** out_new_factor_indices, size_t* out_len);
// No-argument update() — the extra relinearization passes after a closure.
int gtsam_shim_isam2_update_empty(gtsam_shim_isam2* isam2);
// calculateBestEstimate() -> new Values (caller destroys).
gtsam_shim_values* gtsam_shim_isam2_calculate_best_estimate(const gtsam_shim_isam2* isam2);

void gtsam_shim_indices_free(uint64_t* indices);

#ifdef __cplusplus
}  // extern "C"
#endif
