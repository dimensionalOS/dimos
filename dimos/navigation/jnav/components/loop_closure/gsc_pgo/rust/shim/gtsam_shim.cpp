// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the extern "C" gtsam shim. See gtsam_shim.h for the API
// contract (row-major layouts, ownership, no exceptions across the boundary).

#include "gtsam_shim.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

// The handle types are incomplete structs on the C side; on this side they
// alias the real gtsam objects (noise handles wrap the shared_ptr itself so
// robust models can share the base).
gtsam::ISAM2* unwrap(gtsam_shim_isam2* h) { return reinterpret_cast<gtsam::ISAM2*>(h); }
const gtsam::ISAM2* unwrap(const gtsam_shim_isam2* h) { return reinterpret_cast<const gtsam::ISAM2*>(h); }
gtsam::NonlinearFactorGraph* unwrap(gtsam_shim_graph* h) { return reinterpret_cast<gtsam::NonlinearFactorGraph*>(h); }
const gtsam::NonlinearFactorGraph* unwrap(const gtsam_shim_graph* h) { return reinterpret_cast<const gtsam::NonlinearFactorGraph*>(h); }
gtsam::Values* unwrap(gtsam_shim_values* h) { return reinterpret_cast<gtsam::Values*>(h); }
const gtsam::Values* unwrap(const gtsam_shim_values* h) { return reinterpret_cast<const gtsam::Values*>(h); }
gtsam::SharedNoiseModel* unwrap(gtsam_shim_noise* h) { return reinterpret_cast<gtsam::SharedNoiseModel*>(h); }
const gtsam::SharedNoiseModel* unwrap(const gtsam_shim_noise* h) { return reinterpret_cast<const gtsam::SharedNoiseModel*>(h); }

gtsam_shim_noise* wrap(gtsam::SharedNoiseModel* p) { return reinterpret_cast<gtsam_shim_noise*>(p); }
gtsam_shim_values* wrap(gtsam::Values* p) { return reinterpret_cast<gtsam_shim_values*>(p); }

gtsam::Pose3 make_pose3(const double r[9], const double t[3]) {
    gtsam::Matrix3 rotation;
    // Input is row-major; Eigen comma-init fills row by row.
    rotation << r[0], r[1], r[2],
                r[3], r[4], r[5],
                r[6], r[7], r[8];
    return gtsam::Pose3(gtsam::Rot3(rotation), gtsam::Point3(t[0], t[1], t[2]));
}

void write_pose3(const gtsam::Pose3& pose, double out_r[9], double out_t[3]) {
    const gtsam::Matrix3 rotation = pose.rotation().matrix();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out_r[i * 3 + j] = rotation(i, j);
        }
    }
    const gtsam::Point3 translation = pose.translation();
    out_t[0] = translation.x();
    out_t[1] = translation.y();
    out_t[2] = translation.z();
}

}  // namespace

extern "C" {

// ---- Keys ------------------------------------------------------------------

uint64_t gtsam_shim_symbol_key(char chr, uint64_t index) {
    return static_cast<uint64_t>(gtsam::Symbol(static_cast<unsigned char>(chr), index).key());
}

// ---- Noise models ------------------------------------------------------------

gtsam_shim_noise* gtsam_shim_noise_diagonal_variances(const double variances[6]) {
    try {
        gtsam::Vector6 v;
        for (int i = 0; i < 6; i++) {
            v(i) = variances[i];
        }
        return wrap(new gtsam::SharedNoiseModel(gtsam::noiseModel::Diagonal::Variances(v)));
    } catch (...) {
        return nullptr;
    }
}

gtsam_shim_noise* gtsam_shim_noise_gaussian_covariance(const double covariance[36]) {
    try {
        gtsam::Matrix6 cov;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                cov(i, j) = covariance[i * 6 + j];
            }
        }
        return wrap(new gtsam::SharedNoiseModel(gtsam::noiseModel::Gaussian::Covariance(cov)));
    } catch (...) {
        return nullptr;
    }
}

gtsam_shim_noise* gtsam_shim_noise_robust_huber(double k, const gtsam_shim_noise* base) {
    if (base == nullptr) {
        return nullptr;
    }
    try {
        return wrap(new gtsam::SharedNoiseModel(gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(k), *unwrap(base))));
    } catch (...) {
        return nullptr;
    }
}

void gtsam_shim_noise_free(gtsam_shim_noise* noise) {
    delete unwrap(noise);
}

// ---- NonlinearFactorGraph -----------------------------------------------------

gtsam_shim_graph* gtsam_shim_graph_create(void) {
    try {
        return reinterpret_cast<gtsam_shim_graph*>(new gtsam::NonlinearFactorGraph());
    } catch (...) {
        return nullptr;
    }
}

void gtsam_shim_graph_destroy(gtsam_shim_graph* graph) {
    delete unwrap(graph);
}

void gtsam_shim_graph_clear(gtsam_shim_graph* graph) {
    if (graph != nullptr) {
        unwrap(graph)->resize(0);
    }
}

size_t gtsam_shim_graph_size(const gtsam_shim_graph* graph) {
    return graph != nullptr ? unwrap(graph)->size() : 0;
}

int gtsam_shim_graph_add_prior_pose3(gtsam_shim_graph* graph, uint64_t key,
                                     const double r[9], const double t[3],
                                     const gtsam_shim_noise* noise) {
    if (graph == nullptr || noise == nullptr) {
        return 1;
    }
    try {
        unwrap(graph)->add(gtsam::PriorFactor<gtsam::Pose3>(key, make_pose3(r, t), *unwrap(noise)));
        return 0;
    } catch (...) {
        return 1;
    }
}

int gtsam_shim_graph_add_between_pose3(gtsam_shim_graph* graph, uint64_t key1,
                                       uint64_t key2, const double r[9],
                                       const double t[3],
                                       const gtsam_shim_noise* noise) {
    if (graph == nullptr || noise == nullptr) {
        return 1;
    }
    try {
        unwrap(graph)->add(
            gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, make_pose3(r, t), *unwrap(noise)));
        return 0;
    } catch (...) {
        return 1;
    }
}

// ---- Values -------------------------------------------------------------------

gtsam_shim_values* gtsam_shim_values_create(void) {
    try {
        return wrap(new gtsam::Values());
    } catch (...) {
        return nullptr;
    }
}

void gtsam_shim_values_destroy(gtsam_shim_values* values) {
    delete unwrap(values);
}

void gtsam_shim_values_clear(gtsam_shim_values* values) {
    if (values != nullptr) {
        unwrap(values)->clear();
    }
}

int gtsam_shim_values_insert_pose3(gtsam_shim_values* values, uint64_t key,
                                   const double r[9], const double t[3]) {
    if (values == nullptr) {
        return 1;
    }
    try {
        unwrap(values)->insert(key, make_pose3(r, t));
        return 0;
    } catch (...) {
        return 1;
    }
}

bool gtsam_shim_values_at_pose3(const gtsam_shim_values* values, uint64_t key,
                                double out_r[9], double out_t[3]) {
    if (values == nullptr) {
        return false;
    }
    try {
        write_pose3(unwrap(values)->at<gtsam::Pose3>(key), out_r, out_t);
        return true;
    } catch (...) {
        // Missing key / wrong value type both surface as gtsam exceptions.
        return false;
    }
}

// ---- ISAM2 ----------------------------------------------------------------------

gtsam_shim_isam2* gtsam_shim_isam2_create(void) {
    try {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        return reinterpret_cast<gtsam_shim_isam2*>(new gtsam::ISAM2(params));
    } catch (...) {
        return nullptr;
    }
}

void gtsam_shim_isam2_destroy(gtsam_shim_isam2* isam2) {
    delete unwrap(isam2);
}

int gtsam_shim_isam2_update(gtsam_shim_isam2* isam2, const gtsam_shim_graph* graph,
                            const gtsam_shim_values* values,
                            const uint64_t* remove_indices, size_t n_remove,
                            uint64_t** out_new_factor_indices, size_t* out_len) {
    if (out_new_factor_indices != nullptr) {
        *out_new_factor_indices = nullptr;
    }
    if (out_len != nullptr) {
        *out_len = 0;
    }
    if (isam2 == nullptr || graph == nullptr || values == nullptr) {
        return 1;
    }
    if (remove_indices == nullptr && n_remove != 0) {
        return 1;
    }
    try {
        // gtsam::FactorIndex is std::uint64_t, so the copy is 1:1.
        gtsam::FactorIndices remove(remove_indices, remove_indices + n_remove);
        gtsam::ISAM2Result result = unwrap(isam2)->update(*unwrap(graph), *unwrap(values), remove);
        if (out_new_factor_indices != nullptr && out_len != nullptr &&
            !result.newFactorsIndices.empty()) {
            const size_t n = result.newFactorsIndices.size();
            uint64_t* copy = static_cast<uint64_t*>(std::malloc(n * sizeof(uint64_t)));
            if (copy == nullptr) {
                return 2;
            }
            for (size_t i = 0; i < n; i++) {
                copy[i] = static_cast<uint64_t>(result.newFactorsIndices[i]);
            }
            *out_new_factor_indices = copy;
            *out_len = n;
        }
        return 0;
    } catch (...) {
        return 1;
    }
}

int gtsam_shim_isam2_update_empty(gtsam_shim_isam2* isam2) {
    if (isam2 == nullptr) {
        return 1;
    }
    try {
        unwrap(isam2)->update();
        return 0;
    } catch (...) {
        return 1;
    }
}

gtsam_shim_values* gtsam_shim_isam2_calculate_best_estimate(const gtsam_shim_isam2* isam2) {
    if (isam2 == nullptr) {
        return nullptr;
    }
    try {
        return wrap(new gtsam::Values(unwrap(isam2)->calculateBestEstimate()));
    } catch (...) {
        return nullptr;
    }
}

void gtsam_shim_indices_free(uint64_t* indices) {
    std::free(indices);
}

}  // extern "C"
