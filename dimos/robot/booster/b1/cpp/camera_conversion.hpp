// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace dimos::booster::camera {

struct ConvertedImage {
    uint32_t width;
    uint32_t height;
    uint32_t step;
    std::string encoding;
    std::vector<uint8_t> data;
};

ConvertedImage convert_color_image(
    uint32_t width,
    uint32_t height,
    const std::string& encoding,
    bool is_big_endian,
    uint32_t source_step,
    const std::vector<uint8_t>& source_data);

ConvertedImage convert_depth_image(
    uint32_t width,
    uint32_t height,
    const std::string& encoding,
    bool is_big_endian,
    uint32_t source_step,
    const std::vector<uint8_t>& source_data,
    double depth_scale);

ConvertedImage convert_compressed_depth_image(
    const std::string& format,
    const std::vector<uint8_t>& source_data,
    double depth_scale);

}  // namespace dimos::booster::camera
