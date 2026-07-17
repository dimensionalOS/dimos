// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include "camera_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace {

using dimos::booster::camera::ConvertedImage;
using dimos::booster::camera::convert_color_image;
using dimos::booster::camera::convert_compressed_depth_image;
using dimos::booster::camera::convert_depth_image;

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

float read_float_little_endian(const std::vector<uint8_t>& data, size_t offset) {
    const uint32_t bits = static_cast<uint32_t>(data[offset]) |
                          (static_cast<uint32_t>(data[offset + 1]) << 8U) |
                          (static_cast<uint32_t>(data[offset + 2]) << 16U) |
                          (static_cast<uint32_t>(data[offset + 3]) << 24U);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void expect_float(float actual, float expected, const std::string& message) {
    if (std::isnan(expected)) {
        expect(std::isnan(actual), message);
        return;
    }
    expect(std::abs(actual - expected) < 1e-6F, message);
}

void expect_throws(const std::function<void()>& operation, const std::string& expected_message) {
    try {
        operation();
    } catch (const std::runtime_error& error) {
        expect(
            std::string(error.what()).find(expected_message) != std::string::npos,
            "unexpected error: " + std::string(error.what()));
        return;
    }
    throw std::runtime_error("operation did not throw: " + expected_message);
}

void test_color_rows_are_packed_without_padding() {
    const std::vector<uint8_t> source{
        1, 2, 3, 4, 5, 6, 99, 99,
        7, 8, 9, 10, 11, 12, 88, 88,
    };

    const ConvertedImage result = convert_color_image(2, 2, "rgb8", false, 8, source);

    expect(result.width == 2 && result.height == 2, "color dimensions changed");
    expect(result.step == 6, "color step was not packed");
    expect(result.encoding == "rgb8", "color encoding changed");
    expect(
        result.data == std::vector<uint8_t>({1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}),
        "color padding was not removed");
}

void test_big_endian_mono16_is_normalized_to_little_endian() {
    const ConvertedImage result =
        convert_color_image(2, 1, "mono16", true, 4, {0x12, 0x34, 0xAB, 0xCD});

    expect(result.data == std::vector<uint8_t>({0x34, 0x12, 0xCD, 0xAB}), "mono16 byte order");
}

void test_depth_rows_are_scaled_and_packed() {
    const std::vector<uint8_t> source{
        0xE8, 0x03, 0xFA, 0x00, 99, 99,
        0x00, 0x00, 0xD0, 0x07, 88, 88,
    };

    const ConvertedImage result = convert_depth_image(2, 2, "16UC1", false, 6, source, 0.001);

    expect(result.step == 8, "depth step was not converted to float32");
    expect(result.encoding == "32FC1", "depth encoding was not converted to meters");
    expect_float(read_float_little_endian(result.data, 0), 1.0F, "first depth value");
    expect_float(read_float_little_endian(result.data, 4), 0.25F, "second depth value");
    expect_float(read_float_little_endian(result.data, 8), 0.0F, "third depth value");
    expect_float(read_float_little_endian(result.data, 12), 2.0F, "fourth depth value");
}

void test_short_depth_step_is_rejected_before_reading_pixels() {
    expect_throws(
        [] { convert_depth_image(2, 1, "32FC1", false, 4, {0, 0, 0, 0}, 0.001); },
        "shorter than packed row");
}

void test_big_endian_float_depth_preserves_values() {
    const ConvertedImage result =
        convert_depth_image(1, 1, "32FC1", true, 4, {0x3F, 0xA0, 0x00, 0x00}, 0.001);

    expect_float(read_float_little_endian(result.data, 0), 1.25F, "big-endian float depth");
}

void test_compressed_depth_reverses_float_quantization() {
    const cv::Mat inverse_depth = (cv::Mat_<uint16_t>(1, 3) << 50, 0, 25);
    std::vector<uint8_t> png;
    expect(cv::imencode(".png", inverse_depth, png), "failed to encode PNG fixture");

    struct Header {
        int32_t format;
        float depth_params[2];
    } header{0, {100.0F, 0.0F}};
    static_assert(sizeof(header) == 12);
    std::vector<uint8_t> payload(sizeof(header) + png.size());
    std::memcpy(payload.data(), &header, sizeof(header));
    std::copy(png.begin(), png.end(), payload.begin() + sizeof(header));

    const ConvertedImage result =
        convert_compressed_depth_image("32FC1; compressedDepth png", payload, 0.001);

    expect_float(read_float_little_endian(result.data, 0), 2.0F, "quantized first depth");
    expect_float(
        read_float_little_endian(result.data, sizeof(float)),
        std::numeric_limits<float>::quiet_NaN(),
        "quantized invalid depth");
    expect_float(read_float_little_endian(result.data, 8), 4.0F, "quantized third depth");
}

}  // namespace

int main() {
    try {
        test_color_rows_are_packed_without_padding();
        test_big_endian_mono16_is_normalized_to_little_endian();
        test_depth_rows_are_scaled_and_packed();
        test_short_depth_step_is_rejected_before_reading_pixels();
        test_big_endian_float_depth_preserves_values();
        test_compressed_depth_reverses_float_quantization();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "camera conversion test failed: " << error.what() << std::endl;
        return 1;
    }
}
