// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include "camera_conversion.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace dimos::booster::camera {
namespace {

struct CompressedDepthHeader {
    int32_t format;
    float depth_params[2];
};

static_assert(sizeof(CompressedDepthHeader) == 12);

struct ImageLayout {
    size_t packed_step;
    size_t source_step;
};

std::string lowercase(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

std::string trim(std::string value) {
    const auto is_not_space = [](unsigned char character) {
        return std::isspace(character) == 0;
    };
    const auto first = std::find_if(value.begin(), value.end(), is_not_space);
    const auto last = std::find_if(value.rbegin(), value.rend(), is_not_space).base();
    if (first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::string canonical_encoding(const std::string& source) {
    const std::string normalized = lowercase(trim(source));
    if (normalized == "rgb8" || normalized == "bgr8" || normalized == "rgba8" ||
        normalized == "bgra8" || normalized == "mono8" || normalized == "mono16") {
        return normalized;
    }
    if (normalized == "16uc1") {
        return "16UC1";
    }
    if (normalized == "32fc1") {
        return "32FC1";
    }
    throw std::runtime_error("unsupported Booster image encoding '" + source + "'");
}

size_t checked_product(size_t left, size_t right, const char* description) {
    if (right != 0 && left > std::numeric_limits<size_t>::max() / right) {
        throw std::runtime_error(std::string(description) + " exceeds addressable memory");
    }
    return left * right;
}

ImageLayout validate_layout(
    uint32_t width,
    uint32_t height,
    size_t bytes_per_pixel,
    uint32_t source_step_value,
    const std::vector<uint8_t>& source_data) {
    if (width == 0 || height == 0) {
        throw std::runtime_error(
            "invalid Booster image dimensions " + std::to_string(width) + "x" +
            std::to_string(height));
    }
    if (width > static_cast<uint32_t>(std::numeric_limits<int32_t>::max()) ||
        height > static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
        throw std::runtime_error("Booster image dimensions exceed DimOS message limits");
    }

    const size_t packed_step = checked_product(width, bytes_per_pixel, "packed image row");
    if (packed_step > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("packed image row exceeds the DimOS step field");
    }
    const size_t source_step = source_step_value == 0 ? packed_step : source_step_value;
    if (source_step < packed_step) {
        throw std::runtime_error(
            "Booster image step " + std::to_string(source_step) +
            " is shorter than packed row " + std::to_string(packed_step));
    }

    const size_t required_size = checked_product(height, source_step, "image payload");
    if (source_data.size() < required_size) {
        throw std::runtime_error(
            "Booster image has " + std::to_string(source_data.size()) +
            " bytes, expected at least " + std::to_string(required_size));
    }
    return {packed_step, source_step};
}

uint16_t read_uint16(const uint8_t* source, bool big_endian) {
    if (big_endian) {
        return static_cast<uint16_t>(
            (static_cast<uint16_t>(source[0]) << 8U) | source[1]);
    }
    return static_cast<uint16_t>(source[0] | (static_cast<uint16_t>(source[1]) << 8U));
}

uint32_t read_uint32(const uint8_t* source, bool big_endian) {
    if (big_endian) {
        return (static_cast<uint32_t>(source[0]) << 24U) |
               (static_cast<uint32_t>(source[1]) << 16U) |
               (static_cast<uint32_t>(source[2]) << 8U) |
               static_cast<uint32_t>(source[3]);
    }
    return static_cast<uint32_t>(source[0]) |
           (static_cast<uint32_t>(source[1]) << 8U) |
           (static_cast<uint32_t>(source[2]) << 16U) |
           (static_cast<uint32_t>(source[3]) << 24U);
}

void write_uint16_little_endian(uint16_t value, uint8_t* destination) {
    destination[0] = static_cast<uint8_t>(value & 0xFFU);
    destination[1] = static_cast<uint8_t>(value >> 8U);
}

void write_uint32_little_endian(uint32_t value, uint8_t* destination) {
    destination[0] = static_cast<uint8_t>(value & 0xFFU);
    destination[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    destination[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    destination[3] = static_cast<uint8_t>(value >> 24U);
}

void write_float_little_endian(float value, uint8_t* destination) {
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    write_uint32_little_endian(bits, destination);
}

void validate_depth_scale(double depth_scale) {
    if (!std::isfinite(depth_scale) || depth_scale <= 0.0) {
        throw std::runtime_error("depth_scale must be finite and positive");
    }
}

ConvertedImage make_depth_result(uint32_t width, uint32_t height) {
    constexpr uint32_t kFloatBytes = sizeof(float);
    const size_t step = checked_product(width, kFloatBytes, "depth image row");
    if (step > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error("depth image row exceeds the DimOS step field");
    }
    const size_t size = checked_product(height, step, "depth image payload");
    if (size > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
        throw std::runtime_error("depth image payload exceeds DimOS message limits");
    }
    return {
        width,
        height,
        static_cast<uint32_t>(step),
        "32FC1",
        std::vector<uint8_t>(size),
    };
}

}  // namespace

ConvertedImage convert_color_image(
    uint32_t width,
    uint32_t height,
    const std::string& source_encoding,
    bool is_big_endian,
    uint32_t source_step_value,
    const std::vector<uint8_t>& source_data) {
    const std::string encoding = canonical_encoding(source_encoding);
    size_t bytes_per_pixel;
    if (encoding == "mono8") {
        bytes_per_pixel = 1;
    } else if (encoding == "mono16") {
        bytes_per_pixel = 2;
    } else if (encoding == "rgb8" || encoding == "bgr8") {
        bytes_per_pixel = 3;
    } else if (encoding == "rgba8" || encoding == "bgra8") {
        bytes_per_pixel = 4;
    } else {
        throw std::runtime_error("unsupported Booster color encoding '" + source_encoding + "'");
    }

    const ImageLayout layout =
        validate_layout(width, height, bytes_per_pixel, source_step_value, source_data);
    const size_t output_size = checked_product(height, layout.packed_step, "color image payload");
    if (output_size > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
        throw std::runtime_error("color image payload exceeds DimOS message limits");
    }
    ConvertedImage result{
        width,
        height,
        static_cast<uint32_t>(layout.packed_step),
        encoding,
        std::vector<uint8_t>(output_size),
    };

    for (uint32_t row = 0; row < height; ++row) {
        const uint8_t* source_row = source_data.data() + row * layout.source_step;
        uint8_t* destination_row = result.data.data() + row * layout.packed_step;
        if (encoding == "mono16") {
            for (uint32_t column = 0; column < width; ++column) {
                write_uint16_little_endian(
                    read_uint16(source_row + column * 2U, is_big_endian),
                    destination_row + column * 2U);
            }
        } else {
            std::copy_n(source_row, layout.packed_step, destination_row);
        }
    }
    return result;
}

ConvertedImage convert_depth_image(
    uint32_t width,
    uint32_t height,
    const std::string& source_encoding,
    bool is_big_endian,
    uint32_t source_step_value,
    const std::vector<uint8_t>& source_data,
    double depth_scale) {
    validate_depth_scale(depth_scale);
    const std::string encoding = canonical_encoding(source_encoding);
    if (encoding != "16UC1" && encoding != "32FC1") {
        throw std::runtime_error("unsupported Booster depth encoding '" + source_encoding + "'");
    }

    const size_t bytes_per_pixel = encoding == "16UC1" ? 2U : 4U;
    const ImageLayout layout =
        validate_layout(width, height, bytes_per_pixel, source_step_value, source_data);
    ConvertedImage result = make_depth_result(width, height);

    for (uint32_t row = 0; row < height; ++row) {
        const uint8_t* source_row = source_data.data() + row * layout.source_step;
        uint8_t* destination_row = result.data.data() + row * result.step;
        for (uint32_t column = 0; column < width; ++column) {
            uint8_t* destination = destination_row + column * sizeof(float);
            if (encoding == "16UC1") {
                const float meters = static_cast<float>(
                    read_uint16(source_row + column * 2U, is_big_endian) * depth_scale);
                write_float_little_endian(meters, destination);
            } else {
                write_uint32_little_endian(
                    read_uint32(source_row + column * 4U, is_big_endian), destination);
            }
        }
    }
    return result;
}

ConvertedImage convert_compressed_depth_image(
    const std::string& source_format,
    const std::vector<uint8_t>& source_data,
    double depth_scale) {
    validate_depth_scale(depth_scale);
    const size_t separator = source_format.find(';');
    if (separator == std::string::npos) {
        throw std::runtime_error("invalid compressed depth format '" + source_format + "'");
    }

    const std::string encoding = canonical_encoding(source_format.substr(0, separator));
    const std::string transport_format = lowercase(trim(source_format.substr(separator + 1)));
    if (transport_format.find("compresseddepth") == std::string::npos ||
        transport_format.find("rvl") != std::string::npos) {
        throw std::runtime_error("unsupported compressed depth format '" + source_format + "'");
    }
    if (encoding != "16UC1" && encoding != "32FC1") {
        throw std::runtime_error("unsupported compressed depth encoding '" + encoding + "'");
    }
    if (source_data.size() <= sizeof(CompressedDepthHeader)) {
        throw std::runtime_error("compressed depth payload is missing PNG data");
    }
    const size_t png_size = source_data.size() - sizeof(CompressedDepthHeader);
    if (png_size > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("compressed depth PNG exceeds OpenCV's input size");
    }

    CompressedDepthHeader compression_header;
    std::memcpy(&compression_header, source_data.data(), sizeof(compression_header));
    const cv::Mat encoded_png(
        1,
        static_cast<int>(png_size),
        CV_8UC1,
        const_cast<uint8_t*>(source_data.data() + sizeof(compression_header)));
    const cv::Mat decoded = cv::imdecode(encoded_png, cv::IMREAD_UNCHANGED);
    if (decoded.empty() || decoded.type() != CV_16UC1) {
        throw std::runtime_error("compressed depth payload is not a uint16 PNG");
    }

    ConvertedImage result = make_depth_result(
        static_cast<uint32_t>(decoded.cols), static_cast<uint32_t>(decoded.rows));
    for (int row = 0; row < decoded.rows; ++row) {
        const uint16_t* source_row = decoded.ptr<uint16_t>(row);
        uint8_t* destination_row = result.data.data() + static_cast<size_t>(row) * result.step;
        for (int column = 0; column < decoded.cols; ++column) {
            const uint16_t source_value = source_row[column];
            float value;
            if (encoding == "16UC1") {
                value = static_cast<float>(source_value * depth_scale);
            } else if (source_value == 0) {
                value = std::numeric_limits<float>::quiet_NaN();
            } else {
                value = compression_header.depth_params[0] /
                        (static_cast<float>(source_value) - compression_header.depth_params[1]);
            }
            write_float_little_endian(
                value, destination_row + static_cast<size_t>(column) * sizeof(float));
        }
    }
    return result;
}

}  // namespace dimos::booster::camera
