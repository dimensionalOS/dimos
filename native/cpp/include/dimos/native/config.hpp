// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Strict config access over the config object the coordinator sends on stdin.
// Mirrors the Rust SDK's contract: Python owns every default and always sends
// every field, so parse<T>() requires every field to be present and rejects any
// unknown field. The C++ side never fills anything in.

#pragma once

#include <nlohmann/json.hpp>

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace dimos::native {

namespace config_detail {
// Call config.validate() if the type defines it, otherwise do nothing.
template <class T>
auto validate_if_present(const T& value, int) -> decltype(value.validate()) {
    return value.validate();
}
template <class T>
void validate_if_present(const T&, long) {}
}  // namespace config_detail

class Config {
public:
    /// `obj` is the `config` value from the stdin JSON. A JSON null (a module
    /// with no config) is treated as an empty object.
    explicit Config(nlohmann::json obj) : obj_(std::move(obj)) {
        if (obj_.is_null()) {
            obj_ = nlohmann::json::object();
        }
        if (!obj_.is_object()) {
            throw std::runtime_error(std::string("config must be a JSON object, got ") +
                                     obj_.type_name());
        }
        for (auto it = obj_.begin(); it != obj_.end(); ++it) {
            keys_.insert(it.key());
        }
    }

    /// Throw if any field Python sent was never read. This is the deny-unknown
    /// half of the one-to-one check and surfaces both typos and dead config.
    void enforce_all_consumed() const {
        std::vector<std::string> unexpected;
        for (const std::string& key : keys_) {
            if (consumed_.find(key) == consumed_.end()) {
                unexpected.push_back(key);
            }
        }
        if (!unexpected.empty()) {
            std::string msg = "config: unexpected field(s):";
            for (const std::string& key : unexpected) {
                msg += " '" + key + "'";
            }
            throw std::runtime_error(msg);
        }
    }

    /// Deserialize the whole config into a struct declared with DIMOS_NATIVE_CONFIG,
    /// enforcing the one-to-one key check (every field present, no unknowns) and the
    /// struct's optional validate(). Python owns defaults, so a missing field errors.
    template <class T>
    T parse() {
        T out;
        try {
            out = obj_.get<T>();
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("config: ") + e.what());
        }
        // Expected keys are the struct's own fields, recovered by re-serializing.
        // Anything the module sent that isn't one of them is an unknown field.
        nlohmann::json expected = out;
        for (auto it = expected.begin(); it != expected.end(); ++it) {
            consumed_.insert(it.key());
        }
        enforce_all_consumed();
        config_detail::validate_if_present(out, 0);
        return out;
    }

private:
    nlohmann::json obj_;
    std::set<std::string> keys_;
    std::set<std::string> consumed_;
};

}  // namespace dimos::native

// Declare a config struct's fields once (mirrors Rust's #[native_config]).
// Generates the JSON (de)serialization Config::parse<T>() uses. Every listed
// field is required. Add a void validate() const method for range checks.
#define DIMOS_NATIVE_CONFIG(Type, ...) NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Type, __VA_ARGS__)

// List form of DIMOS_NATIVE_CONFIG for structs past nlohmann's 64-field macro
// limit. FIELDS is an X-macro applying F(name) to every field:
//
//   #define MY_CONFIG_FIELDS(F) F(host_ip) F(frequency) ...
//   DIMOS_NATIVE_CONFIG_LIST(MyConfig, MY_CONFIG_FIELDS)
//
// Same contract: every listed field is required, unknown fields are rejected.
#define DIMOS_NATIVE_CONFIG_FIELD_GET(name) j.at(#name).get_to(c.name);
#define DIMOS_NATIVE_CONFIG_FIELD_PUT(name) j[#name] = c.name;
#define DIMOS_NATIVE_CONFIG_LIST(Type, FIELDS)                \
    inline void from_json(const nlohmann::json& j, Type& c) { \
        FIELDS(DIMOS_NATIVE_CONFIG_FIELD_GET)                 \
    }                                                         \
    inline void to_json(nlohmann::json& j, const Type& c) {   \
        j = nlohmann::json::object();                         \
        FIELDS(DIMOS_NATIVE_CONFIG_FIELD_PUT)                 \
    }
