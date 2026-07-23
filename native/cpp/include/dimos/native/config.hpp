// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Strict config access over the config object the coordinator sends on stdin.
// Mirrors the Rust SDK's contract: Python owns every default and always sends
// every field, so parse<T>() requires every field to be present and rejects any
// unknown field. The C++ side never fills anything in.
//
// A config is a plain aggregate struct; parse<T>() reflects over its fields
// with PFR (mirrors Rust's #[native_config] on a plain struct), so the struct
// declaration is the whole contract. Any number of fields, no macros.

#pragma once

#include <nlohmann/json.hpp>
#include <pfr.hpp>

#include <cstddef>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
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

    /// Deserialize the whole config into a plain aggregate struct, enforcing
    /// the one-to-one key check (every field present, no unknowns) and the
    /// struct's optional validate(). Python owns defaults, so a missing field
    /// errors.
    template <class T>
    T parse() {
        static_assert(std::is_aggregate_v<T>,
                      "config structs must be plain aggregates (no constructors, "
                      "no base classes) so PFR can reflect their fields");
        T out{};
        constexpr auto names = pfr::names_as_array<T>();
        pfr::for_each_field(out, [&](auto& field, std::size_t i) {
            const std::string key(names[i]);
            auto it = obj_.find(key);
            if (it == obj_.end()) {
                throw std::runtime_error("config: missing required field '" + key + "'");
            }
            try {
                field = it->template get<std::decay_t<decltype(field)>>();
            } catch (const std::exception& e) {
                throw std::runtime_error("config: field '" + key + "': " + e.what());
            }
            consumed_.insert(key);
        });
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
