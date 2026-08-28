// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Strict access to the config the coordinator sends on stdin. Python owns every
// default and sends every field, so parse<T>() requires them all and rejects
// unknowns.

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
template <class T, class = void>
constexpr bool has_const_validate = false;
template <class T>
constexpr bool has_const_validate<T, std::void_t<decltype(std::declval<const T&>().validate())>> =
    true;

template <class T, class = void>
constexpr bool has_any_validate = false;
template <class T>
constexpr bool has_any_validate<T, std::void_t<decltype(std::declval<T&>().validate())>> = true;

// Call config.validate() if defined. A non-const or non-void validate() is a
// compile error, so a range check is never silently skipped.
template <class T>
void validate_if_present(const T& value) {
    static_assert(has_const_validate<T> || !has_any_validate<T>,
                  "config validate() must be const: declare 'void validate() const'");
    if constexpr (has_const_validate<T>) {
        static_assert(std::is_same_v<decltype(std::declval<const T&>().validate()), void>,
                      "config validate() must return void");
        value.validate();
    }
}

// nlohmann would silently coerce true or 1.9 into an integral field.
template <class F>
void check_json_type(const nlohmann::json& value, const std::string& key) {
    const char* expected = nullptr;
    if constexpr (std::is_same_v<F, bool>) {
        expected = value.is_boolean() ? nullptr : "a boolean";
    } else if constexpr (std::is_integral_v<F>) {
        expected = value.is_number_integer() ? nullptr : "an integer";
    }
    if (expected != nullptr) {
        throw std::runtime_error("config: field '" + key + "': expected " + expected +
                                 ", got " + value.type_name());
    }
}
}  // namespace config_detail

/// Throw unless `value` is greater than zero. For a rate a module divides by,
/// where a zero or negative value yields an infinite or negative period.
inline void require_positive(double value, const char* name) {
    if (!(value > 0.0)) {
        throw std::runtime_error(std::string(name) + " must be greater than 0");
    }
}

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

    /// Read and consume one required field without aggregate field-name reflection.
    ///
    /// Use this on deployment targets whose compiler supports the SDK runtime
    /// but not the C++20 non-type template arguments required by
    /// `pfr::names_as_array`. Call `enforce_all_consumed()` after the final
    /// field to retain the same strict one-to-one config contract as parse().
    template <class T>
    T take(const std::string& key) {
        auto it = obj_.find(key);
        if (it == obj_.end()) {
            throw std::runtime_error("config: missing required field '" + key + "'");
        }
        config_detail::check_json_type<T>(*it, key);
        try {
            T value = it->template get<T>();
            consumed_.insert(key);
            return value;
        } catch (const std::exception& e) {
            throw std::runtime_error("config: field '" + key + "': " + e.what());
        }
    }

    /// Deserialize into a plain aggregate struct, enforcing the one-to-one key
    /// check (every field present, no unknowns) and the optional validate().
    template <class T>
    T parse() {
        static_assert(std::is_aggregate_v<T>,
                      "config structs must be plain aggregates (no constructors, "
                      "no base classes) so PFR can reflect their fields");
        T out{};
        constexpr auto names = pfr::names_as_array<T>();
        pfr::for_each_field(out, [&](auto& field, std::size_t i) {
            const std::string key(names[i]);
            field = take<std::decay_t<decltype(field)>>(key);
        });
        enforce_all_consumed();
        config_detail::validate_if_present(out);
        return out;
    }

private:
    nlohmann::json obj_;
    std::set<std::string> keys_;
    std::set<std::string> consumed_;
};

}  // namespace dimos::native
