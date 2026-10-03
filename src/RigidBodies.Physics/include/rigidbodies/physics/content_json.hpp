#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

namespace rigidbodies::physics::content
{
    // Small, dependency-free document DOM. Numbers are finite IEEE doubles, as are physics Real
    // values. Typed access rejects mismatches; parsing and writing publish only complete results.
    class Json
    {
    public:
        using Array = std::vector<Json>;
        using Object = std::map<std::string, Json, std::less<>>;

        Json() = default;
        Json(std::nullptr_t);
        Json(bool value);
        template <typename Number, std::enable_if_t<std::is_arithmetic_v<Number> && !std::is_same_v<Number, bool>, int> = 0>
        Json(Number value) : value_(static_cast<double>(value))
        {
        }
        Json(const char* value);
        Json(std::string value);
        Json(std::string_view value);
        Json(Array value);
        Json(Object value);

        [[nodiscard]] bool is_null() const;
        [[nodiscard]] bool is_bool() const;
        [[nodiscard]] bool is_number() const;
        [[nodiscard]] bool is_string() const;
        [[nodiscard]] bool is_array() const;
        [[nodiscard]] bool is_object() const;
        [[nodiscard]] bool as_bool() const;
        [[nodiscard]] double as_number() const;
        [[nodiscard]] const std::string& as_string() const;
        [[nodiscard]] std::string& as_string();
        [[nodiscard]] const Array& as_array() const;
        [[nodiscard]] Array& as_array();
        [[nodiscard]] const Object& as_object() const;
        [[nodiscard]] Object& as_object();
        [[nodiscard]] const Json* find(std::string_view key) const;
        [[nodiscard]] Json* find(std::string_view key);
        [[nodiscard]] const Json& at(std::string_view key) const;
        [[nodiscard]] Json& at(std::string_view key);
        Json& operator[](std::string key);

    private:
        std::variant<std::nullptr_t, bool, double, std::string, Array, Object> value_ { nullptr };
    };

    struct JsonLimits
    {
        std::size_t max_bytes { 16 * 1024 * 1024 };
        std::size_t max_depth { 64 };
        std::size_t max_values { 1000000 };
        std::size_t max_string_bytes { 1024 * 1024 };
    };

    [[nodiscard]] bool parse_json(std::string_view text, Json& result, std::string& error, const JsonLimits& limits = {});
    [[nodiscard]] bool write_json(const Json& value, std::string& result, std::string& error, const JsonLimits& limits = {});
    // Convenience for a validated document. Throws std::runtime_error if the DOM is not writable.
    [[nodiscard]] std::string write_json(const Json& value);

    // Schema 1 accepts later minor revisions and unknown additive fields. A changed major is
    // deliberately rejected before geometry or world state is constructed.
    [[nodiscard]] bool validate_document_header(const Json& root, std::string_view format, std::string& error);
    [[nodiscard]] Json document_version();
}
