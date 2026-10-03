#include <rigidbodies/physics/content_json.hpp>

#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rigidbodies::physics::content
{
    namespace
    {
        template <typename T, typename Variant>
        auto& typed(Variant& value, const char* type)
        {
            const auto pointer = std::get_if<T>(&value);
            if (pointer == nullptr)
            {
                throw std::runtime_error(std::string("Expected JSON ") + type + ".");
            }
            return *pointer;
        }

        // Validate UTF-8 scalars, rejecting overlong sequences, surrogates, and values beyond
        // U+10FFFF. The original byte sequence is retained, without lossy locale conversion.
        std::size_t utf8_length(std::string_view input, std::size_t offset)
        {
            const auto first = static_cast<unsigned char>(input[offset]);
            if (first < 0x80)
            {
                return 1;
            }
            const std::size_t length = first >= 0xc2 && first <= 0xdf ? 2 : first >= 0xe0 && first <= 0xef ? 3
                : first >= 0xf0 && first <= 0xf4                                                           ? 4
                                                                                                           : 0;
            if (length == 0 || length > input.size() - offset)
            {
                throw std::runtime_error("Invalid UTF-8 string.");
            }
            unsigned scalar = first & (length == 2 ? 0x1fu : length == 3 ? 0x0fu
                                                                         : 0x07u);
            for (std::size_t i = 1; i < length; ++i)
            {
                const auto next = static_cast<unsigned char>(input[offset + i]);
                if ((next & 0xc0u) != 0x80u)
                {
                    throw std::runtime_error("Invalid UTF-8 continuation.");
                }
                scalar = (scalar << 6u) | (next & 0x3fu);
            }
            if ((length == 2 && scalar < 0x80u) || (length == 3 && scalar < 0x800u) || (length == 4 && scalar < 0x10000u) ||
                (scalar >= 0xd800u && scalar <= 0xdfffu) || scalar > 0x10ffffu)
            {
                throw std::runtime_error("Invalid UTF-8 scalar.");
            }
            return length;
        }

        void append_utf8(std::string& output, unsigned scalar)
        {
            if (scalar < 0x80u)
            {
                output.push_back(static_cast<char>(scalar));
            }
            else if (scalar < 0x800u)
            {
                output.push_back(static_cast<char>(0xc0u | (scalar >> 6u)));
                output.push_back(static_cast<char>(0x80u | (scalar & 0x3fu)));
            }
            else if (scalar < 0x10000u)
            {
                output.push_back(static_cast<char>(0xe0u | (scalar >> 12u)));
                output.push_back(static_cast<char>(0x80u | ((scalar >> 6u) & 0x3fu)));
                output.push_back(static_cast<char>(0x80u | (scalar & 0x3fu)));
            }
            else
            {
                output.push_back(static_cast<char>(0xf0u | (scalar >> 18u)));
                output.push_back(static_cast<char>(0x80u | ((scalar >> 12u) & 0x3fu)));
                output.push_back(static_cast<char>(0x80u | ((scalar >> 6u) & 0x3fu)));
                output.push_back(static_cast<char>(0x80u | (scalar & 0x3fu)));
            }
        }

        class Parser
        {
        public:
            Parser(std::string_view text, const JsonLimits& limits) : text_(text), limits_(limits)
            {
            }
            Json run()
            {
                if (text_.size() > limits_.max_bytes)
                {
                    fail("document exceeds byte limit");
                }
                auto result = value(0);
                whitespace();
                if (position_ != text_.size())
                {
                    fail("trailing content");
                }
                return result;
            }

        private:
            [[noreturn]] void fail(const char* message) const
            {
                throw std::runtime_error("JSON byte " + std::to_string(position_) + ": " + message + ".");
            }
            void whitespace()
            {
                while (position_ < text_.size() && (text_[position_] == ' ' || text_[position_] == '\t' || text_[position_] == '\n' || text_[position_] == '\r'))
                {
                    ++position_;
                }
            }
            bool take(char character)
            {
                if (position_ < text_.size() && text_[position_] == character)
                {
                    ++position_;
                    return true;
                }
                return false;
            }
            void expect(char character)
            {
                if (!take(character))
                {
                    fail("unexpected token");
                }
            }
            unsigned hex_quad()
            {
                unsigned result = 0;
                for (int index = 0; index < 4; ++index)
                {
                    if (position_ == text_.size())
                    {
                        fail("incomplete Unicode escape");
                    }
                    const char ch = text_[position_++];
                    const int digit = ch >= '0' && ch <= '9' ? ch - '0' : ch >= 'a' && ch <= 'f' ? ch - 'a' + 10
                        : ch >= 'A' && ch <= 'F'                                                 ? ch - 'A' + 10
                                                                                                 : -1;
                    if (digit < 0)
                    {
                        fail("invalid Unicode escape");
                    }
                    result = result * 16u + static_cast<unsigned>(digit);
                }
                return result;
            }
            std::string string()
            {
                expect('"');
                std::string result;
                while (position_ < text_.size())
                {
                    if (take('"'))
                    {
                        return result;
                    }
                    const auto ch = static_cast<unsigned char>(text_[position_]);
                    if (ch < 0x20u)
                    {
                        fail("unescaped control in string");
                    }
                    if (take('\\'))
                    {
                        if (position_ == text_.size())
                        {
                            fail("incomplete escape");
                        }
                        switch (text_[position_++])
                        {
                        case '"':
                            result.push_back('"');
                            break;
                        case '\\':
                            result.push_back('\\');
                            break;
                        case '/':
                            result.push_back('/');
                            break;
                        case 'b':
                            result.push_back('\b');
                            break;
                        case 'f':
                            result.push_back('\f');
                            break;
                        case 'n':
                            result.push_back('\n');
                            break;
                        case 'r':
                            result.push_back('\r');
                            break;
                        case 't':
                            result.push_back('\t');
                            break;
                        case 'u':
                        {
                            auto scalar = hex_quad();
                            if (scalar >= 0xd800u && scalar <= 0xdbffu)
                            {
                                expect('\\');
                                expect('u');
                                const auto low = hex_quad();
                                if (low < 0xdc00u || low > 0xdfffu)
                                {
                                    fail("invalid low surrogate");
                                }
                                scalar = 0x10000u + ((scalar - 0xd800u) << 10u) + low - 0xdc00u;
                            }
                            else if (scalar >= 0xdc00u && scalar <= 0xdfffu)
                            {
                                fail("unpaired low surrogate");
                            }
                            append_utf8(result, scalar);
                            break;
                        }
                        default:
                            fail("unknown string escape");
                        }
                    }
                    else
                    {
                        const auto count = utf8_length(text_, position_);
                        result.append(text_.substr(position_, count));
                        position_ += count;
                    }
                    if (result.size() > limits_.max_string_bytes)
                    {
                        fail("string exceeds byte limit");
                    }
                }
                fail("unterminated string");
            }
            Json number()
            {
                const auto begin = position_;
                take('-');
                if (!take('0'))
                {
                    if (position_ == text_.size() || text_[position_] < '1' || text_[position_] > '9')
                    {
                        fail("invalid number");
                    }
                    while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9')
                    {
                        ++position_;
                    }
                }
                if (take('.'))
                {
                    digits();
                }
                if (take('e') || take('E'))
                {
                    if (!take('+'))
                    {
                        take('-');
                    }
                    digits();
                }
                double result {};
                const auto converted = std::from_chars(text_.data() + begin, text_.data() + position_, result, std::chars_format::general);
                if (converted.ec != std::errc() || converted.ptr != text_.data() + position_ || !std::isfinite(result))
                {
                    fail("number outside finite double range");
                }
                return result;
            }
            void digits()
            {
                const auto begin = position_;
                while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9')
                {
                    ++position_;
                }
                if (begin == position_)
                {
                    fail("number requires digits");
                }
            }
            Json value(std::size_t depth)
            {
                whitespace();
                if (depth > limits_.max_depth || ++values_ > limits_.max_values)
                {
                    fail("document exceeds structural limit");
                }
                if (position_ == text_.size())
                {
                    fail("missing value");
                }
                if (text_[position_] == '"')
                {
                    return string();
                }
                if (take('{'))
                {
                    Json::Object object;
                    whitespace();
                    if (take('}'))
                    {
                        return object;
                    }
                    do
                    {
                        whitespace();
                        auto key = string();
                        whitespace();
                        expect(':');
                        auto item = value(depth + 1);
                        if (!object.emplace(std::move(key), std::move(item)).second)
                        {
                            fail("duplicate object member");
                        }
                        whitespace();
                    }
                    while (take(','));
                    expect('}');
                    return object;
                }
                if (take('['))
                {
                    Json::Array array;
                    whitespace();
                    if (take(']'))
                    {
                        return array;
                    }
                    do
                    {
                        array.push_back(value(depth + 1));
                        whitespace();
                    }
                    while (take(','));
                    expect(']');
                    return array;
                }
                if (text_.substr(position_, 4) == "null")
                {
                    position_ += 4;
                    return nullptr;
                }
                if (text_.substr(position_, 4) == "true")
                {
                    position_ += 4;
                    return true;
                }
                if (text_.substr(position_, 5) == "false")
                {
                    position_ += 5;
                    return false;
                }
                return number();
            }

            std::string_view text_;
            const JsonLimits& limits_;
            std::size_t position_ {};
            std::size_t values_ {};
        };

        class Writer
        {
        public:
            explicit Writer(const JsonLimits& limits) : limits_(limits)
            {
            }
            std::string run(const Json& input)
            {
                value(input, 0);
                append("\n");
                return std::move(output_);
            }

        private:
            void append(std::string_view text)
            {
                if (text.size() > limits_.max_bytes || output_.size() > limits_.max_bytes - text.size())
                {
                    throw std::runtime_error("JSON output exceeds byte limit.");
                }
                output_.append(text);
            }
            void string(std::string_view input)
            {
                if (input.size() > limits_.max_string_bytes)
                {
                    throw std::runtime_error("JSON string exceeds byte limit.");
                }
                append("\"");
                for (std::size_t i = 0; i < input.size();)
                {
                    const auto ch = static_cast<unsigned char>(input[i]);
                    if (ch < 0x20u)
                    {
                        constexpr char hex[] = "0123456789abcdef";
                        const char escaped[] = { '\\', 'u', '0', '0', hex[ch >> 4u], hex[ch & 0xfu] };
                        append(std::string_view(escaped, sizeof(escaped)));
                        ++i;
                    }
                    else if (ch == '"' || ch == '\\')
                    {
                        append("\\");
                        append(input.substr(i++, 1));
                    }
                    else
                    {
                        const auto length = utf8_length(input, i);
                        append(input.substr(i, length));
                        i += length;
                    }
                }
                append("\"");
            }
            void value(const Json& input, std::size_t depth)
            {
                if (depth > limits_.max_depth || ++values_ > limits_.max_values)
                {
                    throw std::runtime_error("JSON output exceeds structural limit.");
                }
                if (input.is_null())
                {
                    append("null");
                }
                else if (input.is_bool())
                {
                    append(input.as_bool() ? "true" : "false");
                }
                else if (input.is_number())
                {
                    const auto number = input.as_number();
                    if (!std::isfinite(number))
                    {
                        throw std::runtime_error("JSON number must be finite.");
                    }
                    char buffer[64];
                    const auto converted = std::to_chars(buffer, buffer + sizeof(buffer), number, std::chars_format::general, std::numeric_limits<double>::max_digits10);
                    if (converted.ec != std::errc())
                    {
                        throw std::runtime_error("JSON number cannot be formatted.");
                    }
                    append(std::string_view(buffer, static_cast<std::size_t>(converted.ptr - buffer)));
                }
                else if (input.is_string())
                {
                    string(input.as_string());
                }
                else if (input.is_array())
                {
                    append("[");
                    bool first = true;
                    for (const auto& child : input.as_array())
                    {
                        if (!first)
                            append(", ");
                        first = false;
                        value(child, depth + 1);
                    }
                    append("]");
                }
                else
                {
                    append("{");
                    bool first = true;
                    for (const auto& member : input.as_object())
                    {
                        if (!first)
                            append(",");
                        append("\n");
                        append(std::string((depth + 1) * 2, ' '));
                        first = false;
                        string(member.first);
                        append(": ");
                        value(member.second, depth + 1);
                    }
                    if (!first)
                    {
                        append("\n");
                        append(std::string(depth * 2, ' '));
                    }
                    append("}");
                }
            }
            const JsonLimits& limits_;
            std::string output_;
            std::size_t values_ {};
        };
    }

    Json::Json(std::nullptr_t) : value_(nullptr)
    {
    }
    Json::Json(bool value) : value_(value)
    {
    }
    Json::Json(const char* value) : value_(std::string(value))
    {
    }
    Json::Json(std::string value) : value_(std::move(value))
    {
    }
    Json::Json(std::string_view value) : value_(std::string(value))
    {
    }
    Json::Json(Array value) : value_(std::move(value))
    {
    }
    Json::Json(Object value) : value_(std::move(value))
    {
    }
    bool Json::is_null() const
    {
        return std::holds_alternative<std::nullptr_t>(value_);
    }
    bool Json::is_bool() const
    {
        return std::holds_alternative<bool>(value_);
    }
    bool Json::is_number() const
    {
        return std::holds_alternative<double>(value_);
    }
    bool Json::is_string() const
    {
        return std::holds_alternative<std::string>(value_);
    }
    bool Json::is_array() const
    {
        return std::holds_alternative<Array>(value_);
    }
    bool Json::is_object() const
    {
        return std::holds_alternative<Object>(value_);
    }
    bool Json::as_bool() const
    {
        return typed<bool>(value_, "boolean");
    }
    double Json::as_number() const
    {
        return typed<double>(value_, "number");
    }
    const std::string& Json::as_string() const
    {
        return typed<std::string>(value_, "string");
    }
    std::string& Json::as_string()
    {
        return typed<std::string>(value_, "string");
    }
    const Json::Array& Json::as_array() const
    {
        return typed<Array>(value_, "array");
    }
    Json::Array& Json::as_array()
    {
        return typed<Array>(value_, "array");
    }
    const Json::Object& Json::as_object() const
    {
        return typed<Object>(value_, "object");
    }
    Json::Object& Json::as_object()
    {
        return typed<Object>(value_, "object");
    }
    const Json* Json::find(std::string_view key) const
    {
        if (!is_object())
            return nullptr;
        const auto found = as_object().find(key);
        return found == as_object().end() ? nullptr : &found->second;
    }
    Json* Json::find(std::string_view key)
    {
        if (!is_object())
            return nullptr;
        const auto found = as_object().find(key);
        return found == as_object().end() ? nullptr : &found->second;
    }
    const Json& Json::at(std::string_view key) const
    {
        const auto found = find(key);
        if (!found)
            throw std::runtime_error("Missing JSON member: " + std::string(key) + ".");
        return *found;
    }
    Json& Json::at(std::string_view key)
    {
        const auto found = find(key);
        if (!found)
            throw std::runtime_error("Missing JSON member: " + std::string(key) + ".");
        return *found;
    }
    Json& Json::operator[](std::string key)
    {
        if (is_null())
            value_ = Object {};
        return as_object()[std::move(key)];
    }
    bool parse_json(std::string_view text, Json& result, std::string& error, const JsonLimits& limits)
    {
        try
        {
            auto parsed = Parser(text, limits).run();
            result = std::move(parsed);
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }
    bool write_json(const Json& value, std::string& result, std::string& error, const JsonLimits& limits)
    {
        try
        {
            auto written = Writer(limits).run(value);
            result = std::move(written);
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }
    std::string write_json(const Json& value)
    {
        std::string result, error;
        if (!write_json(value, result, error))
            throw std::runtime_error(error);
        return result;
    }
    bool validate_document_header(const Json& root, std::string_view format, std::string& error)
    {
        try
        {
            if (root.at("format").as_string() != format)
                throw std::runtime_error("Unrecognized document format; expected " + std::string(format) + ".");
            const auto& version = root.at("version");
            const auto major = version.at("major").as_number(), minor = version.at("minor").as_number();
            if (major != 1.0)
                throw std::runtime_error("Unsupported document major version; this application supports version 1.");
            if (!std::isfinite(minor) || minor < 0.0 || minor > 1000000.0 || std::floor(minor) != minor)
                throw std::runtime_error("Document minor version must be a nonnegative integer.");
            if (const auto features = root.find("required_features"))
                if (!features->is_array() || !features->as_array().empty())
                    throw std::runtime_error("Document requires unsupported features.");
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }
    Json document_version()
    {
        return Json::Object { { "major", 1 }, { "minor", 1 } };
    }
}
