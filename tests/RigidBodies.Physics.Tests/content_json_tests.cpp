#include <rigidbodies/physics/benchmark.hpp>
#include <rigidbodies/physics/content_json.hpp>
#include <rigidbodies/physics/shape_document.hpp>

#include "test_framework.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    using namespace rigidbodies::physics::content;

    RIGIDBODIES_TEST("JSON document values and escaping round trip exactly")
    {
        Json value = Json::Object {
            { "null", nullptr }, { "bool", true }, { "number", -1.2345678901234567 }, { "array", Json::Array { false, 1.0e-200, 1.0e200, "quote\"slash\\\n\t" } }
        };
        std::string output, error;
        RIGIDBODIES_EXPECT(write_json(value, output, error), "finite document writes");
        Json decoded;
        RIGIDBODIES_EXPECT(parse_json(output, decoded, error), "writer produces readable JSON");
        RIGIDBODIES_EXPECT(decoded.at("null").is_null() && decoded.at("bool").as_bool(), "JSON scalar types survive");
        RIGIDBODIES_EXPECT(decoded.at("number").as_number() == value.at("number").as_number(), "double is exactly round tripped");
        RIGIDBODIES_EXPECT(write_json(decoded) == output, "writing is deterministic after reading");
    }

    RIGIDBODIES_TEST("JSON numeric conversion preserves double boundaries and signed zero")
    {
        const Json source = Json::Array { std::numeric_limits<double>::max(), std::numeric_limits<double>::min(), std::numeric_limits<double>::denorm_min(), -0.0 };
        Json decoded;
        std::string error;
        RIGIDBODIES_EXPECT(parse_json(write_json(source), decoded, error), "all finite double boundaries parse");
        for (std::size_t index = 0; index < source.as_array().size(); ++index)
            RIGIDBODIES_EXPECT(decoded.as_array()[index].as_number() == source.as_array()[index].as_number(), "exact boundary is restored");
        RIGIDBODIES_EXPECT(std::signbit(decoded.as_array()[3].as_number()), "negative zero is preserved");
    }

    RIGIDBODIES_TEST("JSON Unicode escapes combine surrogate pairs and preserve UTF8")
    {
        Json value;
        std::string error;
        RIGIDBODIES_EXPECT(parse_json(R"({"name":"\u00e9 \u0130 \ud83d\ude80","nul":"\u0000"})", value, error), "Unicode escapes decode");
        RIGIDBODIES_EXPECT(value.at("name").as_string() == "\xc3\xa9 \xc4\xb0 \xf0\x9f\x9a\x80", "UTF8 sequence encodes exact Unicode scalars");
        RIGIDBODIES_EXPECT(value.at("nul").as_string() == std::string(1, '\0'), "escaped NUL is a valid string character");
        Json reparsed;
        RIGIDBODIES_EXPECT(parse_json(write_json(value), reparsed, error), "raw UTF8 from writer reparses");
        RIGIDBODIES_EXPECT(reparsed.at("name").as_string() == value.at("name").as_string(), "UTF8 is retained without locale conversion");
    }

    RIGIDBODIES_TEST("JSON duplicate keys and malformed syntax fail without changing output")
    {
        const char* invalid[] = {
            "", "[", "{", "[1,]", "{\"x\":1,}", "{\"x\":1,\"x\":2}", "true false", "nullx", "01", "-01", "+1", "1.", ".1", "1e", "1e+", "--1", "NaN", "Infinity", "1e999", "1e-999", "\"unterminated", "\"\\q\"", "\"\\u123\"", "\"\\ud800\"", "\"\\udc00\"", "\"\\ud800\\u0041\"", "\"line\nfeed\""
        };
        for (const auto* text : invalid)
        {
            Json output = "retained";
            std::string error;
            RIGIDBODIES_EXPECT(!parse_json(text, output, error), "invalid JSON is rejected");
            RIGIDBODIES_EXPECT(output.as_string() == "retained" && !error.empty(), "failure preserves output and explains problem");
        }
    }

    RIGIDBODIES_TEST("JSON rejects invalid UTF8 in parsed and manually constructed strings")
    {
        const std::string invalid[] = {
            std::string("\x80", 1), std::string("\xc0\x80", 2), std::string("\xe0\x80\x80", 3), std::string("\xed\xa0\x80", 3), std::string("\xf4\x90\x80\x80", 4), std::string("\xf0\x9f", 2)
        };
        for (const auto& bytes : invalid)
        {
            Json output;
            std::string error, written = "retained";
            RIGIDBODIES_EXPECT(!parse_json("\"" + bytes + "\"", output, error), "invalid UTF8 is not accepted in files");
            RIGIDBODIES_EXPECT(!write_json(Json(bytes), written, error) && written == "retained", "invalid UTF8 cannot be exported either");
        }
    }

    RIGIDBODIES_TEST("JSON parser applies bounded bytes strings depth and value counts")
    {
        Json output;
        std::string error;
        JsonLimits limits;
        limits.max_bytes = 4;
        RIGIDBODIES_EXPECT(!parse_json("[1,2]", output, error, limits), "document byte limit applies before parsing");
        limits = {};
        limits.max_string_bytes = 2;
        RIGIDBODIES_EXPECT(!parse_json("\"abc\"", output, error, limits), "decoded string bytes are bounded");
        limits = {};
        limits.max_depth = 1;
        RIGIDBODIES_EXPECT(!parse_json("[[0]]", output, error, limits), "nested values beyond depth are rejected");
        limits = {};
        limits.max_values = 2;
        RIGIDBODIES_EXPECT(!parse_json("[0,1]", output, error, limits), "value count includes container");
    }

    RIGIDBODIES_TEST("JSON writer applies bounded output and rejects nonfinite values atomically")
    {
        std::string written = "retained", error;
        RIGIDBODIES_EXPECT(!write_json(Json(std::numeric_limits<double>::infinity()), written, error), "infinity cannot be written");
        RIGIDBODIES_EXPECT(!write_json(Json(std::numeric_limits<double>::quiet_NaN()), written, error), "NaN cannot be written");
        JsonLimits limits;
        limits.max_bytes = 2;
        RIGIDBODIES_EXPECT(!write_json(Json("a"), written, error, limits), "output bytes are bounded including delimiters");
        limits = {};
        limits.max_depth = 0;
        RIGIDBODIES_EXPECT(!write_json(Json(Json::Array { 0 }), written, error, limits), "output depth is bounded");
        RIGIDBODIES_EXPECT(written == "retained", "all failed writes preserve previous output");
    }

    RIGIDBODIES_TEST("JSON typed access never silently coerces mismatched fields")
    {
        Json value = Json::Object { { "boolean", true } };
        bool missing = false, wrong_type = false;
        try
        {
            static_cast<void>(value.at("missing"));
        }
        catch (const std::runtime_error&)
        {
            missing = true;
        }
        try
        {
            static_cast<void>(value.at("boolean").as_number());
        }
        catch (const std::runtime_error&)
        {
            wrong_type = true;
        }
        RIGIDBODIES_EXPECT(missing && wrong_type, "schema code can reliably distinguish absent or mistyped fields");
        RIGIDBODIES_EXPECT(value.find("missing") == nullptr, "optional fields return a null lookup");
    }

    RIGIDBODIES_TEST("Document versions accept additive minor changes and reject changed majors")
    {
        Json root = Json::Object { { "format", "rigid-bodies.shape" }, { "version", document_version() } };
        std::string error;
        RIGIDBODIES_EXPECT(validate_document_header(root, "rigid-bodies.shape", error), "current format is accepted");
        root["version"]["minor"] = 8;
        RIGIDBODIES_EXPECT(validate_document_header(root, "rigid-bodies.shape", error), "later additive minor is accepted");
        root["version"]["major"] = 2;
        RIGIDBODIES_EXPECT(!validate_document_header(root, "rigid-bodies.shape", error), "incompatible major is rejected");
        root["version"]["major"] = 1;
        root["version"]["minor"] = 0.5;
        RIGIDBODIES_EXPECT(!validate_document_header(root, "rigid-bodies.shape", error), "fractional versions are rejected");
        root["version"]["minor"] = 0;
        RIGIDBODIES_EXPECT(!validate_document_header(root, "rigid-bodies.scenario", error), "wrong document kind is rejected");
        root["required_features"] = Json::Array { "future_behavior" };
        RIGIDBODIES_EXPECT(!validate_document_header(root, "rigid-bodies.shape", error), "required unknown behavior is not silently ignored");
    }

    RIGIDBODIES_TEST("the supported-feature overload accepts only listed string features")
    {
        const auto unsupported = std::string("Document requires unsupported features.");
        Json root = Json::Object { { "format", "rigid-bodies.scenario" }, { "version", document_version() } };
        std::string error;
        RIGIDBODIES_EXPECT(validate_document_header(root, "rigid-bodies.scenario", error, { "special_relativity" }), "a document may require nothing");
        root["required_features"] = Json::Array {};
        RIGIDBODIES_EXPECT(validate_document_header(root, "rigid-bodies.scenario", error, { "special_relativity" }), "an empty list requires nothing");
        RIGIDBODIES_EXPECT(validate_document_header(root, "rigid-bodies.scenario", error), "an empty list passes the strict form too");
        root["required_features"] = Json::Array { "special_relativity" };
        RIGIDBODIES_EXPECT(validate_document_header(root, "rigid-bodies.scenario", error, { "special_relativity" }) && error.empty(), "a listed feature is accepted");
        RIGIDBODIES_EXPECT(validate_document_header(root, "rigid-bodies.scenario", error, { "other_feature", "special_relativity" }), "a feature is found anywhere in the supported list");
        RIGIDBODIES_EXPECT(!validate_document_header(root, "rigid-bodies.scenario", error) && error == unsupported, "the three-argument form supports no feature");
        RIGIDBODIES_EXPECT(!validate_document_header(root, "rigid-bodies.scenario", error, {}) && error == unsupported, "an empty supported list supports no feature");
        const Json invalid[] = {
            Json::Array { "special_relativity", "special_relativity" },
            Json::Array { "special_relativity", "future_behavior" },
            Json::Array { "future_behavior", "special_relativity" },
            Json::Array { "Special_Relativity" },
            Json::Array { "special_relativity " },
            Json::Array { 1 },
            Json::Array { nullptr },
            Json::Array { Json(Json::Array { "special_relativity" }) },
            Json::Array { Json::Object { { "name", "special_relativity" } } },
            Json("special_relativity"),
            Json(Json::Object { { "special_relativity", true } }),
            Json(true),
            Json(nullptr),
        };
        for (const auto& features : invalid)
        {
            root["required_features"] = features;
            error.clear();
            RIGIDBODIES_EXPECT(!validate_document_header(root, "rigid-bodies.scenario", error, { "special_relativity" }), "duplicates, unknown names and non-strings are rejected");
            RIGIDBODIES_EXPECT(error == unsupported, "every rejection keeps the existing explanation");
        }
    }

    RIGIDBODIES_TEST("shape and benchmark headers still reject special_relativity")
    {
        const auto unsupported = std::string("Document requires unsupported features.");
        std::string error;
        for (const auto* format : { "rigid-bodies.shape", "rigid-bodies-benchmark" })
        {
            Json root = Json::Object { { "format", format }, { "version", document_version() }, { "required_features", Json::Array { "special_relativity" } } };
            RIGIDBODIES_EXPECT(!validate_document_header(root, format, error) && error == unsupported, "formats without the feature reject it");
        }
        const auto square = std::string(R"({"format":"rigid-bodies.shape","version":{"major":1,"minor":0},"metadata":{"title":"Square","summary":"Fixture"},"shape":{"outline":{"closed":true,"nodes":[{"position_m":[0,0]},{"position_m":[1,0]},{"position_m":[1,1]},{"position_m":[0,1]}]},"options":{"max_render_vertices":64,"max_collision_vertices":32,"max_subdivision_depth":8}}})");
        rigidbodies::physics::ShapeDocument shape;
        RIGIDBODIES_EXPECT(rigidbodies::physics::parse_shape_document(square, shape, error), error);
        Json changed;
        RIGIDBODIES_EXPECT(parse_json(square, changed, error), error);
        changed["required_features"] = Json::Array { "special_relativity" };
        RIGIDBODIES_EXPECT(!rigidbodies::physics::parse_shape_document(write_json(changed), shape, error) && error == unsupported, "a shape file that requires special relativity is rejected");
        const Json benchmark = Json::Object { { "format", "rigid-bodies-benchmark" }, { "version", document_version() }, { "required_features", Json::Array { "special_relativity" } }, { "suite_revision", 1 } };
        rigidbodies::physics::BenchmarkReport report;
        RIGIDBODIES_EXPECT(!rigidbodies::physics::parse_benchmark_report(write_json(benchmark), report, error) && error == unsupported, "a benchmark report that requires special relativity is rejected by its header");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
