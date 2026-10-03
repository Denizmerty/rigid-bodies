#pragma once

#include <rigidbodies/physics/content_json.hpp>
#include <rigidbodies/physics/shape_authoring.hpp>

namespace rigidbodies::physics
{
    struct ShapeDocument
    {
        content::Json root;
        std::shared_ptr<const AuthoredShape> shape;
        std::string title;
        std::string summary;
    };

    // Nested payloads omit the document envelope, so authored parts in scenarios use precisely
    // the same source geometry and validation as independently exchanged shapes.
    [[nodiscard]] content::Json encode_authored_shape(const AuthoredShape& shape);
    [[nodiscard]] bool decode_authored_shape(const content::Json& value, std::shared_ptr<const AuthoredShape>& shape, std::string& error);

    [[nodiscard]] bool parse_shape_document(std::string_view text, ShapeDocument& document, std::string& error);
    [[nodiscard]] bool capture_shape_document(const AuthoredShape& shape, std::string title, std::string summary,
        ShapeDocument& document, std::string& error, const ShapeDocument* source = nullptr);
    [[nodiscard]] bool write_shape_document(const ShapeDocument& document, std::string& text, std::string& error);
    [[nodiscard]] std::string write_shape_document(const ShapeDocument& document);
}
