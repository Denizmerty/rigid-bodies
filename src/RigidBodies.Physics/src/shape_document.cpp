#include <rigidbodies/physics/shape_document.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace rigidbodies::physics
{
    namespace
    {
        using content::Json;

        Json vector_json(const math::Vec2& value)
        {
            return Json::Array { value.x, value.y };
        }
        math::Vec2 vector_value(const Json& value)
        {
            const auto& array = value.as_array();
            if (array.size() != 2)
                throw std::runtime_error("Shape coordinates require two numbers.");
            const math::Vec2 result { array[0].as_number(), array[1].as_number() };
            if (!std::isfinite(result.x) || !std::isfinite(result.y) || std::abs(result.x) > 1.0e9 || std::abs(result.y) > 1.0e9)
                throw std::runtime_error("Shape coordinates must be finite and within one billion metres.");
            return result;
        }
        double option_number(const Json& object, std::string_view key, double fallback)
        {
            const auto found = object.find(key);
            if (!found)
                return fallback;
            const auto value = found->as_number();
            if (!std::isfinite(value))
                throw std::runtime_error("Shape option must be finite: " + std::string(key) + ".");
            return value;
        }
        std::size_t option_count(const Json& object, std::string_view key, std::size_t fallback, std::size_t maximum)
        {
            const auto number = option_number(object, key, static_cast<double>(fallback));
            if (number < 0.0 || number > static_cast<double>(maximum) || std::floor(number) != number)
                throw std::runtime_error("Shape option exceeds supported integer budget: " + std::string(key) + ".");
            return static_cast<std::size_t>(number);
        }
        std::string optional_string(const Json& object, std::string_view key, std::string fallback)
        {
            const auto found = object.find(key);
            if (found)
                return found->as_string();
            return fallback;
        }
        // Retain extensions on still-present objects and nodes when a document is edited by an
        // older reader. Removed nodes remain removed; array elements correspond by source index.
        Json with_extensions(const Json& source, Json current)
        {
            if (source.is_object() && current.is_object())
            {
                auto combined = source;
                for (auto& entry : current.as_object())
                {
                    const auto previous = source.find(entry.first);
                    combined[entry.first] = previous ? with_extensions(*previous, std::move(entry.second)) : std::move(entry.second);
                }
                return combined;
            }
            if (source.is_array() && current.is_array())
            {
                const auto& prior = source.as_array();
                auto& fresh = current.as_array();
                for (std::size_t index = 0; index < fresh.size() && index < prior.size(); ++index)
                    fresh[index] = with_extensions(prior[index], std::move(fresh[index]));
            }
            return current;
        }
    }

    content::Json encode_authored_shape(const AuthoredShape& shape)
    {
        Json::Array nodes;
        for (const auto& node : shape.source.nodes)
        {
            if (node.outgoing_edge != OutlineEdgeKind::line && node.outgoing_edge != OutlineEdgeKind::cubic)
                throw std::runtime_error("Unsupported authored edge kind.");
            if (node.continuity != OutlineContinuity::corner && node.continuity != OutlineContinuity::aligned && node.continuity != OutlineContinuity::mirrored)
                throw std::runtime_error("Unsupported authored continuity.");
            nodes.emplace_back(Json::Object {
                { "position_m", vector_json(node.position_m) },
                { "incoming_handle_m", vector_json(node.incoming_handle_m) },
                { "outgoing_handle_m", vector_json(node.outgoing_handle_m) },
                { "outgoing_edge", node.outgoing_edge == OutlineEdgeKind::line ? "line" : "cubic" },
                { "continuity", node.continuity == OutlineContinuity::corner ? "corner" : node.continuity == OutlineContinuity::aligned ? "aligned"
                                                                                                                                        : "mirrored" } });
        }
        const auto& options = shape.options;
        return Json::Object {
            { "outline", Json::Object { { "closed", shape.source.closed }, { "nodes", std::move(nodes) } } },
            { "options", Json::Object { { "render_tolerance_m", options.render_tolerance_m }, { "collision_tolerance_m", options.collision_tolerance_m }, { "simplification_tolerance_m", options.simplification_tolerance_m }, { "concavity_tolerance_m", options.concavity_tolerance_m }, { "max_render_vertices", options.max_render_vertices }, { "max_collision_vertices", options.max_collision_vertices }, { "max_subdivision_depth", options.max_subdivision_depth }, { "minimum_area_m2", options.minimum_area_m2 }, { "duplicate_tolerance_m", options.duplicate_tolerance_m } } }
        };
    }

    bool decode_authored_shape(const content::Json& value, std::shared_ptr<const AuthoredShape>& shape, std::string& error)
    {
        try
        {
            const auto& source = value.at("outline");
            Outline outline;
            outline.closed = source.at("closed").as_bool();
            const auto& nodes = source.at("nodes").as_array();
            if (nodes.size() > 4096)
                throw std::runtime_error("Authored outline exceeds 4096 source nodes.");
            for (const auto& item : nodes)
            {
                OutlineNode node;
                node.position_m = vector_value(item.at("position_m"));
                if (const auto incoming = item.find("incoming_handle_m"))
                    node.incoming_handle_m = vector_value(*incoming);
                if (const auto outgoing = item.find("outgoing_handle_m"))
                    node.outgoing_handle_m = vector_value(*outgoing);
                const auto edge = optional_string(item, "outgoing_edge", "line");
                if (edge == "line")
                    node.outgoing_edge = OutlineEdgeKind::line;
                else if (edge == "cubic")
                    node.outgoing_edge = OutlineEdgeKind::cubic;
                else
                    throw std::runtime_error("Unsupported outline edge kind: " + edge + ".");
                const auto continuity = optional_string(item, "continuity", "corner");
                if (continuity == "corner")
                    node.continuity = OutlineContinuity::corner;
                else if (continuity == "aligned")
                    node.continuity = OutlineContinuity::aligned;
                else if (continuity == "mirrored")
                    node.continuity = OutlineContinuity::mirrored;
                else
                    throw std::runtime_error("Unsupported outline continuity: " + continuity + ".");
                outline.nodes.push_back(node);
            }
            ShapeAuthoringOptions options;
            if (const auto settings = value.find("options"))
            {
                if (!settings->is_object())
                    throw std::runtime_error("Shape options must be an object.");
                options.render_tolerance_m = option_number(*settings, "render_tolerance_m", options.render_tolerance_m);
                options.collision_tolerance_m = option_number(*settings, "collision_tolerance_m", options.collision_tolerance_m);
                options.simplification_tolerance_m = option_number(*settings, "simplification_tolerance_m", options.simplification_tolerance_m);
                options.concavity_tolerance_m = option_number(*settings, "concavity_tolerance_m", options.concavity_tolerance_m);
                options.max_render_vertices = option_count(*settings, "max_render_vertices", options.max_render_vertices, 4096);
                options.max_collision_vertices = option_count(*settings, "max_collision_vertices", options.max_collision_vertices, 512);
                options.max_subdivision_depth = static_cast<unsigned>(option_count(*settings, "max_subdivision_depth", options.max_subdivision_depth, 24));
                options.minimum_area_m2 = option_number(*settings, "minimum_area_m2", options.minimum_area_m2);
                options.duplicate_tolerance_m = option_number(*settings, "duplicate_tolerance_m", options.duplicate_tolerance_m);
            }
            const auto built = build_authored_shape(outline, options);
            if (!built.succeeded())
            {
                throw std::runtime_error(built.diagnostics.empty() ? "Authored shape is invalid." : built.diagnostics.front().message);
            }
            shape = built.shape;
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }

    bool parse_shape_document(std::string_view text, ShapeDocument& document, std::string& error)
    {
        try
        {
            ShapeDocument parsed;
            if (!content::parse_json(text, parsed.root, error) || !content::validate_document_header(parsed.root, "rigid-bodies.shape", error))
                return false;
            if (const auto metadata = parsed.root.find("metadata"))
            {
                if (!metadata->is_object())
                    throw std::runtime_error("Shape metadata must be an object.");
                parsed.title = optional_string(*metadata, "title", "Imported shape");
                parsed.summary = optional_string(*metadata, "summary", "");
            }
            else
                parsed.title = "Imported shape";
            if (parsed.title.size() > 256 || parsed.summary.size() > 4096)
                throw std::runtime_error("Shape metadata exceeds supported text length.");
            if (!decode_authored_shape(parsed.root.at("shape"), parsed.shape, error))
                return false;
            document = std::move(parsed);
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }

    bool capture_shape_document(const AuthoredShape& shape, std::string title, std::string summary, ShapeDocument& document, std::string& error, const ShapeDocument* source)
    {
        try
        {
            Json current = Json::Object {
                { "format", "rigid-bodies.shape" },
                { "version", content::document_version() },
                { "metadata", Json::Object { { "title", std::move(title) }, { "summary", std::move(summary) } } },
                { "shape", encode_authored_shape(shape) }
            };
            if (source)
            {
                if (!content::validate_document_header(source->root, "rigid-bodies.shape", error))
                    return false;
                current = with_extensions(source->root, std::move(current));
                current["version"] = source->root.at("version");
            }
            // One canonical validation path also checks edited source geometry, numeric limits,
            // UTF-8, and extension fields before publishing the captured document.
            return parse_shape_document(content::write_json(current), document, error);
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }

    bool write_shape_document(const ShapeDocument& document, std::string& text, std::string& error)
    {
        return content::write_json(document.root, text, error);
    }

    std::string write_shape_document(const ShapeDocument& document)
    {
        return content::write_json(document.root);
    }
}
