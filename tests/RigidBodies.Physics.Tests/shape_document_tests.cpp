#include <rigidbodies/physics/shape_document.hpp>

#include "test_framework.hpp"

namespace
{
    using namespace rigidbodies::physics;
    using content::Json;

    std::shared_ptr<const AuthoredShape> curved_shape()
    {
        Outline source;
        source.closed = true;
        for (const auto point : { rigidbodies::math::Vec2 { 0.0, 0.0 }, { 2.0, 0.0 }, { 2.0, 1.0 }, { 0.0, 1.0 } })
        {
            OutlineNode node;
            node.position_m = point;
            source.nodes.push_back(node);
        }
        source.nodes[2].outgoing_edge = OutlineEdgeKind::cubic;
        source.nodes[2].outgoing_handle_m = { 0.0, 1.0 };
        source.nodes[3].incoming_handle_m = { 0.0, 1.0 };
        source.nodes[2].continuity = OutlineContinuity::aligned;
        ShapeAuthoringOptions options;
        options.render_tolerance_m = 0.003;
        options.collision_tolerance_m = 0.013;
        options.simplification_tolerance_m = 0.001;
        options.max_collision_vertices = 200;
        const auto built = build_authored_shape(source, options);
        RIGIDBODIES_EXPECT(built.succeeded(), "curved fixture builds");
        return built.shape;
    }

    ShapeDocument example_document()
    {
        ShapeDocument result;
        std::string error;
        RIGIDBODIES_EXPECT(capture_shape_document(*curved_shape(), "Curved beam", "A cubic source outline", result, error), "shape document captures");
        return result;
    }

    RIGIDBODIES_TEST("Shape documents retain editable cubic control points and authoring options")
    {
        const auto document = example_document();
        ShapeDocument loaded;
        std::string error;
        RIGIDBODIES_EXPECT(parse_shape_document(write_shape_document(document), loaded, error), "export is importable");
        RIGIDBODIES_EXPECT(loaded.title == document.title && loaded.summary == document.summary, "descriptive metadata survives");
        const auto& node = loaded.shape->source.nodes[2];
        RIGIDBODIES_EXPECT(node.outgoing_edge == OutlineEdgeKind::cubic && node.continuity == OutlineContinuity::aligned, "source edit semantics survive");
        RIGIDBODIES_EXPECT_NEAR(node.outgoing_handle_m.y, 1.0, 0.0, "exact control point is retained");
        RIGIDBODIES_EXPECT_NEAR(loaded.shape->options.render_tolerance_m, 0.003, 0.0, "render quality survives");
        RIGIDBODIES_EXPECT(loaded.shape->options.max_collision_vertices == 200, "collision budget survives");
        RIGIDBODIES_EXPECT(write_shape_document(loaded) == write_shape_document(document), "canonical document is stable");
    }

    RIGIDBODIES_TEST("Shape import regenerates derived meshes from authoritative source")
    {
        auto document = example_document();
        document.root["shape"]["render_outline"] = Json::Array { "untrusted derived data" };
        document.root["shape"]["convex_parts"] = Json::Array {};
        ShapeDocument loaded;
        std::string error;
        RIGIDBODIES_EXPECT(parse_shape_document(write_shape_document(document), loaded, error), "additive unknown fields are accepted");
        RIGIDBODIES_EXPECT(!loaded.shape->convex_parts.empty() && !loaded.shape->render_triangles.empty(), "geometry is regenerated rather than taking serialized caches");
        RIGIDBODIES_EXPECT(loaded.shape->render_outline.size() == document.shape->render_outline.size(), "deterministic rebuild restores same render detail");
    }

    RIGIDBODIES_TEST("Shape imports with future minor extensions retain unknown nested data on edits")
    {
        auto source = example_document();
        source.root["version"]["minor"] = 7;
        source.root["future_root"] = "retained";
        source.root["metadata"]["attribution"] = "A creator";
        source.root["shape"]["options"]["future_tolerance"] = 0.25;
        source.root["shape"]["outline"]["nodes"].as_array()[2]["future_node"] = true;
        ShapeDocument parsed, updated;
        std::string error;
        RIGIDBODIES_EXPECT(parse_shape_document(write_shape_document(source), parsed, error), "later minor schema imports");
        RIGIDBODIES_EXPECT(capture_shape_document(*parsed.shape, "New title", parsed.summary, updated, error, &parsed), "older reader can resave safely");
        RIGIDBODIES_EXPECT(updated.root.at("version").at("minor").as_number() == 7.0, "original version remains truthful");
        RIGIDBODIES_EXPECT(updated.root.at("future_root").as_string() == "retained", "root extension survives");
        RIGIDBODIES_EXPECT(updated.root.at("metadata").at("attribution").as_string() == "A creator", "metadata extension survives");
        RIGIDBODIES_EXPECT(updated.root.at("shape").at("options").at("future_tolerance").as_number() == 0.25, "options extension survives");
        RIGIDBODIES_EXPECT(updated.root.at("shape").at("outline").at("nodes").as_array()[2].at("future_node").as_bool(), "control node extension survives");
        RIGIDBODIES_EXPECT(updated.title == "New title", "known edited fields take precedence");
    }

    RIGIDBODIES_TEST("Shape imports reject incompatible versions and preserve existing document")
    {
        const auto initial = example_document();
        auto invalid = initial;
        invalid.root["version"]["major"] = 2;
        auto destination = initial;
        std::string error;
        RIGIDBODIES_EXPECT(!parse_shape_document(write_shape_document(invalid), destination, error), "future major cannot be interpreted safely");
        RIGIDBODIES_EXPECT(destination.shape == initial.shape && destination.title == initial.title, "rejected version leaves working document untouched");
    }

    RIGIDBODIES_TEST("Shape imports reject unknown geometry kinds with actionable diagnostics")
    {
        auto invalid = example_document();
        invalid.root["shape"]["outline"]["nodes"].as_array()[0]["outgoing_edge"] = "quadratic";
        ShapeDocument destination;
        std::string error;
        RIGIDBODIES_EXPECT(!parse_shape_document(write_shape_document(invalid), destination, error), "unknown behavior is rejected despite forward field compatibility");
        RIGIDBODIES_EXPECT(error.find("quadratic") != std::string::npos, "diagnostic identifies unsupported kind");
    }

    RIGIDBODIES_TEST("Shape imports reject open and self intersecting source without replacing geometry")
    {
        const auto source = example_document();
        auto invalid = source;
        invalid.root["shape"]["outline"]["closed"] = false;
        auto retained = source.shape;
        std::string error;
        RIGIDBODIES_EXPECT(!decode_authored_shape(invalid.root.at("shape"), retained, error), "open source is rejected");
        RIGIDBODIES_EXPECT(retained == source.shape, "nested decoder is transactional");
        invalid = source;
        auto& nodes = invalid.root["shape"]["outline"]["nodes"].as_array();
        nodes[0]["position_m"] = Json::Array { 0, 0 };
        nodes[1]["position_m"] = Json::Array { 2, 1 };
        nodes[2]["position_m"] = Json::Array { 0, 1 };
        nodes[2]["outgoing_edge"] = "line";
        nodes[3]["position_m"] = Json::Array { 2, 0 };
        RIGIDBODIES_EXPECT(!decode_authored_shape(invalid.root.at("shape"), retained, error), "crossing source is rejected");
        RIGIDBODIES_EXPECT(retained == source.shape, "failed rebuild keeps earlier shape");
    }

    RIGIDBODIES_TEST("Shape imports bound tessellation budgets before generating meshes")
    {
        const auto source = example_document();
        for (const auto field : { "max_render_vertices", "max_collision_vertices", "max_subdivision_depth" })
        {
            auto invalid = source;
            invalid.root["shape"]["options"][field] = 1000000000;
            ShapeDocument output;
            std::string error;
            RIGIDBODIES_EXPECT(!parse_shape_document(write_shape_document(invalid), output, error), "unbounded work request is rejected before geometry build");
        }
    }

    RIGIDBODIES_TEST("Shape imports validate coordinate arity option types and integer budgets")
    {
        const auto source = example_document();
        auto invalid = source;
        invalid.root["shape"]["outline"]["nodes"].as_array()[0]["position_m"] = Json::Array { 0, 0, 0 };
        ShapeDocument output;
        std::string error;
        RIGIDBODIES_EXPECT(!parse_shape_document(write_shape_document(invalid), output, error), "three dimensional coordinates are not silently truncated");
        invalid = source;
        invalid.root["shape"]["options"]["collision_tolerance_m"] = "0.01";
        RIGIDBODIES_EXPECT(!parse_shape_document(write_shape_document(invalid), output, error), "numeric string is not silently coerced");
        invalid = source;
        invalid.root["shape"]["options"]["max_render_vertices"] = 32.5;
        RIGIDBODIES_EXPECT(!parse_shape_document(write_shape_document(invalid), output, error), "fractional vertex counts are not silently truncated");
    }

    RIGIDBODIES_TEST("Shape documents permit omitted default options and plain line handles")
    {
        auto source = example_document();
        source.root["shape"].as_object().erase("options");
        auto& nodes = source.root["shape"]["outline"]["nodes"].as_array();
        for (auto& node : nodes)
        {
            node.as_object().erase("incoming_handle_m");
            node.as_object().erase("outgoing_handle_m");
            node.as_object().erase("outgoing_edge");
            node.as_object().erase("continuity");
        }
        source.root.as_object().erase("metadata");
        ShapeDocument output;
        std::string error;
        RIGIDBODIES_EXPECT(parse_shape_document(write_shape_document(source), output, error), "minimal hand authored polygon imports");
        RIGIDBODIES_EXPECT(output.shape->source.nodes.size() == 4 && output.shape->convex_parts.size() == 1, "default lines rebuild expected box");
        RIGIDBODIES_EXPECT(output.title == "Imported shape", "missing optional title gets a useful display name");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
