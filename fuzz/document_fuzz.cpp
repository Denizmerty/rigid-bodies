#include "document_fuzz.hpp"

#include <rigidbodies/physics/scenario_document.hpp>
#include <rigidbodies/physics/shape_document.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace rigidbodies::fuzz
{
    namespace
    {
        using namespace physics;
        using content::Json;

        void require(bool condition, const char* message)
        {
            if (!condition)
                throw std::logic_error(message);
        }

        bool bounded_semantics(const Json& value, std::size_t& values, std::size_t& outlines)
        {
            if (++values > 1024)
                return false;
            if (value.is_array())
            {
                if (value.as_array().size() > 24)
                    return false;
                for (const auto& item : value.as_array())
                    if (!bounded_semantics(item, values, outlines))
                        return false;
            }
            if (value.is_object())
            {
                for (const auto& item : value.as_object())
                {
                    if (item.first == "outline" && ++outlines > 4)
                        return false;
                    // Keep mutated tessellation budgets within the production defaults.
                    // Wrong types and negative counts still reach real schema validation.
                    if (item.second.is_number())
                    {
                        const auto number = item.second.as_number();
                        if ((item.first == "max_render_vertices" && number > 512) ||
                            (item.first == "max_collision_vertices" && number > 128) ||
                            (item.first == "max_subdivision_depth" && number > 16))
                            return false;
                    }
                    if (!bounded_semantics(item.second, values, outlines))
                        return false;
                }
            }
            return true;
        }

        void check_shape(const AuthoredShape& shape)
        {
            require(std::isfinite(shape.approximation_added_area_m2) && std::isfinite(shape.max_collision_error_m), "Accepted shape has non-finite error bounds");
            require(!shape.convex_parts.empty() && !shape.render_triangles.empty(), "Accepted shape has no geometry");
            for (const auto& point : shape.render_outline)
                require(math::is_finite(point), "Accepted render outline is non-finite");
            for (const auto& point : shape.collision_outline)
                require(math::is_finite(point), "Accepted collision outline is non-finite");
            for (const auto& triangle : shape.render_triangles)
                for (const auto& point : triangle)
                    require(math::is_finite(point), "Accepted render triangle is non-finite");
            for (const auto& part : shape.convex_parts)
            {
                require(part.size() >= 3, "Accepted collision cell is degenerate");
                for (const auto& point : part)
                    require(math::is_finite(point), "Accepted collision cell is non-finite");
            }
        }

        void check_world(const World& world)
        {
            for (const auto id : world.body_ids())
            {
                const auto* body = world.find_body(id);
                require(body != nullptr, "Accepted world has an invalid body identifier");
                require(math::is_finite(body->position_m()) && math::is_finite(body->linear_velocity_m_s()) &&
                        math::is_finite(body->world_center_of_mass_m()) && std::isfinite(body->orientation_rad()) &&
                        std::isfinite(body->angular_velocity_rad_s()) && body->mass_properties().is_valid(),
                    "Accepted world has non-finite body state or invalid mass");
                for (const auto& collider : body->colliders())
                {
                    require(collider.shape != nullptr, "Accepted collider has no shape");
                    const auto bounds = collider.compute_bounds(body->transform());
                    require(math::is_finite(bounds.minimum) && math::is_finite(bounds.maximum), "Accepted collider has non-finite bounds");
                }
            }
        }

        World sentinel_world()
        {
            WorldSettings settings;
            settings.gravity_m_s2 = { 1, -3 };
            World world(settings);
            BodyDefinition definition;
            definition.name = "transaction-sentinel";
            definition.position_m = { 2, 7 };
            definition.linear_velocity_m_s = { -3, 5 };
            Collider collider;
            collider.shape = make_circle(0.5);
            definition.colliders.push_back(collider);
            (void)world.create_body(definition, "sentinel");
            return world;
        }

        std::string world_arrangement(const World& world)
        {
            ScenarioDocument document;
            std::string error, text;
            const ScenarioMetadata metadata { "fuzz-sentinel", "Fuzz sentinel", "Transaction fixture", {}, {}, {}, {}, {}, {}, 0, 0, false };
            require(capture_scenario_document(world, metadata, document, error), "Could not capture fuzz transaction fixture");
            require(write_scenario_document(document, text, error), "Could not serialize fuzz transaction fixture");
            return text;
        }
    }

    DocumentCoverage exercise_document(std::string_view input)
    {
        DocumentCoverage coverage;
        if (input.size() > maximum_input_bytes)
            return coverage;
        Json json = "retained-json";
        std::string error;
        coverage.json_accepted = content::parse_json(input, json, error);
        if (coverage.json_accepted)
        {
            require(error.empty(), "Successful JSON parsing left an error");
            std::string encoded, repeated;
            require(content::write_json(json, encoded, error), "Accepted JSON could not be written");
            Json reparsed;
            require(content::parse_json(encoded, reparsed, error), "Written JSON could not be parsed");
            require(content::write_json(reparsed, repeated, error) && encoded == repeated, "JSON roundtrip changed canonical data");
            std::size_t values = 0, outlines = 0;
            if (!bounded_semantics(json, values, outlines))
            {
                coverage.semantic_budget_exceeded = true;
                return coverage;
            }
        }
        else
        {
            require(json.is_string() && json.as_string() == "retained-json" && !error.empty(), "Rejected JSON changed output or omitted its error");
        }

        ShapeDocument shape;
        shape.root = "retained-shape";
        shape.title = "retained-title";
        coverage.shape_accepted = parse_shape_document(input, shape, error);
        if (coverage.shape_accepted)
        {
            require(coverage.json_accepted && shape.shape != nullptr && error.empty(), "Accepted shape has no validated JSON or geometry");
            check_shape(*shape.shape);
            std::string encoded, repeated;
            require(write_shape_document(shape, encoded, error), "Accepted shape could not be written");
            ShapeDocument reparsed;
            require(parse_shape_document(encoded, reparsed, error), "Written shape could not be parsed");
            check_shape(*reparsed.shape);
            require(write_shape_document(reparsed, repeated, error) && encoded == repeated, "Shape roundtrip changed canonical data");
        }
        else
        {
            require(shape.root.is_string() && shape.root.as_string() == "retained-shape" && shape.title == "retained-title" && !shape.shape && !error.empty(), "Rejected shape changed output or omitted its error");
        }

        ScenarioDocument scenario;
        scenario.root = "retained-scenario";
        scenario.metadata.id = "retained-id";
        coverage.scenario_accepted = parse_scenario_document(input, scenario, error);
        if (coverage.scenario_accepted)
        {
            require(coverage.json_accepted && error.empty(), "Accepted scenario has no validated JSON");
            World world;
            require(populate_world(scenario, world, error), "Accepted scenario could not populate a world");
            check_world(world);
            std::string encoded, repeated;
            require(write_scenario_document(scenario, encoded, error), "Accepted scenario could not be written");
            ScenarioDocument reparsed;
            require(parse_scenario_document(encoded, reparsed, error), "Written scenario could not be parsed");
            require(write_scenario_document(reparsed, repeated, error) && encoded == repeated, "Scenario roundtrip changed canonical data");
            World restored;
            require(populate_world(reparsed, restored, error), "Roundtripped scenario could not populate a world");
            check_world(restored);
            require(world_arrangement(world) == world_arrangement(restored), "Scenario roundtrip changed the populated arrangement");
        }
        else
        {
            require(scenario.root.is_string() && scenario.root.as_string() == "retained-scenario" && scenario.metadata.id == "retained-id" && !error.empty(), "Rejected scenario changed output or omitted its error");
            if (coverage.json_accepted)
            {
                auto world = sentinel_world();
                const auto ids = world.body_ids();
                const auto original = world_arrangement(world);
                ScenarioDocument invalid { json, {} };
                require(!populate_world(invalid, world, error) && !error.empty(), "Scenario parser and world loader disagree about rejection");
                require(world.body_ids() == ids && world_arrangement(world) == original, "Rejected world load changed the destination");
                coverage.world_rejection_checked = true;
            }
        }
        return coverage;
    }
}
