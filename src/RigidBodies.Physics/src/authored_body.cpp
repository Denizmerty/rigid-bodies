#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace rigidbodies::physics
{
    // Body edits stage copies of mutable body/world bookkeeping while retaining the identities
    // of pluggable components. The draft never advances or edits those shared components. This
    // makes allocation/validation failures atomic even with non-cloneable user generators.
    struct AuthoredBodyAccess
    {
        static std::unique_ptr<World> stage(const World& world)
        {
            return world.clone(false);
        }
        static void replace(World& world, BodyId id, const BodyDefinition& definition)
        {
            RigidBody replacement(definition);
            for (const auto& manifold : world.manifolds_)
                if (!manifold.is_sensor && (manifold.first == id || manifold.second == id))
                    if (auto* neighbor = world.find_body(manifold.first == id ? manifold.second : manifold.first))
                        neighbor->wake();
            world.remove_collision_state(id);
            world.bodies_[world.slots_[id.index].dense_index] = std::move(replacement);
            world.slots_[id.index].applied_spring_load = {};
        }
        static void publish(World& destination, std::unique_ptr<World> draft)
        {
            draft->refresh_statistics();
            destination = std::move(*draft);
        }
    };

    namespace
    {
        void require(bool valid, const char* message)
        {
            if (!valid)
                throw std::invalid_argument(message);
        }

        bool nonnegative(Real value)
        {
            return math::is_finite(value) && value >= 0.0;
        }

        void validate_part(const AuthoredPartDefinition& part)
        {
            require(part.shape != nullptr, "An authored part requires a validated shape");
            const auto& rotation = part.local_transform.rotation;
            require(math::is_finite(part.local_transform.translation) && math::is_finite(rotation.sine) && math::is_finite(rotation.cosine) &&
                    std::abs(rotation.sine * rotation.sine + rotation.cosine * rotation.cosine - 1.0) <= 1.0e-10,
                "Authored part placement must be a finite rigid transform");
            require(nonnegative(part.depth_m), "Authored part depth must be finite and nonnegative");
            require(!part.density_override_kg_m3 || nonnegative(*part.density_override_kg_m3), "Authored density must be finite and nonnegative");
            require(!part.drag_coefficient_override || nonnegative(*part.drag_coefficient_override), "Authored drag coefficient must be finite and nonnegative");
            const auto& material = part.material;
            require(nonnegative(material.density_kg_m3) && nonnegative(material.restitution) && material.restitution <= 1.0 &&
                    nonnegative(material.static_friction) && nonnegative(material.kinetic_friction) && nonnegative(material.drag_coefficient) &&
                    nonnegative(material.rolling_friction_m) && nonnegative(material.spinning_friction_m) &&
                    math::is_finite(material.friction_axis_local) && nonnegative(material.friction_anisotropy_ratio),
                "Authored material properties must be finite and physically valid");
            require(!part.shape->convex_parts.empty(), "An authored shape must contain collision cells");
        }

        std::shared_ptr<const AuthoredShape> freeze_shape(const std::shared_ptr<const AuthoredShape>& source)
        {
            require(source != nullptr, "An authored part requires a validated shape");
            const auto result = build_authored_shape(source->source, source->options);
            if (!result.succeeded())
                throw std::invalid_argument(result.diagnostics.empty() ? "The authored outline is invalid" : result.diagnostics.front().message);
            return result.shape;
        }

        std::vector<Collider> cells(AuthoredPartDefinition definition, bool detach_shape)
        {
            if (detach_shape)
                definition.shape = freeze_shape(definition.shape);
            validate_part(definition);
            auto part = std::make_shared<const AuthoredPartDefinition>(std::move(definition));
            std::vector<Collider> result;
            result.reserve(part->shape->convex_parts.size());
            for (const auto& vertices : part->shape->convex_parts)
            {
                Collider collider;
                collider.shape = std::make_shared<ConvexPolygonShape>(vertices);
                collider.local_transform = part->local_transform;
                collider.material = part->material;
                collider.depth_m = part->depth_m;
                collider.filter = part->filter;
                collider.density_override_kg_m3 = part->density_override_kg_m3;
                collider.drag_coefficient_override = part->drag_coefficient_override;
                collider.is_sensor = part->is_sensor;
                collider.authored_part = part;
                result.push_back(std::move(collider));
            }
            return result;
        }

        void append(std::vector<Collider>& destination, std::vector<Collider> source)
        {
            destination.insert(destination.end(), std::make_move_iterator(source.begin()), std::make_move_iterator(source.end()));
        }

        BodyDefinition placement_of(const RigidBody& body)
        {
            BodyDefinition definition;
            definition.name = body.name();
            definition.type = body.type();
            definition.position_m = body.position_m();
            definition.orientation_rad = body.orientation_rad();
            definition.linear_velocity_m_s = body.linear_velocity_m_s();
            definition.angular_velocity_rad_s = body.angular_velocity_rad_s();
            definition.linear_damping = body.linear_damping();
            definition.angular_damping = body.angular_damping();
            definition.gravity_scale = body.gravity_scale();
            definition.fixed_rotation = body.has_fixed_rotation();
            definition.sleep_enabled = body.is_sleep_enabled();
            return definition;
        }

        RigidBody validate_body(const World& world, const BodyDefinition& definition)
        {
            const auto& limits = world.settings().limits;
            require(definition.type == BodyType::dynamic_body || definition.type == BodyType::static_body || definition.type == BodyType::kinematic_body,
                "Authored body type is invalid");
            require(math::is_finite(definition.position_m) && std::abs(definition.position_m.x) <= limits.maximum_position_m &&
                    std::abs(definition.position_m.y) <= limits.maximum_position_m && math::is_finite(definition.orientation_rad) &&
                    std::abs(definition.orientation_rad) <= limits.maximum_orientation_rad,
                "Authored placement exceeds the world's finite motion limits");
            require(math::is_finite(definition.linear_velocity_m_s) && std::hypot(definition.linear_velocity_m_s.x, definition.linear_velocity_m_s.y) <= limits.maximum_linear_speed_m_s &&
                    math::is_finite(definition.angular_velocity_rad_s) && std::abs(definition.angular_velocity_rad_s) <= limits.maximum_angular_speed_rad_s,
                "Authored motion exceeds the world's finite speed limits");
            require(nonnegative(definition.linear_damping) && nonnegative(definition.angular_damping) && math::is_finite(definition.gravity_scale),
                "Authored motion settings must be finite");
            require(!definition.fixed_rotation || definition.angular_velocity_rad_s == 0.0,
                "A fixed-rotation authored body cannot have nonzero spin");
            RigidBody candidate(definition);
            const auto& mass = candidate.mass_properties();
            require(math::is_finite(mass.mass_kg) && math::is_finite(mass.center_of_mass_m) && math::is_finite(mass.inertia_kg_m2),
                "Authored mass properties cannot be represented");
            require(definition.type != BodyType::dynamic_body || (mass.mass_kg > 0.0 && mass.inertia_kg_m2 > 0.0),
                "A dynamic authored body requires positive geometry-defined mass and inertia");
            return candidate;
        }

        const RigidBody& require_body(const World& world, BodyId id)
        {
            const auto* body = world.find_body(id);
            require(body != nullptr, "The authored body no longer exists");
            return *body;
        }

        void require_unambiguous_mass(const RigidBody& body)
        {
            require(!body.has_mass_override(), "Remove the explicit mass override before changing authored geometry");
            require(body.accumulated_force_n() == math::Vec2 {} && body.accumulated_torque_n_m() == 0.0,
                "Apply pending body loads before changing authored geometry");
        }

        void require_detachable(const World& world, BodyId id)
        {
            const auto& body = require_body(world, id);
            require_unambiguous_mass(body);
            require(body.type() == BodyType::dynamic_body, "Assembly and separation require dynamic authored bodies");
            require(!body.has_fixed_rotation(), "Assembly and separation require freely rotating authored bodies");
            require(!body.colliders().empty() && std::all_of(body.colliders().begin(), body.colliders().end(), [](const auto& collider)
                                                     {
                                                         return collider.authored_part != nullptr;
                                                     }),
                "Every collider must belong to an authored logical part");
            require(world.force_generators(id).empty(), "Detach body-specific force generators before assembly or separation");
            require(world.kinematic_motion(id) == nullptr, "Detach prescribed motion before assembly or separation");
            for (const auto& constraint : world.constraints())
                require(!(constraint->first_body() == id) && !(constraint->second_body() == id), "Detach joints before assembly or separation");
            for (const auto spring : world.spring_ids())
                std::visit([&](const auto& definition)
                    {
                        require(!(definition.first == id) && !(definition.second == id), "Detach springs before assembly or separation");
                    },
                    *world.spring_definition(spring));
        }

        bool compatible(const RigidBody& first, const RigidBody& second)
        {
            return first.gravity_scale() == second.gravity_scale() && first.linear_damping() == second.linear_damping() &&
                first.angular_damping() == second.angular_damping() && first.is_sleep_enabled() == second.is_sleep_enabled();
        }
    }

    std::vector<AuthoredPartPtr> authored_parts(const RigidBody& body)
    {
        std::vector<AuthoredPartPtr> result;
        for (const auto& collider : body.colliders())
            if (collider.authored_part && std::find(result.begin(), result.end(), collider.authored_part) == result.end())
                result.push_back(collider.authored_part);
        return result;
    }

    BodyId create_authored_body(World& world, BodyDefinition placement, const std::vector<AuthoredPartDefinition>& parts, std::string stable_key)
    {
        require(placement.colliders.empty(), "Authored body placement must not contain additional colliders");
        require(!parts.empty(), "An authored body requires at least one logical part");
        for (const auto& part : parts)
            append(placement.colliders, cells(part, true));
        (void)validate_body(world, placement);
        auto draft = AuthoredBodyAccess::stage(world);
        const auto result = draft->create_body(placement, std::move(stable_key));
        AuthoredBodyAccess::publish(world, std::move(draft));
        return result;
    }

    void edit_authored_part(World& world, BodyId id, std::size_t part_index, std::shared_ptr<const AuthoredShape> shape)
    {
        const auto& body = require_body(world, id);
        require_unambiguous_mass(body);
        require(body.type() != BodyType::kinematic_body, "Stop prescribed kinematic authoring before changing the outline");
        const auto parts = authored_parts(body);
        require(part_index < parts.size(), "The authored logical part no longer exists");
        const auto& original = parts[part_index];
        auto replacement = *original;
        replacement.shape = std::move(shape);
        auto replacements = cells(std::move(replacement), true);
        auto definition = placement_of(body);
        bool inserted = false;
        for (const auto& collider : body.colliders())
        {
            if (collider.authored_part == original)
            {
                if (!inserted)
                {
                    append(definition.colliders, std::move(replacements));
                    inserted = true;
                }
            }
            else
                definition.colliders.push_back(collider);
        }
        const auto candidate = validate_body(world, definition);
        definition.linear_velocity_m_s = body.velocity_at_world_point(candidate.world_center_of_mass_m());
        (void)validate_body(world, definition);
        auto draft = AuthoredBodyAccess::stage(world);
        AuthoredBodyAccess::replace(*draft, id, definition);
        AuthoredBodyAccess::publish(world, std::move(draft));
    }

    BodyId assemble_authored_bodies(World& world, const std::vector<BodyId>& bodies, std::string name, std::string stable_key)
    {
        require(bodies.size() >= 2, "Assembly requires at least two authored bodies");
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            require_detachable(world, bodies[i]);
            require(std::find(bodies.begin(), bodies.begin() + static_cast<std::ptrdiff_t>(i), bodies[i]) == bodies.begin() + static_cast<std::ptrdiff_t>(i),
                "Assembly body selection must not contain duplicates");
        }
        std::vector<BodyId> ordered;
        for (const auto id : world.body_ids())
            if (std::find(bodies.begin(), bodies.end(), id) != bodies.end())
                ordered.push_back(id);
        const auto& first = *world.find_body(ordered.front());
        auto definition = placement_of(first);
        definition.name = std::move(name);
        definition.linear_velocity_m_s = {};
        definition.angular_velocity_rad_s = 0.0;
        const auto inverse_frame = math::inverse(first.transform());
        for (const auto id : ordered)
        {
            const auto& source = *world.find_body(id);
            require(compatible(first, source), "Assembly bodies must have matching gravity, damping, and sleeping settings");
            for (const auto& part : authored_parts(source))
            {
                auto placed = *part;
                placed.local_transform = math::concatenate(inverse_frame, math::concatenate(source.transform(), part->local_transform));
                append(definition.colliders, cells(std::move(placed), false));
            }
        }
        const auto candidate = validate_body(world, definition);
        const auto center = candidate.world_center_of_mass_m();
        math::Vec2 momentum;
        Real angular_momentum = 0.0;
        for (const auto id : ordered)
        {
            const auto& source = *world.find_body(id);
            momentum += source.linear_momentum_kg_m_s();
            angular_momentum += source.angular_momentum_about_center_kg_m2_s() + math::cross(source.world_center_of_mass_m() - center, source.linear_momentum_kg_m_s());
        }
        definition.linear_velocity_m_s = momentum / candidate.mass_properties().mass_kg;
        definition.angular_velocity_rad_s = angular_momentum / candidate.mass_properties().inertia_kg_m2;
        (void)validate_body(world, definition);
        auto draft = AuthoredBodyAccess::stage(world);
        const auto result = draft->create_body(definition, std::move(stable_key));
        for (const auto id : ordered)
            draft->destroy_body(id);
        AuthoredBodyAccess::publish(world, std::move(draft));
        return result;
    }

    std::vector<BodyId> separate_authored_body(World& world, BodyId id)
    {
        require_detachable(world, id);
        const auto& body = *world.find_body(id);
        const auto parts = authored_parts(body);
        require(parts.size() >= 2, "A single authored logical part cannot be separated into collision cells");
        std::vector<BodyDefinition> definitions;
        for (std::size_t index = 0; index < parts.size(); ++index)
        {
            auto definition = placement_of(body);
            definition.name = parts[index]->name.empty() ? body.name() + " / part " + std::to_string(index + 1) : parts[index]->name;
            definition.colliders = cells(*parts[index], false);
            const auto candidate = validate_body(world, definition);
            definition.linear_velocity_m_s = body.velocity_at_world_point(candidate.world_center_of_mass_m());
            (void)validate_body(world, definition);
            definitions.push_back(std::move(definition));
        }
        auto draft = AuthoredBodyAccess::stage(world);
        std::vector<BodyId> result;
        result.reserve(definitions.size());
        for (const auto& definition : definitions)
            result.push_back(draft->create_body(definition));
        draft->destroy_body(id);
        AuthoredBodyAccess::publish(world, std::move(draft));
        return result;
    }
}
