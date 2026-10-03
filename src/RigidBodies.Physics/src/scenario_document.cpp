#include <rigidbodies/physics/scenario_document.hpp>
#include <rigidbodies/physics/authored_body.hpp>
#include <rigidbodies/physics/joint.hpp>
#include <rigidbodies/physics/shape_document.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <type_traits>

namespace rigidbodies::physics
{
    struct ScenarioDocumentAccess
    {
        static void seed_identifiers(World& staged, const World& destination)
        {
            staged.next_generation_ = destination.next_generation_;
            staged.next_spring_generation_ = destination.next_spring_generation_;
        }
        static const std::string& body_key(const World& world, BodyId id)
        {
            return world.slots_[id.index].stable_key;
        }
        static const std::string& document_id(const World& world, BodyId id)
        {
            return world.slots_[id.index].document_id;
        }
        static void document_id(World& world, BodyId id, std::string value)
        {
            world.slots_[id.index].document_id = std::move(value);
        }
        static Real motion_elapsed(const World& world, BodyId id)
        {
            return world.statistics_.elapsed_time_s - world.slots_[id.index].motion_start_time_s;
        }
        static void motion(World& world, BodyId id, const KinematicMotion& value, Real elapsed)
        {
            const auto state = value.sample(elapsed);
            if (!world.accept_kinematic_state(id, state))
                throw std::runtime_error("Prescribed motion exceeds the world's motion limits");
            auto& slot = world.slots_[id.index];
            slot.motion = value;
            slot.motion_start_time_s = -elapsed;
        }
        static const std::string& spring_key(const World& world, SpringId id)
        {
            return world.spring_slots_[id.index].stable_key;
        }
        static void broken(JointConstraint& joint, bool value)
        {
            joint.broken_ = value;
        }
    };

    namespace
    {
        using Json = content::Json;
        using Object = Json::Object;
        using Array = Json::Array;
        constexpr std::size_t maximum_bodies = 4096;
        constexpr std::size_t maximum_parts = 16384;

        void require(bool condition, const std::string& message)
        {
            if (!condition)
                throw std::runtime_error(message);
        }
        const Json& object(const Json& value)
        {
            require(value.is_object(), "Expected a JSON object");
            return value;
        }
        const Array& array(const Json& value)
        {
            require(value.is_array(), "Expected a JSON array");
            return value.as_array();
        }
        Real number(const Json& value)
        {
            require(value.is_number() && std::isfinite(value.as_number()), "Expected a finite number");
            return value.as_number();
        }
        template <typename Integer>
        Integer integer(const Json& value)
        {
            const auto real = number(value);
            require(std::floor(real) == real && real >= static_cast<Real>(std::numeric_limits<Integer>::lowest()) && real <= static_cast<Real>(std::numeric_limits<Integer>::max()), "Integer is outside its supported range");
            return static_cast<Integer>(real);
        }
        std::string string(const Json& value)
        {
            require(value.is_string(), "Expected a string");
            require(value.as_string().size() <= 65536, "Document string is too long");
            return value.as_string();
        }
        math::Vec2 vector(const Json& value)
        {
            const auto& values = array(value);
            require(values.size() == 2, "A vector requires two coordinates");
            return { number(values[0]), number(values[1]) };
        }
        Json encode(math::Vec2 value)
        {
            return Array { value.x, value.y };
        }
        Json encode(const std::optional<Real>& value)
        {
            return value ? Json(*value) : Json(nullptr);
        }
        template <typename T>
        Json encode(const T& value)
        {
            return Json(value);
        }
        void decode(const Json& value, Real& result)
        {
            result = number(value);
        }
        void decode(const Json& value, int& result)
        {
            result = integer<int>(value);
        }
        void decode(const Json& value, std::uint32_t& result)
        {
            result = integer<std::uint32_t>(value);
        }
        void decode(const Json& value, bool& result)
        {
            require(value.is_bool(), "Expected a boolean");
            result = value.as_bool();
        }
        void decode(const Json& value, std::string& result)
        {
            result = string(value);
        }
        void decode(const Json& value, math::Vec2& result)
        {
            result = vector(value);
        }
        void decode(const Json& value, std::optional<Real>& result)
        {
            result = value.is_null() ? std::nullopt : std::optional<Real>(number(value));
        }
        template <typename T>
        void field(const Json& source, const char* name, T& value)
        {
            if (const auto* item = object(source).find(name))
                decode(*item, value);
        }
        const Json& required(const Json& source, std::string_view key)
        {
            object(source);
            const auto* value = source.find(key);
            require(value != nullptr, "Missing required field: " + std::string(key));
            return *value;
        }
        std::string kind(const Json& value)
        {
            return string(required(value, "type"));
        }
        void nonnegative(Real value, const char* description)
        {
            require(std::isfinite(value) && value >= 0, std::string(description) + " must be finite and nonnegative");
        }
        void positive(Real value, const char* description)
        {
            require(std::isfinite(value) && value > 0, std::string(description) + " must be finite and positive");
        }

#define WRITE_FIELD(member) result[#member] = encode(value.member)
#define READ_FIELD(member) field(source, #member, result.member)

        Json encode_metadata(const ScenarioMetadata& value)
        {
            Json result = Object {};
            WRITE_FIELD(id);
            WRITE_FIELD(title);
            WRITE_FIELD(summary);
            WRITE_FIELD(suggested_order);
            WRITE_FIELD(collection);
            WRITE_FIELD(level);
            WRITE_FIELD(hook);
            WRITE_FIELD(collection_order);
            WRITE_FIELD(lab);
            Array concepts, prerequisites, tags;
            for (const auto& item : value.concepts)
                concepts.emplace_back(item);
            for (const auto& item : value.prerequisites)
                prerequisites.emplace_back(item);
            for (const auto& item : value.tags)
                tags.emplace_back(item);
            result["concepts"] = concepts;
            result["prerequisites"] = prerequisites;
            result["tags"] = tags;
            return result;
        }
        ScenarioMetadata decode_metadata(const Json& source)
        {
            ScenarioMetadata result;
            result.id = string(required(source, "id"));
            result.title = string(required(source, "title"));
            require(!result.id.empty() && !result.title.empty(), "Scenario id and title must not be empty");
            READ_FIELD(summary);
            READ_FIELD(suggested_order);
            READ_FIELD(collection);
            READ_FIELD(level);
            READ_FIELD(hook);
            READ_FIELD(collection_order);
            READ_FIELD(lab);
            require(result.suggested_order >= 0, "Suggested order must be nonnegative");
            require(result.collection_order >= 0, "Collection order must be nonnegative");
            for (const auto* name : { "concepts", "prerequisites", "tags" })
            {
                if (const auto* values = source.find(name))
                {
                    require(array(*values).size() <= 256, "Scenario metadata contains too many entries");
                    auto& target = std::string_view(name) == "concepts" ? result.concepts : std::string_view(name) == "prerequisites" ? result.prerequisites
                                                                                                                                      : result.tags;
                    for (const auto& entry : values->as_array())
                    {
                        auto text = string(entry);
                        require(!text.empty(), "Metadata entries must not be empty");
                        target.push_back(std::move(text));
                    }
                }
            }
            return result;
        }
        void validate_envelope(const Json& root)
        {
            std::string error;
            const auto valid = content::validate_document_header(root, "rigid-bodies.scenario", error);
            require(valid, error);
            // Also fail closed on the early development spelling of required features.
            if (const auto* features = root.find("requires"))
                require(array(*features).empty(), "Document requires unsupported features");
        }
        const char* mixing_name(MaterialMixing value)
        {
            switch (value)
            {
            case MaterialMixing::geometric_mean:
                return "geometric_mean";
            case MaterialMixing::arithmetic_mean:
                return "arithmetic_mean";
            case MaterialMixing::minimum:
                return "minimum";
            case MaterialMixing::maximum:
                return "maximum";
            }
            throw std::runtime_error("Unsupported material mixing policy");
        }
        MaterialMixing mixing(const Json& value)
        {
            const auto name = string(value);
            if (name == "geometric_mean")
                return MaterialMixing::geometric_mean;
            if (name == "arithmetic_mean")
                return MaterialMixing::arithmetic_mean;
            if (name == "minimum")
                return MaterialMixing::minimum;
            if (name == "maximum")
                return MaterialMixing::maximum;
            throw std::runtime_error("Unsupported material mixing policy: " + name);
        }
        Json encode_settings(const WorldSettings& value)
        {
            Json result = Object {};
            WRITE_FIELD(gravity_m_s2);
            WRITE_FIELD(air_density_kg_m3);
            WRITE_FIELD(air_velocity_m_s);
            WRITE_FIELD(air_dynamic_viscosity_pa_s);
            WRITE_FIELD(broad_phase_margin_m);
            WRITE_FIELD(constraint_graph_enabled);
            result["solver"] = Object { { "velocity_iterations", value.solver.velocity_iterations }, { "position_iterations", value.solver.position_iterations }, { "linear_slop_m", value.solver.linear_slop_m }, { "position_correction_fraction", value.solver.position_correction_fraction }, { "restitution_threshold_m_s", value.solver.restitution_threshold_m_s }, { "warm_starting", value.solver.warm_starting }, { "maximum_position_correction_m", value.solver.maximum_position_correction_m } };
            result["sleep"] = Object { { "enabled", value.sleep.enabled }, { "linear_speed_m_s", value.sleep.linear_speed_m_s }, { "angular_speed_rad_s", value.sleep.angular_speed_rad_s }, { "linear_acceleration_m_s2", value.sleep.linear_acceleration_m_s2 }, { "angular_acceleration_rad_s2", value.sleep.angular_acceleration_rad_s2 }, { "quiet_duration_s", value.sleep.quiet_duration_s } };
            result["limits"] = Object { { "maximum_position_m", value.limits.maximum_position_m }, { "maximum_linear_speed_m_s", value.limits.maximum_linear_speed_m_s }, { "maximum_angular_speed_rad_s", value.limits.maximum_angular_speed_rad_s }, { "maximum_orientation_rad", value.limits.maximum_orientation_rad } };
            result["collision"] = Object { { "continuous", value.collision.continuous }, { "contact_margin_m", value.collision.contact_margin_m }, { "matching_tolerance_m", value.collision.matching_tolerance_m }, { "sweep_tolerance_m", value.collision.sweep_tolerance_m }, { "maximum_sweep_iterations", value.collision.maximum_sweep_iterations }, { "friction_mixing", mixing_name(value.collision.friction_mixing) }, { "restitution_mixing", mixing_name(value.collision.restitution_mixing) } };
            return result;
        }
        WorldSettings decode_settings(const Json& source)
        {
            WorldSettings result;
            READ_FIELD(gravity_m_s2);
            READ_FIELD(air_density_kg_m3);
            READ_FIELD(air_velocity_m_s);
            READ_FIELD(air_dynamic_viscosity_pa_s);
            READ_FIELD(broad_phase_margin_m);
            READ_FIELD(constraint_graph_enabled);
#define NESTED(group, member) field(*part, #member, result.group.member)
            if (const auto* part = source.find("solver"))
            {
                object(*part);
                NESTED(solver, velocity_iterations);
                NESTED(solver, position_iterations);
                NESTED(solver, linear_slop_m);
                NESTED(solver, position_correction_fraction);
                NESTED(solver, restitution_threshold_m_s);
                NESTED(solver, warm_starting);
                NESTED(solver, maximum_position_correction_m);
            }
            if (const auto* part = source.find("sleep"))
            {
                object(*part);
                NESTED(sleep, enabled);
                NESTED(sleep, linear_speed_m_s);
                NESTED(sleep, angular_speed_rad_s);
                NESTED(sleep, linear_acceleration_m_s2);
                NESTED(sleep, angular_acceleration_rad_s2);
                NESTED(sleep, quiet_duration_s);
            }
            if (const auto* part = source.find("limits"))
            {
                object(*part);
                NESTED(limits, maximum_position_m);
                NESTED(limits, maximum_linear_speed_m_s);
                NESTED(limits, maximum_angular_speed_rad_s);
                NESTED(limits, maximum_orientation_rad);
            }
            if (const auto* part = source.find("collision"))
            {
                object(*part);
                NESTED(collision, continuous);
                NESTED(collision, contact_margin_m);
                NESTED(collision, matching_tolerance_m);
                NESTED(collision, sweep_tolerance_m);
                NESTED(collision, maximum_sweep_iterations);
                if (auto* v = part->find("friction_mixing"))
                    result.collision.friction_mixing = mixing(*v);
                if (auto* v = part->find("restitution_mixing"))
                    result.collision.restitution_mixing = mixing(*v);
            }
#undef NESTED
            return result;
        }
        Json encode_material(const Material& value)
        {
            Json result = Object {};
            WRITE_FIELD(name);
            WRITE_FIELD(density_kg_m3);
            WRITE_FIELD(restitution);
            WRITE_FIELD(static_friction);
            WRITE_FIELD(kinetic_friction);
            WRITE_FIELD(drag_coefficient);
            WRITE_FIELD(rolling_friction_m);
            WRITE_FIELD(spinning_friction_m);
            WRITE_FIELD(friction_axis_local);
            WRITE_FIELD(friction_anisotropy_ratio);
            return result;
        }
        Material decode_material(const Json& source)
        {
            Material result;
            READ_FIELD(name);
            READ_FIELD(density_kg_m3);
            READ_FIELD(restitution);
            READ_FIELD(static_friction);
            READ_FIELD(kinetic_friction);
            READ_FIELD(drag_coefficient);
            READ_FIELD(rolling_friction_m);
            READ_FIELD(spinning_friction_m);
            READ_FIELD(friction_axis_local);
            READ_FIELD(friction_anisotropy_ratio);
            nonnegative(result.density_kg_m3, "Material density");
            nonnegative(result.restitution, "Restitution");
            require(result.restitution <= 1, "Restitution must not exceed one");
            nonnegative(result.static_friction, "Static friction");
            nonnegative(result.kinetic_friction, "Kinetic friction");
            nonnegative(result.drag_coefficient, "Drag coefficient");
            nonnegative(result.rolling_friction_m, "Rolling friction");
            nonnegative(result.spinning_friction_m, "Spinning friction");
            nonnegative(result.friction_anisotropy_ratio, "Friction anisotropy");
            require(math::length_squared(result.friction_axis_local) > 0, "Friction axis must be nonzero");
            return result;
        }
        Json encode_transform(const math::Transform2& value)
        {
            return Object { { "translation_m", encode(value.translation) }, { "rotation_rad", value.rotation.angle() } };
        }
        math::Transform2 decode_transform(const Json& source)
        {
            math::Vec2 translation;
            Real rotation {};
            field(source, "translation_m", translation);
            field(source, "rotation_rad", rotation);
            return math::Transform2::from_angle(translation, rotation);
        }
        template <typename Part>
        Json encode_part_properties(const Part& value)
        {
            Json result = Object {};
            WRITE_FIELD(depth_m);
            WRITE_FIELD(density_override_kg_m3);
            WRITE_FIELD(drag_coefficient_override);
            WRITE_FIELD(is_sensor);
            result["material"] = encode_material(value.material);
            result["local_transform"] = encode_transform(value.local_transform);
            result["filter"] = Object { { "category", value.filter.category }, { "mask", value.filter.mask }, { "group", value.filter.group } };
            return result;
        }
        template <typename Part>
        void decode_part_properties(const Json& source, Part& result)
        {
            READ_FIELD(depth_m);
            READ_FIELD(density_override_kg_m3);
            READ_FIELD(drag_coefficient_override);
            READ_FIELD(is_sensor);
            nonnegative(result.depth_m, "Part depth");
            if (result.density_override_kg_m3)
                nonnegative(*result.density_override_kg_m3, "Part density");
            if (result.drag_coefficient_override)
                nonnegative(*result.drag_coefficient_override, "Part drag coefficient");
            if (const auto* v = source.find("material"))
                result.material = decode_material(*v);
            if (const auto* v = source.find("local_transform"))
                result.local_transform = decode_transform(*v);
            if (const auto* v = source.find("filter"))
            {
                field(*v, "category", result.filter.category);
                field(*v, "mask", result.filter.mask);
                field(*v, "group", result.filter.group);
            }
        }
        Json encode_shape(const Shape& shape)
        {
            if (const auto* value = dynamic_cast<const CircleShape*>(&shape))
                return Object { { "type", "circle" }, { "radius_m", value->radius_m() }, { "center_m", encode(value->local_center_m()) } };
            if (const auto* value = dynamic_cast<const SegmentShape*>(&shape))
                return Object { { "type", "segment" }, { "start_m", encode(value->start_m()) }, { "end_m", encode(value->end_m()) } };
            if (const auto* value = dynamic_cast<const ConvexPolygonShape*>(&shape))
            {
                Array vertices;
                for (auto vertex : value->vertices())
                    vertices.push_back(encode(vertex));
                return Object { { "type", "convex_polygon" }, { "vertices_m", vertices } };
            }
            throw std::runtime_error("Cannot save an unsupported custom collision shape");
        }
        ShapePtr decode_shape(const Json& source)
        {
            const auto type = kind(source);
            if (type == "circle")
            {
                const auto radius = number(required(source, "radius_m"));
                positive(radius, "Circle radius");
                return make_circle(radius, vector(required(source, "center_m")));
            }
            if (type == "segment")
                return make_segment(vector(required(source, "start_m")), vector(required(source, "end_m")));
            if (type == "convex_polygon")
            {
                std::vector<math::Vec2> vertices;
                const auto& values = array(required(source, "vertices_m"));
                require(values.size() >= 3 && values.size() <= 4096, "Invalid convex polygon vertex count");
                for (const auto& v : values)
                    vertices.push_back(vector(v));
                return std::make_shared<ConvexPolygonShape>(std::move(vertices));
            }
            throw std::runtime_error("Unsupported collision shape type: " + type);
        }
        Json encode_force(const ForceGenerator& force)
        {
            Json result = Object { { "enabled", force.is_enabled() } };
            if (dynamic_cast<const UniformGravity*>(&force))
                result["type"] = "uniform_gravity";
            else if (const auto* attractor = dynamic_cast<const PointAttractor*>(&force))
            {
                result["type"] = "point_attractor";
                result["position_m"] = encode(attractor->world_position_m());
                result["stiffness_n_m"] = attractor->stiffness_n_m();
            }
            else if (const auto* drag = dynamic_cast<const AerodynamicDrag*>(&force))
            {
                result["type"] = "aerodynamic_drag";
                const auto& value = drag->settings();
                WRITE_FIELD(estimate_outline_coefficient);
                WRITE_FIELD(reynolds_correction);
                WRITE_FIELD(angular_drag);
                WRITE_FIELD(magnus_lift);
                WRITE_FIELD(angular_drag_coefficient);
                WRITE_FIELD(magnus_efficiency);
                WRITE_FIELD(maximum_lift_coefficient);
            }
            else
                throw std::runtime_error("Cannot save unsupported custom force generator: " + std::string(force.name()));
            return result;
        }
        ForceGeneratorPtr decode_force(const Json& source)
        {
            ForceGeneratorPtr result;
            const auto type = kind(source);
            if (type == "uniform_gravity")
                result = std::make_shared<UniformGravity>();
            else if (type == "point_attractor")
            {
                const auto stiffness = number(required(source, "stiffness_n_m"));
                nonnegative(stiffness, "Attractor stiffness");
                result = std::make_shared<PointAttractor>(vector(required(source, "position_m")), stiffness);
            }
            else if (type == "aerodynamic_drag")
            {
                AerodynamicSettings settings;
                field(source, "estimate_outline_coefficient", settings.estimate_outline_coefficient);
                field(source, "reynolds_correction", settings.reynolds_correction);
                field(source, "angular_drag", settings.angular_drag);
                field(source, "magnus_lift", settings.magnus_lift);
                field(source, "angular_drag_coefficient", settings.angular_drag_coefficient);
                field(source, "magnus_efficiency", settings.magnus_efficiency);
                field(source, "maximum_lift_coefficient", settings.maximum_lift_coefficient);
                result = std::make_shared<AerodynamicDrag>(settings);
            }
            else
                throw std::runtime_error("Unsupported force generator type: " + type);
            bool enabled = true;
            field(source, "enabled", enabled);
            result->set_enabled(enabled);
            return result;
        }
        Json encode_motion(const KinematicMotion& motion, Real elapsed)
        {
            return std::visit([&](const auto& value) -> Json
                {
                    using T = std::decay_t<decltype(value)>;
                    Json result = Object { { "elapsed_s", elapsed } };
                    if constexpr (std::is_same_v<T, LinearMotion>)
                    {
                        result["type"] = "linear";
                        WRITE_FIELD(origin_m);
                        WRITE_FIELD(initial_orientation_rad);
                        WRITE_FIELD(velocity_m_s);
                        WRITE_FIELD(angular_velocity_rad_s);
                    }
                    else if constexpr (std::is_same_v<T, HarmonicMotion>)
                    {
                        result["type"] = "harmonic";
                        WRITE_FIELD(origin_m);
                        WRITE_FIELD(initial_orientation_rad);
                        WRITE_FIELD(translation_amplitude_m);
                        WRITE_FIELD(rotation_amplitude_rad);
                        WRITE_FIELD(frequency_hz);
                        WRITE_FIELD(phase_rad);
                    }
                    else
                    {
                        result["type"] = "circular";
                        WRITE_FIELD(center_m);
                        WRITE_FIELD(radius_m);
                        WRITE_FIELD(angular_speed_rad_s);
                        WRITE_FIELD(phase_rad);
                        WRITE_FIELD(orientation_offset_rad);
                        WRITE_FIELD(orient_to_path);
                    }
                    return result;
                },
                motion.definition());
        }
        KinematicMotion decode_motion(const Json& source)
        {
            const auto type = kind(source);
            if (type == "linear")
            {
                LinearMotion result;
                READ_FIELD(origin_m);
                READ_FIELD(initial_orientation_rad);
                READ_FIELD(velocity_m_s);
                READ_FIELD(angular_velocity_rad_s);
                return KinematicMotion(result);
            }
            if (type == "harmonic")
            {
                HarmonicMotion result;
                READ_FIELD(origin_m);
                READ_FIELD(initial_orientation_rad);
                READ_FIELD(translation_amplitude_m);
                READ_FIELD(rotation_amplitude_rad);
                READ_FIELD(frequency_hz);
                READ_FIELD(phase_rad);
                return KinematicMotion(result);
            }
            if (type == "circular")
            {
                CircularMotion result;
                READ_FIELD(center_m);
                READ_FIELD(radius_m);
                READ_FIELD(angular_speed_rad_s);
                READ_FIELD(phase_rad);
                READ_FIELD(orientation_offset_rad);
                READ_FIELD(orient_to_path);
                return KinematicMotion(result);
            }
            throw std::runtime_error("Unsupported prescribed motion type: " + type);
        }
        Json encode_algorithms(const World& world)
        {
            Json result = Object {};
            if (dynamic_cast<const SemiImplicitEulerIntegrator*>(&world.integrator()))
                result["integrator"] = "semi_implicit_euler";
            else if (dynamic_cast<const VelocityVerletIntegrator*>(&world.integrator()))
                result["integrator"] = "velocity_verlet";
            else if (dynamic_cast<const RungeKutta4Integrator*>(&world.integrator()))
                result["integrator"] = "runge_kutta_4";
            else
                throw std::runtime_error("Cannot save an unsupported custom integrator");
            if (dynamic_cast<const BruteForceBroadPhase*>(&world.broad_phase()))
                result["broad_phase"] = Object { { "type", "brute_force" } };
            else if (const auto* tree = dynamic_cast<const DynamicTreeBroadPhase*>(&world.broad_phase()))
                result["broad_phase"] = Object { { "type", "dynamic_tree" }, { "base_margin_m", tree->settings().base_margin_m }, { "displacement_multiplier", tree->settings().displacement_multiplier } };
            else
                throw std::runtime_error("Cannot save an unsupported custom broad phase");
            if (dynamic_cast<const CollisionNarrowPhase*>(&world.narrow_phase()))
                result["narrow_phase"] = "collision";
            else if (dynamic_cast<const NullNarrowPhase*>(&world.narrow_phase()))
                result["narrow_phase"] = "none";
            else
                throw std::runtime_error("Cannot save an unsupported custom narrow phase");
            if (dynamic_cast<const SequentialImpulseContactSolver*>(&world.contact_solver()))
                result["contact_solver"] = "sequential_impulse";
            else if (dynamic_cast<const NullContactSolver*>(&world.contact_solver()))
                result["contact_solver"] = "none";
            else
                throw std::runtime_error("Cannot save an unsupported custom contact solver");
            return result;
        }
        void decode_algorithms(const Json& source, World& world)
        {
            const auto integrator = string(required(source, "integrator"));
            if (integrator == "semi_implicit_euler")
                world.set_integrator(std::make_shared<SemiImplicitEulerIntegrator>());
            else if (integrator == "velocity_verlet")
                world.set_integrator(std::make_shared<VelocityVerletIntegrator>());
            else if (integrator == "runge_kutta_4")
                world.set_integrator(std::make_shared<RungeKutta4Integrator>());
            else
                throw std::runtime_error("Unsupported integrator: " + integrator);
            const auto& broad = required(source, "broad_phase");
            const auto broad_type = kind(broad);
            if (broad_type == "brute_force")
                world.set_broad_phase(std::make_shared<BruteForceBroadPhase>());
            else if (broad_type == "dynamic_tree")
            {
                DynamicTreeSettings settings;
                field(broad, "base_margin_m", settings.base_margin_m);
                field(broad, "displacement_multiplier", settings.displacement_multiplier);
                world.set_broad_phase(std::make_shared<DynamicTreeBroadPhase>(settings));
            }
            else
                throw std::runtime_error("Unsupported broad phase: " + broad_type);
            const auto narrow = string(required(source, "narrow_phase"));
            if (narrow == "collision")
                world.set_narrow_phase(std::make_shared<CollisionNarrowPhase>());
            else if (narrow == "none")
                world.set_narrow_phase(std::make_shared<NullNarrowPhase>());
            else
                throw std::runtime_error("Unsupported narrow phase: " + narrow);
            const auto solver = string(required(source, "contact_solver"));
            if (solver == "sequential_impulse")
                world.set_contact_solver(std::make_shared<SequentialImpulseContactSolver>());
            else if (solver == "none")
                world.set_contact_solver(std::make_shared<NullContactSolver>());
            else
                throw std::runtime_error("Unsupported contact solver: " + solver);
        }
        using BodyReferences = std::vector<std::pair<BodyId, std::string>>;
        std::string reference(const BodyReferences& references, BodyId id)
        {
            const auto found = std::find_if(references.begin(), references.end(), [&](const auto& item)
                {
                    return item.first == id;
                });
            require(found != references.end(), "A component refers to a missing body");
            return found->second;
        }
        BodyId reference(const std::map<std::string, BodyId>& references, const Json& value)
        {
            const auto name = string(value);
            const auto found = references.find(name);
            require(found != references.end(), "Unknown body reference: " + name);
            return found->second;
        }
        Json encode_body(const World& world, BodyId id, const std::string& document_id)
        {
            const auto& body = *world.find_body(id);
            const char* type = body.type() == BodyType::dynamic_body ? "dynamic" : body.type() == BodyType::kinematic_body ? "kinematic"
                                                                                                                           : "static";
            Json result = Object { { "id", document_id }, { "stable_key", ScenarioDocumentAccess::body_key(world, id) }, { "name", body.name() }, { "type", type }, { "position_m", encode(body.position_m()) }, { "orientation_rad", body.orientation_rad() }, { "linear_velocity_m_s", encode(body.linear_velocity_m_s()) }, { "angular_velocity_rad_s", body.angular_velocity_rad_s() }, { "linear_damping", body.linear_damping() }, { "angular_damping", body.angular_damping() }, { "gravity_scale", body.gravity_scale() }, { "fixed_rotation", body.has_fixed_rotation() }, { "sleep_enabled", body.is_sleep_enabled() }, { "awake", body.is_awake() }, { "mass_override_kg", body.has_mass_override() ? Json(body.mass_properties().mass_kg) : Json(nullptr) }, { "pending_force_n", encode(body.accumulated_force_n()) }, { "pending_torque_n_m", body.accumulated_torque_n_m() } };
            Array parts;
            std::set<const AuthoredPartDefinition*> authored;
            for (const auto& collider : body.colliders())
            {
                if (collider.authored_part)
                {
                    const auto& part = *collider.authored_part;
                    if (!authored.insert(&part).second)
                        continue;
                    require(part.shape != nullptr, "Authored part has no source shape");
                    auto item = encode_part_properties(part);
                    item["type"] = "authored";
                    item["name"] = part.name;
                    item["shape"] = encode_authored_shape(*part.shape);
                    item["source_document"] = nullptr;
                    if (part.source_document)
                    {
                        ShapeDocument source_document;
                        std::string error;
                        const auto captured = capture_shape_document(*part.shape, part.source_document->title, part.source_document->summary, source_document, error, part.source_document.get());
                        require(captured, "Could not save imported shape provenance: " + error);
                        item["source_document"] = std::move(source_document.root);
                    }
                    parts.push_back(std::move(item));
                }
                else
                {
                    require(collider.shape != nullptr, "Collider has no shape");
                    auto item = encode_part_properties(collider);
                    item["type"] = "primitive";
                    item["shape"] = encode_shape(*collider.shape);
                    item["shell_thickness_m"] = encode(collider.shell_thickness_m);
                    parts.push_back(std::move(item));
                }
            }
            result["parts"] = parts;
            Array forces;
            for (const auto& generator : world.force_generators(id))
                forces.push_back(encode_force(*generator));
            result["forces"] = forces;
            result["motion"] = world.kinematic_motion(id) ? encode_motion(*world.kinematic_motion(id), ScenarioDocumentAccess::motion_elapsed(world, id)) : Json(nullptr);
            return result;
        }
        BodyDefinition decode_body(const Json& source, std::size_t& part_count)
        {
            BodyDefinition result;
            READ_FIELD(name);
            READ_FIELD(position_m);
            READ_FIELD(orientation_rad);
            READ_FIELD(linear_velocity_m_s);
            READ_FIELD(angular_velocity_rad_s);
            READ_FIELD(linear_damping);
            READ_FIELD(angular_damping);
            READ_FIELD(gravity_scale);
            READ_FIELD(fixed_rotation);
            READ_FIELD(sleep_enabled);
            const auto type = kind(source);
            if (type == "dynamic")
                result.type = BodyType::dynamic_body;
            else if (type == "kinematic")
                result.type = BodyType::kinematic_body;
            else if (type == "static")
                result.type = BodyType::static_body;
            else
                throw std::runtime_error("Unsupported body type: " + type);
            nonnegative(result.linear_damping, "Linear damping");
            nonnegative(result.angular_damping, "Angular damping");
            require(!result.fixed_rotation || result.angular_velocity_rad_s == 0, "A fixed-rotation body must not have angular velocity");
            for (const auto& part : array(required(source, "parts")))
            {
                require(++part_count <= maximum_parts, "Scenario has too many parts");
                const auto part_type = kind(part);
                if (part_type == "primitive")
                {
                    Collider collider;
                    decode_part_properties(part, collider);
                    collider.shape = decode_shape(required(part, "shape"));
                    field(part, "shell_thickness_m", collider.shell_thickness_m);
                    if (collider.shell_thickness_m)
                        nonnegative(*collider.shell_thickness_m, "Shell thickness");
                    result.colliders.push_back(std::move(collider));
                }
                else if (part_type == "authored")
                {
                    auto definition = std::make_shared<AuthoredPartDefinition>();
                    decode_part_properties(part, *definition);
                    field(part, "name", definition->name);
                    std::string error;
                    const auto valid = decode_authored_shape(required(part, "shape"), definition->shape, error);
                    require(valid, "Invalid authored shape: " + error);
                    if (const auto* embedded = part.find("source_document"))
                        if (!embedded->is_null())
                        {
                            auto source_document = std::make_shared<ShapeDocument>();
                            const auto parsed = parse_shape_document(content::write_json(*embedded), *source_document, error);
                            require(parsed, "Invalid imported shape provenance: " + error);
                            definition->source_document = std::move(source_document);
                        }
                    require(result.colliders.size() + definition->shape->convex_parts.size() <= maximum_parts, "Authored body has too many convex cells");
                    for (const auto& vertices : definition->shape->convex_parts)
                    {
                        Collider collider;
                        collider.shape = std::make_shared<ConvexPolygonShape>(vertices);
                        collider.local_transform = definition->local_transform;
                        collider.material = definition->material;
                        collider.depth_m = definition->depth_m;
                        collider.filter = definition->filter;
                        collider.density_override_kg_m3 = definition->density_override_kg_m3;
                        collider.drag_coefficient_override = definition->drag_coefficient_override;
                        collider.is_sensor = definition->is_sensor;
                        collider.authored_part = definition;
                        result.colliders.push_back(std::move(collider));
                    }
                }
                else
                    throw std::runtime_error("Unsupported body part type: " + part_type);
            }
            return result;
        }
        void validate_body_definition(const BodyDefinition& definition, const MotionLimits& limits)
        {
            require(std::abs(definition.position_m.x) <= limits.maximum_position_m && std::abs(definition.position_m.y) <= limits.maximum_position_m &&
                    std::abs(definition.orientation_rad) <= limits.maximum_orientation_rad &&
                    std::hypot(definition.linear_velocity_m_s.x, definition.linear_velocity_m_s.y) <= limits.maximum_linear_speed_m_s &&
                    std::abs(definition.angular_velocity_rad_s) <= limits.maximum_angular_speed_rad_s,
                "Body placement or velocity exceeds the document's motion limits");
            for (const auto& collider : definition.colliders)
            {
                require(collider.shape->compute_mass_properties(1).is_valid() && math::is_finite(collider.shape->bounding_radius()), "Shape geometry produces unrepresentable mass or dimensions");
                require(collider.compute_mass_properties().is_valid(), "Collider material or placement produces unrepresentable mass properties");
                const auto bounds = collider.compute_bounds(math::Transform2::from_angle(definition.position_m, definition.orientation_rad));
                require(math::is_finite(bounds.minimum) && math::is_finite(bounds.maximum) && math::is_finite(bounds.extents()), "Collider bounds are not representable");
            }
        }
        Json encode_spring(const SpringDefinition& spring, const BodyReferences& references, const std::string& key)
        {
            return std::visit([&](const auto& value) -> Json
                {
                    Json result = Object { { "stable_key", key }, { "first", reference(references, value.first) }, { "second", reference(references, value.second) } };
                    WRITE_FIELD(enabled);
                    if constexpr (std::is_same_v<std::decay_t<decltype(value)>, LinearSpringDefinition>)
                    {
                        result["type"] = "linear";
                        WRITE_FIELD(local_anchor_first_m);
                        WRITE_FIELD(local_anchor_second_m);
                        WRITE_FIELD(rest_length_m);
                        WRITE_FIELD(stiffness_n_m);
                        WRITE_FIELD(damping_n_s_m);
                    }
                    else
                    {
                        result["type"] = "angular";
                        WRITE_FIELD(rest_angle_rad);
                        WRITE_FIELD(stiffness_n_m_rad);
                        WRITE_FIELD(damping_n_m_s_rad);
                    }
                    return result;
                },
                spring);
        }
        SpringDefinition decode_spring(const Json& source, const std::map<std::string, BodyId>& references)
        {
            const auto first = reference(references, required(source, "first")), second = reference(references, required(source, "second"));
            const auto type = kind(source);
            if (type == "linear")
            {
                LinearSpringDefinition result;
                result.first = first;
                result.second = second;
                READ_FIELD(enabled);
                READ_FIELD(local_anchor_first_m);
                READ_FIELD(local_anchor_second_m);
                READ_FIELD(rest_length_m);
                READ_FIELD(stiffness_n_m);
                READ_FIELD(damping_n_s_m);
                return result;
            }
            if (type == "angular")
            {
                AngularSpringDefinition result;
                result.first = first;
                result.second = second;
                READ_FIELD(enabled);
                READ_FIELD(rest_angle_rad);
                READ_FIELD(stiffness_n_m_rad);
                READ_FIELD(damping_n_m_s_rad);
                return result;
            }
            throw std::runtime_error("Unsupported spring type: " + type);
        }
        Json threshold(Real value)
        {
            require(!std::isnan(value) && value >= 0, "Invalid joint breaking threshold");
            return std::isinf(value) ? Json(nullptr) : Json(value);
        }
        Json encode_joint(const JointConstraint& joint, const BodyReferences& references, const std::string& key)
        {
            return std::visit([&](const auto& value) -> Json
                {
                    using T = std::decay_t<decltype(value)>;
                    Json result = Object { { "stable_key", key }, { "first", reference(references, value.first) }, { "second", reference(references, value.second) }, { "enabled", joint.is_enabled() }, { "broken", joint.is_broken() }, { "break_force_n", threshold(value.break_force_n) }, { "break_torque_n_m", threshold(value.break_torque_n_m) } };
                    WRITE_FIELD(local_anchor_first_m);
                    WRITE_FIELD(local_anchor_second_m);
                    WRITE_FIELD(collide_connected);
                    if constexpr (std::is_same_v<T, DistanceJointDefinition>)
                    {
                        result["type"] = "distance";
                        WRITE_FIELD(length_m);
                    }
                    else
                    {
                        WRITE_FIELD(reference_angle_rad);
                        if constexpr (std::is_same_v<T, RevoluteJointDefinition>)
                        {
                            result["type"] = "revolute";
                            WRITE_FIELD(limits_enabled);
                            WRITE_FIELD(lower_angle_rad);
                            WRITE_FIELD(upper_angle_rad);
                            WRITE_FIELD(motor_enabled);
                            WRITE_FIELD(motor_speed_rad_s);
                            WRITE_FIELD(maximum_motor_torque_n_m);
                        }
                        else if constexpr (std::is_same_v<T, PrismaticJointDefinition>)
                        {
                            result["type"] = "prismatic";
                            WRITE_FIELD(local_axis_first);
                            WRITE_FIELD(limits_enabled);
                            WRITE_FIELD(lower_translation_m);
                            WRITE_FIELD(upper_translation_m);
                            WRITE_FIELD(motor_enabled);
                            WRITE_FIELD(motor_speed_m_s);
                            WRITE_FIELD(maximum_motor_force_n);
                        }
                        else
                            result["type"] = "weld";
                    }
                    return result;
                },
                joint.definition());
        }
        template <typename T>
        void decode_endpoints(const Json& source, const std::map<std::string, BodyId>& references, T& result)
        {
            result.first = reference(references, required(source, "first"));
            result.second = reference(references, required(source, "second"));
            READ_FIELD(local_anchor_first_m);
            READ_FIELD(local_anchor_second_m);
            READ_FIELD(collide_connected);
            if (const auto* value = source.find("break_force_n"))
                result.break_force_n = value->is_null() ? std::numeric_limits<Real>::infinity() : number(*value);
            if (const auto* value = source.find("break_torque_n_m"))
                result.break_torque_n_m = value->is_null() ? std::numeric_limits<Real>::infinity() : number(*value);
        }
        JointDefinition decode_joint(const Json& source, const std::map<std::string, BodyId>& references)
        {
            const auto type = kind(source);
            if (type == "distance")
            {
                DistanceJointDefinition result;
                decode_endpoints(source, references, result);
                READ_FIELD(length_m);
                return result;
            }
            if (type == "revolute")
            {
                RevoluteJointDefinition result;
                decode_endpoints(source, references, result);
                READ_FIELD(reference_angle_rad);
                READ_FIELD(limits_enabled);
                READ_FIELD(lower_angle_rad);
                READ_FIELD(upper_angle_rad);
                READ_FIELD(motor_enabled);
                READ_FIELD(motor_speed_rad_s);
                READ_FIELD(maximum_motor_torque_n_m);
                return result;
            }
            if (type == "prismatic")
            {
                PrismaticJointDefinition result;
                decode_endpoints(source, references, result);
                READ_FIELD(reference_angle_rad);
                READ_FIELD(local_axis_first);
                READ_FIELD(limits_enabled);
                READ_FIELD(lower_translation_m);
                READ_FIELD(upper_translation_m);
                READ_FIELD(motor_enabled);
                READ_FIELD(motor_speed_m_s);
                READ_FIELD(maximum_motor_force_n);
                return result;
            }
            if (type == "weld")
            {
                WeldJointDefinition result;
                decode_endpoints(source, references, result);
                READ_FIELD(reference_angle_rad);
                return result;
            }
            throw std::runtime_error("Unsupported joint type: " + type);
        }
        // Unknown fields stay attached to stable body identifiers. Other fixed-layout objects
        // inherit additive fields only when their variant and collection position still match.
        Json inherit(const Json& previous, const Json& current)
        {
            if (previous.is_object() && current.is_object())
            {
                if (const auto* a = previous.find("type"))
                    if (const auto* b = current.find("type"))
                        if (a->is_string() && b->is_string() && a->as_string() != b->as_string())
                            return current;
                auto result = previous;
                for (const auto& entry : current.as_object())
                {
                    const auto* old = previous.find(entry.first);
                    // Shape provenance already carries its own complete source envelope. Its
                    // metadata follows a logical part, never an old scenario array position.
                    result[entry.first] = old && entry.first != "source_document" ? inherit(*old, entry.second) : entry.second;
                }
                return result;
            }
            if (previous.is_array() && current.is_array())
            {
                Array result;
                const auto& old = previous.as_array();
                for (std::size_t index = 0; index < current.as_array().size(); ++index)
                {
                    const auto& item = current.as_array()[index];
                    const Json* matching = nullptr;
                    if (item.is_object())
                    {
                        if (const auto* id = item.find("id"))
                        {
                            for (const auto& candidate : old)
                                if (candidate.is_object())
                                    if (const auto* old_id = candidate.find("id"))
                                        if (id->is_string() && old_id->is_string() && id->as_string() == old_id->as_string())
                                        {
                                            matching = &candidate;
                                            break;
                                        }
                        }
                        else if (old.size() == current.as_array().size())
                            matching = &old[index];
                    }
                    result.push_back(matching ? inherit(*matching, item) : item);
                }
                return result;
            }
            return current;
        }
#undef WRITE_FIELD
#undef READ_FIELD
    }

    bool populate_world(const ScenarioDocument& document, World& world, std::string& error)
    {
        try
        {
            validate_envelope(document.root);
            (void)decode_metadata(required(document.root, "metadata"));
            const auto& source = required(document.root, "world");
            object(source);
            World staged(source.find("settings") ? decode_settings(*source.find("settings")) : WorldSettings {});
            ScenarioDocumentAccess::seed_identifiers(staged, world);
            staged.set_parallel_settings(world.parallel_settings());
            staged.set_profiling_enabled(world.profiling_enabled());
            Real reference_height = 0;
            field(source, "potential_energy_reference_height_m", reference_height);
            staged.set_potential_energy_reference_height(reference_height);
            if (const auto* algorithms = source.find("algorithms"))
                decode_algorithms(*algorithms, staged);
            const auto old_forces = staged.force_generators();
            for (const auto& force : old_forces)
                staged.remove_force_generator(force);
            if (const auto* forces = source.find("forces"))
            {
                require(array(*forces).size() <= 1024, "Too many world force generators");
                for (const auto& force : forces->as_array())
                    staged.add_force_generator(decode_force(force));
            }
            const auto& bodies = array(required(source, "bodies"));
            require(bodies.size() <= maximum_bodies, "Scenario has too many bodies");
            std::map<std::string, BodyId> references;
            std::vector<std::pair<BodyId, bool>> awake;
            std::size_t part_count = 0;
            for (const auto& body : bodies)
            {
                const auto id = string(required(body, "id"));
                require(!id.empty() && references.find(id) == references.end(), "Body identifiers must be nonempty and unique");
                auto definition = decode_body(body, part_count);
                validate_body_definition(definition, staged.settings().limits);
                std::string key;
                field(body, "stable_key", key);
                const auto handle = staged.create_body(definition, key);
                references.emplace(id, handle);
                ScenarioDocumentAccess::document_id(staged, handle, id);
                auto* live = staged.find_body(handle);
                if (const auto* mass = body.find("mass_override_kg"))
                    if (!mass->is_null())
                    {
                        const auto value = number(*mass);
                        positive(value, "Mass override");
                        live->override_mass(value);
                    }
                require(live->mass_properties().is_valid() && math::is_finite(live->world_center_of_mass_m()), "Body has unrepresentable combined mass properties");
                if (const auto* forces = body.find("forces"))
                {
                    require(array(*forces).size() <= 1024, "Too many body force generators");
                    for (const auto& force : forces->as_array())
                        require(staged.add_force_generator(handle, decode_force(force)), "Could not attach a body force generator");
                }
                if (const auto* motion = body.find("motion"))
                    if (!motion->is_null())
                    {
                        require(definition.type == BodyType::kinematic_body, "Only kinematic bodies accept prescribed motion");
                        Real elapsed = 0;
                        field(*motion, "elapsed_s", elapsed);
                        nonnegative(elapsed, "Motion elapsed time");
                        ScenarioDocumentAccess::motion(staged, handle, decode_motion(*motion), elapsed);
                    }
                math::Vec2 force;
                Real torque = 0;
                field(body, "pending_force_n", force);
                field(body, "pending_torque_n_m", torque);
                live->apply_force_at_center(force);
                live->apply_torque(torque);
                bool is_awake = true;
                field(body, "awake", is_awake);
                awake.emplace_back(handle, is_awake);
            }
            if (const auto* springs = source.find("springs"))
            {
                require(array(*springs).size() <= maximum_parts, "Too many springs");
                for (const auto& spring : springs->as_array())
                {
                    std::string key;
                    field(spring, "stable_key", key);
                    const auto id = staged.create_spring(decode_spring(spring, references), key);
                    require(id.is_valid(), "Invalid spring endpoints");
                }
            }
            if (const auto* joints = source.find("joints"))
            {
                require(array(*joints).size() <= maximum_parts, "Too many joints");
                for (const auto& item : joints->as_array())
                {
                    auto joint = std::make_shared<JointConstraint>(decode_joint(item, references));
                    bool enabled = true, broken = false;
                    field(item, "enabled", enabled);
                    field(item, "broken", broken);
                    joint->set_enabled(enabled);
                    ScenarioDocumentAccess::broken(*joint, broken);
                    std::string key;
                    field(item, "stable_key", key);
                    staged.add_constraint(std::move(joint), key);
                }
            }
            for (const auto& item : awake)
                staged.find_body(item.first)->set_awake(item.second);
            world = std::move(staged);
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }
    bool parse_scenario_document(std::string_view text, ScenarioDocument& document, std::string& error)
    {
        try
        {
            ScenarioDocument result;
            if (!content::parse_json(text, result.root, error))
                return false;
            validate_envelope(result.root);
            result.metadata = decode_metadata(required(result.root, "metadata"));
            World validation;
            if (!populate_world(result, validation, error))
                return false;
            document = std::move(result);
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }
    bool write_scenario_document(const ScenarioDocument& document, std::string& text, std::string& error)
    {
        try
        {
            auto root = document.root;
            root["metadata"] = inherit(required(root, "metadata"), encode_metadata(document.metadata));
            ScenarioDocument candidate { root, document.metadata };
            World validation;
            if (!populate_world(candidate, validation, error))
                return false;
            return content::write_json(root, text, error);
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }
    bool capture_scenario_document(const World& world, const ScenarioMetadata& metadata, ScenarioDocument& document, std::string& error, const ScenarioDocument* source)
    {
        try
        {
            Json world_value = Object { { "settings", encode_settings(world.settings()) }, { "algorithms", encode_algorithms(world) }, { "potential_energy_reference_height_m", world.potential_energy_reference_height_m() } };
            BodyReferences references;
            std::set<std::string> identifiers;
            for (const auto id : world.body_ids())
                if (!ScenarioDocumentAccess::document_id(world, id).empty())
                    identifiers.insert(ScenarioDocumentAccess::document_id(world, id));
            for (const auto id : world.body_ids())
            {
                auto name = ScenarioDocumentAccess::document_id(world, id);
                if (name.empty())
                {
                    name = "body-" + std::to_string(id.index) + "-" + std::to_string(id.generation);
                    while (!identifiers.insert(name).second)
                        name += "-new";
                }
                references.emplace_back(id, std::move(name));
            }
            Array bodies, forces, springs, joints;
            for (const auto& item : references)
                bodies.push_back(encode_body(world, item.first, item.second));
            for (const auto& force : world.force_generators())
                forces.push_back(encode_force(*force));
            for (const auto id : world.spring_ids())
                springs.push_back(encode_spring(*world.spring_definition(id), references, ScenarioDocumentAccess::spring_key(world, id)));
            for (const auto& constraint : world.constraints())
            {
                const auto* joint = dynamic_cast<const JointConstraint*>(constraint.get());
                require(joint != nullptr, "Cannot save an unsupported custom constraint");
                joints.push_back(encode_joint(*joint, references, world.constraint_key(constraint)));
            }
            world_value["bodies"] = bodies;
            world_value["forces"] = forces;
            world_value["springs"] = springs;
            world_value["joints"] = joints;
            Json root = Object { { "format", "rigid-bodies.scenario" }, { "version", Object { { "major", 1 }, { "minor", 0 } } }, { "metadata", encode_metadata(metadata) }, { "world", world_value } };
            if (source)
            {
                validate_envelope(source->root);
                root = inherit(source->root, root);
                root["version"] = source->root.at("version");
            }
            ScenarioDocument result { std::move(root), metadata };
            World validation;
            if (!populate_world(result, validation, error))
                return false;
            document = std::move(result);
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }
}
