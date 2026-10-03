#include <rigidbodies/render/scene_renderer.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace rigidbodies::render
{
    namespace
    {
        template <class Pool>
        std::size_t bounded_budget(const Pool& pool, std::size_t budget)
        {
            return std::min(pool.size(), budget);
        }

        template <class Pool>
        void age_particles(Pool& pool, double dt, std::size_t budget, bool enabled, double gravity)
        {
            for (std::size_t index = 0; index < pool.size(); ++index)
            {
                auto& particle = pool[index];
                if (!enabled || index >= budget)
                    particle.lifetime_s = 0.0;
                if (particle.lifetime_s <= 0.0)
                    continue;
                particle.age_s += dt;
                particle.velocity.y -= gravity * dt;
                particle.position += particle.velocity * dt;
                if (particle.age_s >= particle.lifetime_s)
                    particle.lifetime_s = 0.0;
            }
        }

        template <class Pool, class Value>
        void spawn(Pool& pool, const Value& value, std::size_t budget)
        {
            for (std::size_t index = 0; index < bounded_budget(pool, budget); ++index)
                if (pool[index].lifetime_s <= 0.0)
                {
                    pool[index] = value;
                    return;
                }
            // Saturation drops the new particle, preserving a hard storage and draw-work ceiling.
        }

        double unit_random(std::uint32_t& seed)
        {
            seed = seed * 1664525u + 1013904223u;
            return static_cast<double>(seed >> 8u) / 16777216.0;
        }

        const physics::Material* first_material(const physics::RigidBody* body)
        {
            return body && !body->colliders().empty() ? &body->colliders().front().material : nullptr;
        }

        double soft_response(const physics::RigidBody* body)
        {
            if (!body || body->type() != physics::BodyType::dynamic_body)
                return 0.0;
            double result = 0.0;
            for (const auto& collider : body->colliders())
                result = std::max(result, static_cast<double>(material_appearance(collider.material).softness));
            return result;
        }

        float scene_scale(const SceneRenderSettings& settings)
        {
            return std::isfinite(settings.display_scale) ? std::clamp(settings.display_scale, 0.5f, 4.0f) : 1.0f;
        }

        float scene_stroke(const SceneRenderSettings& settings, float logical)
        {
            const auto weight = std::isfinite(settings.theme.stroke_weight) ? std::clamp(settings.theme.stroke_weight, 0.5f, 3.0f) : 1.0f;
            return logical * scene_scale(settings) * weight;
        }

        std::shared_ptr<const IndexedMesh> dust_mesh()
        {
            // One soft unit disc is shared by every puff and every frame. The hardware path
            // instances it; the software reference expands identical instance transforms.
            static const auto mesh = []
            {
                auto result = std::make_shared<IndexedMesh>();
                constexpr int segments = 24;
                result->vertices.push_back({ {}, {}, { 0.65f, 0.65f, 0.65f, 0.65f } });
                for (int index = 0; index < segments; ++index)
                {
                    const auto angle = math::two_pi * static_cast<double>(index) / segments;
                    const Vec2 unit { std::cos(angle), std::sin(angle) };
                    result->vertices.push_back({ unit * 0.4, {}, { 0.35f, 0.35f, 0.35f, 0.35f } });
                    result->vertices.push_back({ unit, {}, { 0.0f, 0.0f, 0.0f, 0.0f } });
                }
                for (int index = 0; index < segments; ++index)
                {
                    const auto inner = 1 + index * 2;
                    const auto next = 1 + ((index + 1) % segments) * 2;
                    result->indices.insert(result->indices.end(), { 0, inner, next, inner, inner + 1, next + 1, inner, next + 1, next });
                }
                return result;
            }();
            return mesh;
        }

        // Catmull-Rom resampling turns a dozen motion samples into a smooth ribbon spine.
        void smooth_path(const Vec2* points, std::size_t count, int subdivisions, std::vector<Vec2>& result)
        {
            result.clear();
            if (count < 2)
                return;
            for (std::size_t index = 0; index + 1 < count; ++index)
            {
                const auto& p0 = points[index == 0 ? 0 : index - 1];
                const auto& p1 = points[index];
                const auto& p2 = points[index + 1];
                const auto& p3 = points[index + 2 < count ? index + 2 : count - 1];
                for (int step = 0; step < subdivisions; ++step)
                {
                    const auto t = static_cast<double>(step) / subdivisions;
                    const auto t2 = t * t, t3 = t2 * t;
                    result.push_back((p1 * 2.0 + (p2 - p0) * t + (p0 * 2.0 - p1 * 5.0 + p2 * 4.0 - p3) * t2 + (p1 * 3.0 - p0 - p2 * 3.0 + p3) * t3) * 0.5);
                }
            }
            result.push_back(points[count - 1]);
        }
    }

    MaterialAppearance material_appearance(const physics::Material& material)
    {
        MaterialAppearance result;
        const auto density = std::isfinite(material.density_kg_m3) ? std::clamp(material.density_kg_m3 / 7850.0, 0.0, 1.0) : 0.5;
        result.surface = mix(Color::from_bytes(128, 160, 168), Color::from_bytes(104, 128, 158), static_cast<float>(density));
        result.shade = mix(result.surface, Color::from_bytes(30, 38, 48), 0.36f);
        result.highlight = mix(result.surface, Color::from_bytes(240, 244, 248), 0.3f);
        result.rim = mix(result.surface, Color::from_bytes(250, 252, 255), 0.55f);
        result.gloss = 0.14f;
        result.edge_light = 0.22f;
        if (material.name == "oak_wood")
        {
            result.surface = Color::from_bytes(186, 140, 92);
            result.shade = Color::from_bytes(140, 100, 62);
            result.highlight = Color::from_bytes(206, 164, 114);
            result.rim = Color::from_bytes(226, 194, 150);
            result.gloss = 0.12f;
            result.grain = 0.16f;
            result.edge_light = 0.2f;
        }
        else if (material.name == "steel")
        {
            result.surface = Color::from_bytes(136, 150, 168);
            result.shade = Color::from_bytes(86, 97, 113);
            result.highlight = Color::from_bytes(184, 197, 214);
            result.rim = Color::from_bytes(232, 239, 248);
            result.specular = 0.5f;
            result.gloss = 0.55f;
            result.edge_light = 0.45f;
        }
        else if (material.name == "aluminium")
        {
            result.surface = Color::from_bytes(174, 182, 192);
            result.shade = Color::from_bytes(122, 130, 142);
            result.highlight = Color::from_bytes(214, 220, 228);
            result.rim = Color::from_bytes(246, 248, 251);
            result.specular = 0.42f;
            result.gloss = 0.45f;
            result.edge_light = 0.4f;
        }
        else if (material.name == "rubber")
        {
            result.surface = Color::from_bytes(104, 76, 82);
            result.shade = Color::from_bytes(66, 48, 54);
            result.highlight = Color::from_bytes(132, 102, 108);
            result.rim = Color::from_bytes(170, 142, 148);
            result.gloss = 0.24f;
            result.edge_light = 0.14f;
        }
        else if (material.name == "glass")
        {
            result.surface = Color::from_bytes(140, 204, 226, 92);
            result.shade = Color::from_bytes(90, 156, 184, 112);
            result.highlight = Color::from_bytes(206, 238, 250, 140);
            result.rim = Color::from_bytes(232, 248, 255, 240);
            result.specular = 0.35f;
            result.gloss = 0.5f;
            result.edge_light = 0.85f;
            result.translucent = true;
        }
        else if (material.name == "expanded_polystyrene")
        {
            result.surface = Color::from_bytes(226, 223, 210);
            result.shade = Color::from_bytes(186, 182, 168);
            result.highlight = Color::from_bytes(240, 238, 230);
            result.rim = Color::from_bytes(250, 249, 244);
            result.gloss = 0.1f;
            result.edge_light = 0.12f;
        }
        result.roughness = std::isfinite(material.kinetic_friction) ? static_cast<float>(std::clamp(material.kinetic_friction, 0.08, 0.95)) : 0.5f;
        if (material.name == "steel" || material.name == "aluminium" || material.name == "glass")
            result.roughness *= 0.35f;
        if (material.name == "rubber")
            result.softness = static_cast<float>(std::clamp(material.restitution, 0.0, 1.0));
        else if (material.name == "expanded_polystyrene")
            result.softness = 0.5f;
        return result;
    }

    void SceneRenderer::clear_visual_effects()
    {
        motion_ = {};
        flashes_ = {};
        sparks_ = {};
        dust_ = {};
        deformations_ = {};
        last_visual_step_ = impact_bursts_ = 0;
        has_visual_sample_ = false;
        peak_applied_n_ = quiet_loads_s_ = rescaled_note_s_ = 0.0;
        key_corner_ = key_home_free_frames_ = 0;
        for (auto& history : contact_history_)
            history = {};
    }

    SceneEffectStatistics SceneRenderer::effect_statistics() const
    {
        SceneEffectStatistics result;
        for (const auto& sample : motion_)
            result.motion_bodies += sample.count > 0 ? 1u : 0u;
        const auto count = [](const auto& pool)
        {
            return static_cast<std::size_t>(std::count_if(pool.begin(), pool.end(), [](const auto& value)
                {
                    return value.lifetime_s > 0.0;
                }));
        };
        result.flashes = count(flashes_);
        result.sparks = count(sparks_);
        result.dust = count(dust_);
        result.deformations = count(deformations_);
        result.impact_bursts = impact_bursts_;
        return result;
    }

    void SceneRenderer::record_visual_sample(const physics::World& world, double time_step_s, const SceneRenderSettings& settings)
    {
        if (!std::isfinite(time_step_s) || time_step_s <= 0.0)
            return;
        contact_step_s_ = time_step_s;
        const auto step = world.statistics().step_index;
        if (has_visual_sample_ && step < last_visual_step_)
            clear_visual_effects();
        if (has_visual_sample_ && step == last_visual_step_)
            return;
        has_visual_sample_ = true;
        last_visual_step_ = step;
        {
            // Accumulated point impulses are what each solved substep applied. A sleeping body is
            // not solved, so its stale impulses are not recorded.
            std::fill(contact_scratch_n_.begin(), contact_scratch_n_.end(), Vec2 {});
            const auto add = [&](physics::BodyId id, const Vec2& force)
            {
                if (id.index >= contact_scratch_n_.size())
                    contact_scratch_n_.resize(static_cast<std::size_t>(id.index) + 1);
                contact_scratch_n_[id.index] += force;
            };
            for (const auto& manifold : world.manifolds())
            {
                if (manifold.is_sensor || manifold.point_count == 0)
                    continue;
                const auto length = std::hypot(manifold.normal.x, manifold.normal.y);
                if (!std::isfinite(length) || length <= math::geometric_epsilon)
                    continue;
                const auto normal = manifold.normal / length;
                double normal_impulse = 0.0, tangent_impulse = 0.0;
                for (std::size_t index = 0; index < std::min(manifold.point_count, physics::maximum_manifold_points); ++index)
                {
                    normal_impulse += manifold.points[index].normal_impulse_n_s;
                    tangent_impulse += manifold.points[index].tangent_impulse_n_s;
                }
                const auto force = (normal * normal_impulse + math::perpendicular(normal) * tangent_impulse) / time_step_s;
                if (!math::is_finite(force))
                    continue;
                add(manifold.second, force);
                add(manifold.first, -force);
            }
            double largest_load_n = 0.0;
            world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                {
                    if (body.type() == physics::BodyType::static_body)
                        return;
                    if (body.type() == physics::BodyType::dynamic_body)
                        if (const auto load = std::hypot(body.applied_force_n().x, body.applied_force_n().y); std::isfinite(load))
                            largest_load_n = std::max(largest_load_n, load);
                    if (!body.is_awake())
                    {
                        // Waking starts a fresh average rather than one mixed with older contacts.
                        if (id.index < contact_history_.size())
                            contact_history_[id.index].count = 0;
                        return;
                    }
                    if (id.index >= contact_history_.size())
                        contact_history_.resize(static_cast<std::size_t>(id.index) + 1);
                    auto& history = contact_history_[id.index];
                    if (history.generation != id.generation)
                        history = { id.generation };
                    history.force_n[history.next] = id.index < contact_scratch_n_.size() ? contact_scratch_n_[id.index] : Vec2 {};
                    history.step_s[history.next] = time_step_s;
                    history.next = (history.next + 1) % history.force_n.size();
                    history.count = std::min(history.count + 1, history.force_n.size());
                });
            // A scene without gravity scales its forces to the largest load so far. Once every
            // load has stayed below a sixth of that for a second, which leaves every arrow a
            // stub, the scale drops to the present largest load in one announced step.
            rescaled_note_s_ = std::max(0.0, rescaled_note_s_ - time_step_s);
            if (peak_applied_n_ > 0.0 && largest_load_n < peak_applied_n_ / 6.0)
            {
                quiet_loads_s_ += time_step_s;
                if (quiet_loads_s_ >= 1.0 && largest_load_n > 1.0e-5)
                {
                    peak_applied_n_ = largest_load_n;
                    quiet_loads_s_ = 0.0;
                    rescaled_note_s_ = 2.0;
                }
            }
            else
                quiet_loads_s_ = 0.0;
        }
        // A user-supplied giant step cannot send decorative particles arbitrarily far away.
        const auto dt = std::min(time_step_s, 0.1);
        age_particles(flashes_, dt, settings.impact_flash_budget, settings.impact_flashes, 0.0);
        age_particles(sparks_, dt, settings.spark_budget, settings.impact_sparks, 3.0);
        age_particles(dust_, dt, settings.dust_budget, settings.impact_dust, -0.05);
        for (std::size_t index = 0; index < deformations_.size(); ++index)
        {
            auto& deformation = deformations_[index];
            deformation.age_s += dt;
            if (!settings.soft_deformation || index >= settings.deformation_budget || deformation.age_s >= deformation.lifetime_s || !world.is_valid(deformation.body))
                deformation.lifetime_s = 0.0;
        }
        for (std::size_t index = 0; index < motion_.size(); ++index)
        {
            auto& sample = motion_[index];
            sample.age_s += dt;
            if (!settings.motion_trails || index >= settings.motion_body_budget || !world.is_valid(sample.body) || sample.age_s > 0.24)
                sample = {};
        }
        if (settings.motion_trails)
            world.for_each_body([&](physics::BodyId id, const physics::RigidBody& body)
                {
                    if (body.type() == physics::BodyType::static_body || math::length_squared(body.linear_velocity_m_s()) < 0.04)
                        return;
                    MotionSample* slot = nullptr;
                    const auto budget = bounded_budget(motion_, settings.motion_body_budget);
                    for (std::size_t index = 0; index < budget; ++index)
                        if (motion_[index].body == id && motion_[index].count != 0)
                            slot = &motion_[index];
                    if (!slot)
                        for (std::size_t index = 0; index < budget; ++index)
                            if (motion_[index].count == 0)
                            {
                                slot = &motion_[index];
                                slot->body = id;
                                break;
                            }
                    if (!slot)
                        return;
                    // Uniform simulation sampling at up to 60 Hz bounds history independent of the
                    // world's substep count, and retains visible persistence at 120/240 Hz.
                    if (slot->count > 0 && slot->age_s < 1.0 / 60.0)
                        return;
                    if (slot->count == slot->points.size())
                    {
                        std::move(slot->points.begin() + 1, slot->points.end(), slot->points.begin());
                        --slot->count;
                    }
                    slot->points[slot->count++] = body.world_center_of_mass_m();
                    slot->age_s = 0.0;
                });

        for (const auto& impact : world.impact_reports())
        {
            const auto impulse = std::abs(impact.normal_impulse_n_s);
            const auto incoming = -math::dot(impact.second_before.velocity_m_s - impact.first_before.velocity_m_s, impact.normal);
            // Resting support impulses do not glitter. The reports already represent collision
            // episodes; these thresholds additionally suppress tiny, barely moving touches.
            if (!std::isfinite(impulse) || impulse < 0.02 || incoming < 0.15 || !math::is_finite(impact.point_m))
                continue;
            ++impact_bursts_;
            const auto strength = std::clamp(std::log1p(impulse) / 3.0, 0.04, 1.0);
            std::uint32_t seed = static_cast<std::uint32_t>(step) ^ (impact.first_before.id.index * 73856093u) ^ (impact.second_before.id.index * 19349663u);
            const auto* first = world.find_body(impact.first_before.id);
            const auto* second = world.find_body(impact.second_before.id);
            const auto second_moves = second && second->type() == physics::BodyType::dynamic_body;
            const auto* material = first_material(second_moves ? second : first);
            const auto tint = material ? material_appearance(*material).surface.with_alpha(1.0f) : settings.theme.contact;
            // Debris leaves the struck surface toward the moving body, mostly along the surface.
            const auto normal = math::normalized(impact.normal) * (second_moves ? 1.0 : -1.0);
            const auto tangent = math::perpendicular(normal);
            const auto light_stage = settings.theme.light_stage;
            const auto streak = light_stage ? mix(tint, Color::from_bytes(20, 24, 30), 0.35f) : mix(tint, Color::from_bytes(255, 255, 255), 0.45f);
            if (settings.impact_flashes)
                spawn(flashes_, Particle { impact.point_m, {}, mix(settings.theme.label_text, tint, 0.3f), 0.0, 0.16 + strength * 0.14, strength }, settings.impact_flash_budget);
            const auto particles = static_cast<std::size_t>(1.0 + strength * 5.0);
            for (std::size_t index = 0; index < particles; ++index)
            {
                const auto side = index % 2 == 0 ? 1.0 : -1.0;
                const auto rise = 0.25 + unit_random(seed) * 0.75;
                const auto speed = (0.55 + unit_random(seed) * 0.6) * (0.5 + strength * 2.2);
                const auto velocity = math::normalized(tangent * side + normal * rise) * speed;
                if (settings.impact_sparks)
                    spawn(sparks_, Particle { impact.point_m, velocity, streak, 0.0, 0.12 + unit_random(seed) * 0.12, strength }, settings.spark_budget);
                if (settings.impact_dust && index % 2 == 0)
                    spawn(dust_, Particle { impact.point_m, tangent * (side * speed * 0.18) + normal * 0.04, mix(tint, settings.theme.static_body_fill, 0.5f), 0.0, 0.32 + unit_random(seed) * 0.24, strength }, settings.dust_budget);
            }
            if (settings.soft_deformation)
                for (const auto id : { impact.first_before.id, impact.second_before.id })
                {
                    const auto* body = world.find_body(id);
                    const auto softness = soft_response(body);
                    if (softness <= 0.0)
                        continue;
                    // Softness is a visual cue, never a collider edit. Compression is limited to
                    // 12%, with reciprocal tangential expansion so drawn area stays unchanged.
                    for (auto& value : deformations_)
                        if (value.body == id)
                            value.lifetime_s = 0.0;
                    spawn(deformations_, Deformation { id, math::normalized(impact.normal), 0.0, 0.16 + softness * 0.12, std::min(0.12, strength * softness * 0.16) }, settings.deformation_budget);
                }
        }
    }

    Vec2 SceneRenderer::deform_point(physics::BodyId id, const Vec2& point, const Vec2& center, bool enabled, std::size_t budget) const
    {
        if (enabled)
            for (std::size_t index = 0; index < std::min(budget, deformations_.size()); ++index)
                if (const auto& deformation = deformations_[index]; deformation.body == id && deformation.lifetime_s > 0.0)
                {
                    const auto fraction = deformation.age_s / deformation.lifetime_s;
                    const auto decay = (1.0 - fraction) * (1.0 - fraction);
                    const auto compression = 1.0 - deformation.strength * decay;
                    const auto delta = point - center;
                    const auto along = deformation.normal * math::dot(delta, deformation.normal);
                    return center + along * compression + (delta - along) / compression;
                }
        return point;
    }

    void SceneRenderer::draw_visual_motion(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        const auto scale = scene_scale(settings);
        if (settings.motion_trails)
        {
            thread_local std::vector<Vec2> spine;
            std::array<Vec2, 13> points {};
            for (std::size_t index = 0; index < bounded_budget(motion_, settings.motion_body_budget); ++index)
            {
                const auto& sample = motion_[index];
                const auto* body = world.find_body(sample.body);
                if (!body || sample.count < 2)
                    continue;
                const auto speed = math::length(body->linear_velocity_m_s());
                const auto weight = static_cast<float>(std::clamp((speed - 0.4) / 4.0, 0.0, 1.0));
                const auto fade = static_cast<float>(std::clamp(1.0 - sample.age_s / 0.24, 0.0, 1.0));
                if (weight * fade <= 0.01f)
                    continue;
                // The ribbon ends at the drawn centre so it never detaches from the body.
                std::size_t count = 0;
                for (std::size_t point = 0; point < sample.count; ++point)
                    points[count++] = camera.world_to_screen(sample.points[point]);
                const auto drawn = camera.world_to_screen(math::transform_point(body->interpolated_transform(settings.interpolation_alpha), body->mass_properties().center_of_mass_m));
                points[count++] = drawn;
                smooth_path(points.data(), count, 4, spine);
                const auto bounds = body->compute_bounds();
                const auto radius = camera.world_to_screen_length(std::min(bounds.extents().x, bounds.extents().y) * 0.5);
                // A ribbon that would barely clear the body reads as a stub, not as motion.
                double travelled = 0.0;
                for (std::size_t point = 1; point < spine.size(); ++point)
                    travelled += math::length(spine[point] - spine[point - 1]);
                if (travelled < radius * 2.0 + 12.0 * scale)
                    continue;
                const auto width = static_cast<float>(std::clamp(radius * 0.6, 2.0 * scale, 10.0 * scale));
                const auto color = settings.theme.trajectory.with_alpha(settings.theme.trajectory.alpha * 0.5f * weight * fade);
                list.add_tapered_polyline(spine, color.with_alpha(0.0f), color, 0.0f, width);
            }
        }
        if (settings.directional_blur)
        {
            std::size_t count = 0;
            world.for_each_body([&](physics::BodyId, const physics::RigidBody& body)
                {
                    if (count >= std::min<std::size_t>(settings.directional_blur_budget, 128) || body.type() == physics::BodyType::static_body)
                        return;
                    const auto velocity = camera.world_to_screen_direction(body.linear_velocity_m_s());
                    const auto speed = math::length(body.linear_velocity_m_s());
                    if (speed < 1.5)
                        return;
                    ++count;
                    const auto center = camera.world_to_screen(math::transform_point(body.interpolated_transform(settings.interpolation_alpha), body.mass_properties().center_of_mass_m));
                    const auto bounds = body.compute_bounds();
                    const auto radius = std::clamp(camera.world_to_screen_length(std::min(bounds.extents().x, bounds.extents().y) * 0.32), 2.0, 30.0 * scale);
                    const auto length = std::min(camera.world_to_screen_length(speed / 80.0), 70.0 * scale);
                    const auto end = center - math::normalized(velocity) * length;
                    const auto* material = first_material(&body);
                    const auto color = material ? material_appearance(*material).surface.with_alpha(1.0f) : settings.theme.body_fill;
                    const auto strength = static_cast<float>(std::clamp((speed - 1.5) / 6.0, 0.0, 1.0));
                    for (int band = 3; band >= 1; --band)
                        list.add_line(center, end, color.with_alpha(0.02f * static_cast<float>(band) * strength), static_cast<float>(radius * 2.0 + static_cast<double>(band) * 2.0 * scale));
                });
        }
    }

    void SceneRenderer::draw_visual_impacts(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        const auto scale = scene_scale(settings);
        if (settings.impact_dust)
        {
            std::vector<MeshInstance> instances;
            instances.reserve(bounded_budget(dust_, settings.dust_budget));
            for (std::size_t index = 0; index < bounded_budget(dust_, settings.dust_budget); ++index)
            {
                const auto& particle = dust_[index];
                if (particle.lifetime_s <= 0.0)
                    continue;
                const auto fraction = particle.age_s / particle.lifetime_s;
                const auto radius = static_cast<float>((3.0 + particle.strength * 7.0) * (0.5 + fraction * 0.9) * scale);
                const auto center = camera.world_to_screen(particle.position);
                const auto color = particle.color.with_alpha(static_cast<float>(0.24 * (1.0 - fraction) * (1.0 - fraction)));
                instances.push_back({ center, { radius, radius * 0.7f }, 0.0f, color });
            }
            if (!instances.empty())
                list.add_instanced_mesh(dust_mesh(), std::move(instances));
        }
        if (settings.impact_sparks)
            for (std::size_t index = 0; index < bounded_budget(sparks_, settings.spark_budget); ++index)
            {
                const auto& particle = sparks_[index];
                if (particle.lifetime_s <= 0.0)
                    continue;
                // Short streaks: length follows speed, width and opacity ease out with age.
                const auto fraction = particle.age_s / particle.lifetime_s;
                const auto fade = static_cast<float>((1.0 - fraction) * (1.0 - fraction));
                const auto head = camera.world_to_screen(particle.position);
                const auto tail = camera.world_to_screen(particle.position - particle.velocity * 0.03);
                list.add_line(tail, head, particle.color.with_alpha(fade * 0.75f), scene_stroke(settings, 1.25f) * static_cast<float>(1.0 - 0.4 * fraction));
            }
        if (settings.impact_flashes)
            for (std::size_t index = 0; index < bounded_budget(flashes_, settings.impact_flash_budget); ++index)
            {
                const auto& particle = flashes_[index];
                if (particle.lifetime_s <= 0.0)
                    continue;
                // A single expanding ring with an ease-out radius: a pulse, not a burst.
                const auto fraction = particle.age_s / particle.lifetime_s;
                const auto eased = 1.0 - (1.0 - fraction) * (1.0 - fraction) * (1.0 - fraction);
                const auto center = camera.world_to_screen(particle.position);
                const auto radius = static_cast<float>((3.0 + (6.0 + particle.strength * 16.0) * eased) * scale);
                const auto alpha = static_cast<float>((1.0 - fraction) * (1.0 - fraction) * (0.35 + 0.35 * particle.strength));
                list.add_circle_outline(center, radius, particle.color.with_alpha(alpha), scene_stroke(settings, 1.5f) * static_cast<float>(1.0 - 0.5 * fraction));
            }
    }

    void SceneRenderer::draw_contact_shadows(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const
    {
        if (!settings.contact_shadows)
            return;
        const auto scale = scene_scale(settings);
        const auto previous = list.layer();
        // Above fixed ground, below moving bodies: occlusion darkens the support surface and the
        // crevice beside the contact, then the resting body covers its own share.
        list.set_layer(-6);
        std::size_t count = 0;
        std::shared_ptr<IndexedMesh> mesh;
        for (const auto& manifold : world.manifolds())
        {
            if (manifold.is_sensor || manifold.point_count == 0)
                continue;
            if (count >= std::min<std::size_t>(settings.contact_shadow_budget, 192))
                break;
            const auto* first = world.find_body(manifold.first);
            const auto* second = world.find_body(manifold.second);
            if (!first || !second)
                continue;
            // The occluder is the body that can rest on the other.
            const auto* resting = second->type() == physics::BodyType::dynamic_body ? second : first;
            Vec2 center_m;
            double separation_m = 1.0e300;
            for (std::size_t index = 0; index < manifold.point_count; ++index)
            {
                center_m += manifold.points[index].world_position_m;
                separation_m = std::min(separation_m, manifold.points[index].separation_m);
            }
            center_m = center_m / static_cast<double>(manifold.point_count);
            const auto separation_px = camera.world_to_screen_length(std::max(0.0, separation_m));
            const auto approach = manifold.is_speculative ? 1.0 - std::clamp(separation_px / (14.0 * scale), 0.0, 1.0) : 1.0;
            if (approach <= 0.02 || !math::is_finite(center_m))
                continue;
            ++count;
            const auto normal = math::normalized(camera.world_to_screen_direction(manifold.normal));
            const auto tangent = math::perpendicular(normal);
            double spread = 0.0;
            for (std::size_t index = 0; index < manifold.point_count; ++index)
                spread = std::max(spread, std::abs(math::dot(camera.world_to_screen(manifold.points[index].world_position_m) - camera.world_to_screen(center_m), tangent)));
            const auto bounds = resting->compute_bounds();
            const auto size = camera.world_to_screen_length(std::min(bounds.extents().x, bounds.extents().y) * 0.5);
            const auto half_length = std::max(spread + 8.0 * scale, std::min(size * 0.7, 48.0 * scale));
            const auto half_height = 5.0 * scale;
            const auto center = camera.world_to_screen(center_m);
            if (!mesh)
                mesh = acquire_mesh();
            // A soft elliptical occlusion: concentric rings whose opacity falls smoothly to zero,
            // so the blob has no edge or corner for antialiasing to sharpen.
            constexpr int segments = 24, rings = 4;
            const auto shade = settings.theme.shadow.with_alpha(settings.theme.shadow.alpha * 0.85f * static_cast<float>(approach));
            const auto premultiplied = [](const Color& color)
            {
                return Color { color.red * color.alpha, color.green * color.alpha, color.blue * color.alpha, color.alpha };
            };
            const auto base = static_cast<int>(mesh->vertices.size());
            mesh->vertices.push_back({ center, {}, premultiplied(shade) });
            for (int ring = 1; ring <= rings; ++ring)
            {
                const auto fraction = static_cast<double>(ring) / rings;
                const auto falloff = (1.0 - fraction * fraction) * (1.0 - fraction * fraction);
                const auto color = premultiplied(shade.with_alpha(shade.alpha * static_cast<float>(falloff)));
                for (int index = 0; index < segments; ++index)
                {
                    const auto angle = math::two_pi * static_cast<double>(index) / segments;
                    mesh->vertices.push_back({ center + tangent * (std::cos(angle) * half_length * fraction) + normal * (std::sin(angle) * half_height * fraction), {}, color });
                }
            }
            for (int index = 0; index < segments; ++index)
            {
                const auto next = (index + 1) % segments;
                mesh->indices.insert(mesh->indices.end(), { base, base + 1 + index, base + 1 + next });
                for (int ring = 1; ring < rings; ++ring)
                {
                    const auto inner = base + 1 + (ring - 1) * segments, outer = inner + segments;
                    mesh->indices.insert(mesh->indices.end(), { inner + index, outer + index, outer + next, inner + index, outer + next, inner + next });
                }
            }
        }
        if (mesh && !mesh->indices.empty())
            list.add_indexed_mesh(mesh);
        list.set_layer(previous);
    }
}
