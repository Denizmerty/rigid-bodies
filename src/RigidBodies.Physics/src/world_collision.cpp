#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/physics/continuous_collision.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

namespace rigidbodies::physics
{
    namespace
    {
        bool same_pair(const BroadPhasePair& a, const BroadPhasePair& b)
        {
            return a.first == b.first && a.second == b.second &&
                a.first_collider == b.first_collider && a.second_collider == b.second_collider;
        }

        BroadPhasePair pair_of(const ContactManifold& manifold)
        {
            return { manifold.first, manifold.second, manifold.first_collider, manifold.second_collider };
        }

        struct CollisionIdentity
        {
            BroadPhasePair pair;
            bool sensor {};

            bool operator==(const CollisionIdentity& other) const
            {
                return same_pair(pair, other.pair) && sensor == other.sensor;
            }
        };

        struct CollisionIdentityHash
        {
            std::size_t operator()(const CollisionIdentity& value) const
            {
                std::size_t hash = 0;
                const auto append = [&](std::size_t part)
                {
                    hash ^= part + 0x9e3779b9U + (hash << 6) + (hash >> 2);
                };
                append(value.pair.first.index);
                append(value.pair.first.generation);
                append(value.pair.second.index);
                append(value.pair.second.generation);
                append(value.pair.first_collider);
                append(value.pair.second_collider);
                append(value.sensor ? 1U : 0U);
                return hash;
            }
        };

        void track_events(std::vector<CollisionEvent>& active, std::vector<CollisionEvent> current,
            std::vector<CollisionEvent>& events, std::vector<CollisionEvent>& pending)
        {
            events = std::move(pending);
            pending.clear();
            // Membership uses a hash index; observable order still comes solely from the
            // canonical vectors. This removes quadratic work in dense sensor/contact scenes.
            std::unordered_set<CollisionIdentity, CollisionIdentityHash> old_index;
            std::unordered_set<CollisionIdentity, CollisionIdentityHash> new_index;
            old_index.reserve(active.size());
            new_index.reserve(current.size());
            for (const auto& event : active)
                old_index.insert({ event.pair, event.is_sensor });
            for (const auto& event : current)
                new_index.insert({ event.pair, event.is_sensor });
            for (const auto& previous : active)
            {
                if (new_index.find({ previous.pair, previous.is_sensor }) == new_index.end())
                {
                    auto ended = previous;
                    ended.kind = CollisionEventKind::end;
                    events.push_back(ended);
                }
            }
            for (const auto& event : current)
            {
                if (old_index.find({ event.pair, event.is_sensor }) == old_index.end())
                    events.push_back(event);
            }
            active = std::move(current);
        }

        math::Transform2 body_placement(const IntegratedMotion& motion, const math::Vec2& local_center)
        {
            return math::Transform2::from_angle(motion.center_position_m - math::rotate(math::Rotation2 { motion.orientation_rad }, local_center), motion.orientation_rad);
        }
    }

    ShapeSweep World::make_sweep(BodyId id, const Collider& collider, Real time_step_s, const IntegratedMotion& target, const IntegratedMotion* start) const
    {
        ShapeSweep sweep;
        sweep.shape = collider.shape.get();
        // Leave room for finite broad-phase inflation. An unrepresentable travel bound is
        // explicitly unresolved; conservative advancement must stop instead of trusting a
        // saturated number as though it were a valid Lipschitz bound.
        constexpr auto maximum_bound = std::numeric_limits<Real>::max() / 64.0;
        const auto bounded_sum = [&](Real first, Real second)
        {
            if (!math::is_finite(first) || !math::is_finite(second) || first > maximum_bound - second)
            {
                sweep.conservative_stop = true;
                return maximum_bound;
            }
            return first + second;
        };
        const auto bounded_product = [&](std::initializer_list<Real> factors)
        {
            // Zero-amplitude/radius paths remain still even for a huge finite frequency or
            // duration. Check zero before multiplying, and combine exponents to avoid an
            // overflowing intermediate when the complete product remains representable.
            if (std::any_of(factors.begin(), factors.end(), [](Real value)
                    {
                        return value == 0.0;
                    }))
                return 0.0;
            Real mantissa = 1.0;
            int exponent = 0;
            for (const auto factor : factors)
            {
                if (!math::is_finite(factor))
                {
                    sweep.conservative_stop = true;
                    return maximum_bound;
                }
                int part_exponent = 0;
                mantissa *= std::frexp(factor, &part_exponent);
                exponent += part_exponent;
            }
            int normalization = 0;
            mantissa = std::frexp(mantissa, &normalization);
            exponent += normalization;
            int maximum_exponent = 0;
            const auto maximum_mantissa = std::frexp(maximum_bound, &maximum_exponent);
            if (exponent > maximum_exponent || (exponent == maximum_exponent && mantissa > maximum_mantissa))
            {
                sweep.conservative_stop = true;
                return maximum_bound;
            }
            return std::ldexp(mantissa, exponent);
        };
        const auto& body = *find_body(id);
        const auto start_center = start ? start->center_position_m : body.world_center_of_mass_m();
        const auto start_angle = start ? start->orientation_rad : body.orientation_rad();
        const auto local_center = body.mass_properties().center_of_mass_m;
        const auto local_transform = collider.local_transform;
        const auto offset = local_transform.translation - local_center;
        const auto radius = [&]()
        {
            if (const auto* circle = dynamic_cast<const CircleShape*>(collider.shape.get()))
            {
                // A circle's boundary is invariant under spin about its own centre. Only the
                // centre's orbit about a compound body's COM contributes geometric sweep travel;
                // material surface velocity remains available to the ordinary friction solver.
                const auto orbit = math::transform_point(local_transform, circle->local_center_m()) - local_center;
                return bounded_sum(std::hypot(orbit.x, orbit.y), 0.0);
            }
            return bounded_sum(collider.shape->bounding_radius(), std::hypot(offset.x, offset.y));
        }();
        const auto& slot = slots_[id.index];
        const auto path = body.type() == BodyType::kinematic_body ? slot.motion : std::optional<KinematicMotion> {};
        const auto start_time = statistics_.elapsed_time_s - slot.motion_start_time_s;
        sweep.linear_displacement_m = target.center_position_m - start_center;
        if (!math::is_finite(sweep.linear_displacement_m))
        {
            sweep.conservative_stop = true;
            sweep.linear_displacement_m = {};
        }
        sweep.nonlinear_travel_m = bounded_product({ std::abs(target.orientation_rad - start_angle), radius });
        if (path)
        {
            std::visit([&](const auto& definition)
                {
                    using Path = std::decay_t<decltype(definition)>;
                    if constexpr (std::is_same_v<Path, HarmonicMotion>)
                    {
                        sweep.linear_displacement_m = {};
                        const auto translation = bounded_product({ math::two_pi, definition.frequency_hz, time_step_s, std::hypot(definition.translation_amplitude_m.x, definition.translation_amplitude_m.y) });
                        const auto rotation = bounded_product({ math::two_pi, definition.frequency_hz, time_step_s, std::abs(definition.rotation_amplitude_rad), radius });
                        sweep.nonlinear_travel_m = bounded_sum(translation, rotation);
                        // Endpoint samples can be finite while an intermediate periodic peak
                        // overflows. Such an interval is unresolved rather than sampled unsafely.
                        (void)bounded_sum(std::abs(definition.origin_m.x), std::abs(definition.translation_amplitude_m.x));
                        (void)bounded_sum(std::abs(definition.origin_m.y), std::abs(definition.translation_amplitude_m.y));
                        (void)bounded_sum(std::abs(definition.initial_orientation_rad), std::abs(definition.rotation_amplitude_rad));
                    }
                    else if constexpr (std::is_same_v<Path, CircularMotion>)
                    {
                        sweep.linear_displacement_m = {};
                        const auto translation = bounded_product({ std::abs(definition.angular_speed_rad_s), time_step_s, definition.radius_m });
                        const auto rotation = definition.orient_to_path
                            ? bounded_product({ std::abs(definition.angular_speed_rad_s), time_step_s, radius })
                            : 0.0;
                        sweep.nonlinear_travel_m = bounded_sum(translation, rotation);
                        (void)bounded_sum(std::abs(definition.center_m.x), definition.radius_m);
                        (void)bounded_sum(std::abs(definition.center_m.y), definition.radius_m);
                    }
                },
                path->definition());
        }
        sweep.transform_at_fraction = [=](Real fraction)
        {
            IntegratedMotion state;
            state.center_position_m = math::lerp(start_center, target.center_position_m, fraction);
            state.orientation_rad = start_angle + (target.orientation_rad - start_angle) * fraction;
            if (path && fraction > 0.0 && fraction < 1.0)
            {
                const auto sampled = path->sample(start_time + fraction * time_step_s);
                state.center_position_m = sampled.position_m;
                state.orientation_rad = sampled.orientation_rad;
            }
            return math::concatenate(body_placement(state, local_center), local_transform);
        };
        return sweep;
    }

    void World::build_proxies(Real time_step_s, const std::vector<IntegratedMotion>& targets)
    {
        proxies_.clear();
        for_each_body([&](BodyId id, const RigidBody& body)
            {
                for (std::size_t index = 0; index < body.colliders().size(); ++index)
                {
                    const auto& collider = body.colliders()[index];
                    if (!collider.shape)
                        continue;
                    BroadPhaseProxy proxy;
                    proxy.body = id;
                    proxy.collider_index = index;
                    proxy.bounds = collider.compute_bounds(body.transform());
                    proxy.displacement_m = targets[id.index].center_position_m - body.world_center_of_mass_m();
                    if (settings_.collision.continuous)
                    {
                        const auto sweep = make_sweep(id, collider, time_step_s, targets[id.index]);
                        proxy.bounds.expand(collider.shape->compute_bounds(sweep.transform_at_fraction(1.0)));
                        // The point travel bound covers curved paths and rotational sweeps even
                        // when the endpoints coincide after a complete revolution.
                        proxy.bounds.grow(sweep.nonlinear_travel_m);
                    }
                    proxy.bounds.grow(std::max(settings_.broad_phase_margin_m, settings_.collision.contact_margin_m));
                    proxy.filter = collider.filter;
                    proxy.body_type = body.type();
                    proxy.is_sensor = collider.is_sensor;
                    proxies_.push_back(proxy);
                }
            });
    }

    void World::detect_collisions(Real time_step_s, const std::vector<IntegratedMotion>& targets)
    {
        {
            const ScopedProfileTimer timer(profiling_enabled_, profile_.broad_phase_s);
            build_proxies(time_step_s, targets);
            pairs_.clear();
            if (const auto* tree = dynamic_cast<const DynamicTreeBroadPhase*>(broad_phase_.get()))
                profile_.broad_phase_workers = tree->find_pairs_parallel(proxies_, pairs_, parallel_executor(), parallel_settings_.minimum_batch_size);
            else if (const auto* brute = dynamic_cast<const BruteForceBroadPhase*>(broad_phase_.get()))
                profile_.broad_phase_workers = brute->find_pairs_parallel(proxies_, pairs_, parallel_executor(), parallel_settings_.minimum_batch_size);
            else
            {
                // Custom implementations may own mutable caches or callbacks even through const.
                broad_phase_->find_pairs(proxies_, pairs_);
                profile_.broad_phase_workers = proxies_.empty() ? 0 : 1;
            }
            pairs_.erase(std::remove_if(pairs_.begin(), pairs_.end(), [this](const BroadPhasePair& pair)
                             {
                                 if (!is_valid(pair.first) || !is_valid(pair.second) || pair.first == pair.second || !bodies_can_collide(pair.first, pair.second))
                                     return true;
                                 const auto& first = *find_body(pair.first);
                                 const auto& second = *find_body(pair.second);
                                 if (pair.first_collider >= first.colliders().size() || pair.second_collider >= second.colliders().size())
                                     return true;
                                 const auto& a = first.colliders()[pair.first_collider];
                                 const auto& b = second.colliders()[pair.second_collider];
                                 return !a.shape || !b.shape || !should_collide(a.filter, b.filter) ||
                                     (first.type() != BodyType::dynamic_body && second.type() != BodyType::dynamic_body && !a.is_sensor && !b.is_sensor);
                             }),
                pairs_.end());
            std::vector<std::size_t> ranks(slots_.size());
            for (std::size_t rank = 0; rank < ordered_slots_.size(); ++rank)
                ranks[ordered_slots_[rank]] = rank;
            for (auto& pair : pairs_)
            {
                if (ranks[pair.second.index] < ranks[pair.first.index])
                {
                    std::swap(pair.first, pair.second);
                    std::swap(pair.first_collider, pair.second_collider);
                }
            }
            const auto key = [&](const BroadPhasePair& pair)
            {
                return std::make_tuple(ranks[pair.first.index], ranks[pair.second.index], pair.first_collider, pair.second_collider);
            };
            std::sort(pairs_.begin(), pairs_.end(), [&](const auto& a, const auto& b)
                {
                    return key(a) < key(b);
                });
            pairs_.erase(std::unique(pairs_.begin(), pairs_.end(), same_pair), pairs_.end());
        }

        const ScopedProfileTimer timer(profiling_enabled_, profile_.narrow_phase_s);
        auto previous = std::move(manifolds_);
        const auto previous_index = [&]
        {
            std::unordered_map<CollisionIdentity, const ContactManifold*, CollisionIdentityHash> index;
            index.reserve(previous.size());
            for (const auto& manifold : previous)
                if (!manifold.is_sensor)
                    index.emplace(CollisionIdentity { pair_of(manifold), false }, &manifold);
            return index;
        }();
        manifolds_.clear();
        std::vector<CollisionEvent> current_pairs;
        std::vector<CollisionEvent> current_contacts;
        statistics_.persistent_contact_point_count = 0;
        statistics_.speculative_manifold_count = 0;
        statistics_.sweep_iteration_limit_count = 0;
        statistics_.ccd_clamped_body_count = 0;
        const auto impulse_scale = previous_contact_step_s_ > 0.0 ? std::min(time_step_s / previous_contact_step_s_, 4.0) : 0.0;
        const auto match_distance_squared = settings_.collision.matching_tolerance_m * settings_.collision.matching_tolerance_m;
        struct PairResult
        {
            ContactManifold manifold;
            std::size_t persistent_points {};
            std::size_t sweep_limits {};
            bool is_sensor { false };
            bool collided { false };
            bool touching { false };
        };
        std::vector<PairResult> results(pairs_.size());
        const auto process_pair = [&](std::size_t pair_index)
        {
            const auto& pair = pairs_[pair_index];
            auto& result = results[pair_index];
            const auto& first = *find_body(pair.first);
            const auto& second = *find_body(pair.second);
            const auto& a = first.colliders()[pair.first_collider];
            const auto& b = second.colliders()[pair.second_collider];
            NarrowPhaseQuery query;
            query.first = pair.first;
            query.second = pair.second;
            query.first_collider = pair.first_collider;
            query.second_collider = pair.second_collider;
            query.first_shape = a.shape.get();
            query.second_shape = b.shape.get();
            query.first_transform = math::concatenate(first.transform(), a.local_transform);
            query.second_transform = math::concatenate(second.transform(), b.local_transform);
            query.material = combine_materials(a.material, b.material, settings_.collision.friction_mixing, settings_.collision.restitution_mixing);
            query.is_sensor = a.is_sensor || b.is_sensor;
            query.contact_margin_m = query.is_sensor ? 0.0 : settings_.collision.contact_margin_m;
            result.is_sensor = query.is_sensor;
            auto& manifold = result.manifold;
            bool collided = narrow_phase_->collide(query, manifold) && !manifold.is_empty();
            manifold.point_count = std::min(manifold.point_count, maximum_manifold_points);
            bool swept_contact = false;

            const auto only_positive_points = collided && std::all_of(manifold.points.begin(), manifold.points.begin() + manifold.point_count, [](const ContactPoint& point)
                                                              {
                                                                  return point.separation_m > math::geometric_epsilon;
                                                              });
            const auto prescribed_curve = [&](BodyId id, const RigidBody& body)
            {
                return body.type() == BodyType::kinematic_body &&
                    (slots_[id.index].motion.has_value() || body.angular_velocity_rad_s() != 0.0);
            };
            // Ordinary near contacts already constrain their endpoint displacement. Prescribed
            // curved motion needs an interior sweep even inside the contact margin: its endpoint
            // chord may vanish, and the post-integration guard cannot shorten a prescribed path.
            const auto sweep_existing_gap = only_positive_points && (prescribed_curve(pair.first, first) || prescribed_curve(pair.second, second));
            if ((!collided || sweep_existing_gap) && settings_.collision.continuous && !query.is_sensor &&
                dynamic_cast<const CollisionNarrowPhase*>(narrow_phase_.get()) != nullptr)
            {
                const auto first_sweep = make_sweep(pair.first, a, time_step_s, targets[pair.first.index]);
                const auto second_sweep = make_sweep(pair.second, b, time_step_s, targets[pair.second.index]);
                const auto sweep = sweep_shapes(first_sweep, second_sweep, settings_.collision.sweep_tolerance_m, settings_.collision.maximum_sweep_iterations);
                if (!sweep.converged)
                    ++result.sweep_limits;
                if (sweep.hit && sweep.distance.valid)
                {
                    auto at_impact = query;
                    at_impact.first_transform = first_sweep.transform_at_fraction(sweep.fraction);
                    at_impact.second_transform = second_sweep.transform_at_fraction(sweep.fraction);
                    at_impact.contact_margin_m = settings_.collision.sweep_tolerance_m * 2.0;
                    if (!narrow_phase_->collide(at_impact, manifold) || manifold.is_empty())
                    {
                        manifold = {};
                        manifold.normal = sweep.distance.normal;
                        manifold.point_count = 1;
                        manifold.points[0].world_position_m = (sweep.distance.first_point_m + sweep.distance.second_point_m) * 0.5;
                        manifold.points[0].separation_m = sweep.distance.intersecting ? -sweep.distance.penetration_depth_m : sweep.distance.distance_m;
                        manifold.points[0].feature.incoming_edge = 65535;
                    }
                    for (std::size_t index = 0; index < manifold.point_count; ++index)
                    {
                        auto& point = manifold.points[index];
                        const auto first_at_impact = point.world_position_m - manifold.normal * (point.separation_m * 0.5);
                        const auto second_at_impact = point.world_position_m + manifold.normal * (point.separation_m * 0.5);
                        const auto first_now = math::transform_point(query.first_transform, math::inverse_transform_point(at_impact.first_transform, first_at_impact));
                        const auto second_now = math::transform_point(query.second_transform, math::inverse_transform_point(at_impact.second_transform, second_at_impact));
                        point.world_position_m = (first_now + second_now) * 0.5;
                        point.separation_m = math::dot(second_now - first_now, manifold.normal);
                        point.local_anchor_first_m = math::inverse_transform_point(first.transform(), first_now);
                        point.local_anchor_second_m = math::inverse_transform_point(second.transform(), second_now);
                        point.body_anchors_valid = true;
                        // Curved motion may return to its starting pose within this step. Use
                        // the interval leading to impact, not its cancelling endpoint chord.
                        const auto interval = std::max(sweep.fraction, 1.0e-8) * time_step_s;
                        const auto average_velocity = ((second_at_impact - second_now) - (first_at_impact - first_now)) / interval;
                        const auto actual_velocity = second.velocity_at_world_point(second_now) - first.velocity_at_world_point(first_now);
                        point.speculative_velocity_bias_m_s = math::dot(actual_velocity - average_velocity, manifold.normal);
                    }
                    collided = true;
                    swept_contact = true;
                }
            }
            if (!collided)
                return;
            manifold.first = pair.first;
            manifold.second = pair.second;
            manifold.first_collider = pair.first_collider;
            manifold.second_collider = pair.second_collider;
            manifold.material = query.material;
            manifold.is_sensor = query.is_sensor;
            manifold.point_count = std::min(manifold.point_count, maximum_manifold_points);
            if (!math::is_finite(manifold.normal))
                return;
            manifold.normal = math::normalized(manifold.normal);
            if (math::length_squared(manifold.normal) == 0.0)
                manifold.normal = { 1.0, 0.0 };
            bool touching = false;
            bool valid = true;
            for (std::size_t index = 0; index < manifold.point_count; ++index)
            {
                auto& point = manifold.points[index];
                valid = valid && math::is_finite(point.world_position_m) && math::is_finite(point.separation_m);
                touching = touching || point.separation_m <= settings_.collision.sweep_tolerance_m;
                const auto first_point = point.body_anchors_valid ? math::transform_point(first.transform(), point.local_anchor_first_m)
                                                                  : point.world_position_m - manifold.normal * (point.separation_m * 0.5);
                const auto second_point = point.body_anchors_valid ? math::transform_point(second.transform(), point.local_anchor_second_m)
                                                                   : point.world_position_m + manifold.normal * (point.separation_m * 0.5);
                point.local_anchor_first_m = math::inverse_transform_point(first.transform(), first_point);
                point.local_anchor_second_m = math::inverse_transform_point(second.transform(), second_point);
                point.body_anchors_valid = true;
                if (point.separation_m > 0.0 && !swept_contact)
                {
                    const auto first_end = math::transform_point(body_placement(targets[pair.first.index], first.mass_properties().center_of_mass_m), point.local_anchor_first_m);
                    const auto second_end = math::transform_point(body_placement(targets[pair.second.index], second.mass_properties().center_of_mass_m), point.local_anchor_second_m);
                    const auto average_velocity = ((second_end - second_point) - (first_end - first_point)) / time_step_s;
                    const auto velocity = second.velocity_at_world_point(second_point) - first.velocity_at_world_point(first_point);
                    point.speculative_velocity_bias_m_s = math::dot(velocity - average_velocity, manifold.normal);
                }
            }
            if (!valid)
                return;
            manifold.is_speculative = !touching;
            result.touching = touching;

            if (!query.is_sensor && !first.motion_edited_ && !second.motion_edited_)
            {
                const auto found = previous_index.find({ pair, false });
                const auto* old = found == previous_index.end() ? nullptr : found->second;
                if (old && math::dot(old->normal, manifold.normal) > 0.95)
                {
                    std::array<bool, maximum_manifold_points> used {};
                    for (std::size_t index = 0; index < manifold.point_count; ++index)
                    {
                        auto& point = manifold.points[index];
                        std::size_t best = maximum_manifold_points;
                        Real best_distance = match_distance_squared;
                        for (std::size_t prior = 0; prior < std::min(old->point_count, maximum_manifold_points); ++prior)
                        {
                            if (used[prior])
                                continue;
                            const auto& candidate = old->points[prior];
                            const auto distance = std::max(math::length_squared(candidate.local_anchor_first_m - point.local_anchor_first_m),
                                math::length_squared(candidate.local_anchor_second_m - point.local_anchor_second_m));
                            if (distance <= best_distance)
                            {
                                best = prior;
                                best_distance = distance;
                                if (point.feature.key() == candidate.feature.key())
                                    break;
                            }
                        }
                        if (best != maximum_manifold_points)
                        {
                            used[best] = true;
                            const auto& candidate = old->points[best];
                            point.normal_impulse_n_s = candidate.normal_impulse_n_s * impulse_scale;
                            point.tangent_impulse_n_s = candidate.tangent_impulse_n_s * impulse_scale;
                            point.rolling_impulse_n_m_s = candidate.rolling_impulse_n_m_s * impulse_scale;
                            point.spinning_impulse_n_m_s = candidate.spinning_impulse_n_m_s * impulse_scale;
                            point.pending_impact_speed_m_s = candidate.pending_impact_speed_m_s;
                            ++result.persistent_points;
                        }
                    }
                }
            }
            result.collided = true;
        };
        const auto built_in_shape = [](const Shape* shape)
        {
            return dynamic_cast<const CircleShape*>(shape) || dynamic_cast<const ConvexPolygonShape*>(shape) || dynamic_cast<const SegmentShape*>(shape);
        };
        const auto independent_narrow_phase =
            dynamic_cast<const NullNarrowPhase*>(narrow_phase_.get()) ||
            (dynamic_cast<const CollisionNarrowPhase*>(narrow_phase_.get()) &&
                std::all_of(pairs_.begin(), pairs_.end(), [&](const auto& pair)
                    {
                        return built_in_shape(find_body(pair.first)->colliders()[pair.first_collider].shape.get()) &&
                            built_in_shape(find_body(pair.second)->colliders()[pair.second_collider].shape.get());
                    }));
        if (independent_narrow_phase)
        {
            profile_.narrow_phase_workers = parallel_executor().run(pairs_.size(), parallel_settings_.minimum_batch_size, [&](std::size_t begin, std::size_t end, std::size_t)
                {
                    for (auto index = begin; index < end; ++index)
                        process_pair(index);
                });
        }
        else
        {
            profile_.narrow_phase_workers = pairs_.empty() ? 0 : 1;
            for (std::size_t index = 0; index < pairs_.size(); ++index)
                process_pair(index);
        }
        for (std::size_t index = 0; index < pairs_.size(); ++index)
        {
            const auto& result = results[index];
            current_pairs.push_back({ pairs_[index], CollisionEventKind::begin, result.is_sensor });
            statistics_.sweep_iteration_limit_count += result.sweep_limits;
            statistics_.persistent_contact_point_count += result.persistent_points;
            if (result.collided)
            {
                if (result.touching)
                    current_contacts.push_back({ pairs_[index], CollisionEventKind::begin, result.is_sensor });
                else
                    ++statistics_.speculative_manifold_count;
                manifolds_.push_back(result.manifold);
            }
        }
        track_events(active_pairs_, std::move(current_pairs), pair_events_, pending_pair_events_);
        track_events(active_contacts_, std::move(current_contacts), contact_events_, pending_contact_events_);
        previous_contact_step_s_ = time_step_s;
    }

    void World::remove_collision_state(BodyId id)
    {
        impact_episodes_.erase(std::remove_if(impact_episodes_.begin(), impact_episodes_.end(), [id](const auto& episode)
                                   {
                                       return episode.report.first_before.id == id || episode.report.second_before.id == id;
                                   }),
            impact_episodes_.end());
        const auto matches = [id](const BroadPhasePair& pair)
        {
            return pair.first == id || pair.second == id;
        };
        const auto queue_ends = [&](auto& active, auto& pending)
        {
            for (const auto& event : active)
            {
                if (matches(event.pair))
                {
                    auto ended = event;
                    ended.kind = CollisionEventKind::end;
                    pending.push_back(ended);
                }
            }
            active.erase(std::remove_if(active.begin(), active.end(), [&](const auto& event)
                             {
                                 return matches(event.pair);
                             }),
                active.end());
        };
        queue_ends(active_pairs_, pending_pair_events_);
        queue_ends(active_contacts_, pending_contact_events_);
        pairs_.erase(std::remove_if(pairs_.begin(), pairs_.end(), matches), pairs_.end());
        manifolds_.erase(std::remove_if(manifolds_.begin(), manifolds_.end(), [&](const auto& manifold)
                             {
                                 return matches(pair_of(manifold));
                             }),
            manifolds_.end());
    }

    const std::vector<CollisionEvent>& World::pair_events() const
    {
        return pair_events_;
    }
    const std::vector<CollisionEvent>& World::contact_events() const
    {
        return contact_events_;
    }
}
