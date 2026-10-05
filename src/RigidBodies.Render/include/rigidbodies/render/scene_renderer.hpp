#pragma once

#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/physics/world.hpp>
#include <rigidbodies/render/camera2d.hpp>
#include <rigidbodies/render/draw_list.hpp>
#include <rigidbodies/render/relativity_stage.hpp>
#include <rigidbodies/render/visualization_layers.hpp>

#include <deque>
#include <optional>
#include <array>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigidbodies::render
{

    // The scale bar sorts above every scene layer, so an overlay placed between the two can dim
    // the scene while the bar stays readable.
    inline constexpr int instrument_layer = 30;

    // How long a vector quantity is drawn on screen for one unit of it.
    //
    // Physical quantities have no natural length in pixels, and a single fixed factor either makes
    // a slow object show nothing or makes a fast one leave the window. Each quantity therefore gets
    // its own factor, and the values are exposed so that the interface can offer them.
    struct VectorScales
    {
        // Pixels per metre per second.
        double velocity { 24.0 };

        // Pixels per metre per second squared.
        double acceleration { 6.0 };

        // Pixels per newton.
        double force { 6.0 };

        // Pixels per kilogram metre per second.
        double momentum { 30.0 };

        // Vectors shorter than this are not drawn, so that a scene at rest is not covered in
        // arrowheads pointing at nothing.
        float minimum_drawn_length_px { 6.0f };

        // Vectors longer than this are drawn clipped, with the length still reported numerically.
        float maximum_drawn_length_px { 400.0f };

        // Independent per-quantity maxima map to this length when automatic scaling is on.
        // Manual factors remain pixels per SI unit, regardless of the displayed units.
        bool automatic { false };
        float automatic_target_length_px { 120.0f };
    };

    enum class VectorComponents
    {
        none,
        world_axes,
        custom_axes,
        contact_axes
    };

    struct SceneRenderSettings
    {
        Theme theme;
        LayerMask layers { LayerMask::defaults() };
        VectorScales vector_scales;
        VectorComponents vector_components { VectorComponents::none };
        bool vectors_selected_only { false };
        // Counter-clockwise world angle of the u axis, useful for along/across-ramp decomposition.
        double component_angle_rad { 0.0 };
        core::DisplayUnits display_units { core::DisplayUnits::si };

        // Number of past positions kept per body for the trajectory layer.
        std::size_t trajectory_length { 240 };

        // Body drawn with the selection highlight, if any.
        physics::BodyId selection;
        physics::BodyId hover;
        // A body whose stage label is left out because an overlay card currently names it.
        physics::BodyId label_replaced;

        // Fraction between the last fixed step's starting and ending poses. A paused view uses
        // 1.0 so that edits and explicit single steps appear immediately.
        double interpolation_alpha { 1.0 };

        // Presentation only. Every effect has its own switch, and all working storage has a
        // fixed ceiling irrespective of scene size. Budgets are clamped to those ceilings.
        bool material_shading { true };
        bool contact_shadows { true };
        bool depth_background { true };
        bool motion_trails { true };
        bool directional_blur { true };
        bool impact_flashes { true };
        bool impact_sparks { true };
        bool impact_dust { true };
        bool soft_deformation { true };
        bool transitions { true };
        float display_scale { 1.0f };
        // The reader's text-size preference. It enlarges stage labels without thickening lines.
        float text_scale { 1.0f };
        std::size_t shading_body_budget { 512 };
        std::size_t motion_body_budget { 64 };
        std::size_t directional_blur_budget { 64 };
        std::size_t contact_shadow_budget { 96 };
        std::size_t impact_flash_budget { 24 };
        std::size_t spark_budget { 128 };
        std::size_t dust_budget { 64 };
        std::size_t deformation_budget { 32 };
    };

    struct MaterialAppearance
    {
        Color surface, highlight, rim;
        float roughness { 0.5f };
        float softness { 0.0f };
        // The tone turned away from the light. Shading runs shade -> surface -> highlight
        // along a fixed top-left light, so every body on the stage reads as lit the same way.
        Color shade;
        // Strength of a soft reflective band across the body (metals and glass).
        float specular { 0.0f };
        // Strength of the round highlight on circular bodies.
        float gloss { 0.0f };
        // Opacity of the tonal figure drawn along a wooden body's own axis.
        float grain { 0.0f };
        // Strength of the lit bevel inside the outline; glass uses a bright edge.
        float edge_light { 0.15f };
        bool translucent { false };
    };
    // The same physical material that controls mass and friction controls its appearance.
    [[nodiscard]] MaterialAppearance material_appearance(const physics::Material& material);

    struct SceneEffectStatistics
    {
        std::size_t motion_bodies {}, flashes {}, sparks {}, dust {}, deformations {};
        std::uint64_t impact_bursts {};
    };

    // Turns the state of a world into drawing commands.
    //
    // The renderer reads the world and never writes to it, and the world has no idea it exists.
    // That one-way relationship is what allows the simulation to run headless, and it is why the
    // renderer keeps its own history for trajectories rather than asking the bodies to record one.
    class SceneRenderer
    {
    public:
        // Records one frame. The list is cleared first, so a caller may reuse the same one every
        // frame and pay no allocation after it has grown.
        void render(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list);

        // Samples the current positions into the trajectory history. Called once per simulation
        // step rather than once per frame, so that a trail records the simulation rather than the
        // frame rate.
        void record_trajectory_sample(const physics::World& world, std::size_t maximum_samples);

        void clear_trajectories();

        // Names for the stage labels, so a body reads the same on the stage as in the panels. A
        // body without one is called by its humanised identifier, or "Object n" if it has none.
        void set_body_names(std::vector<std::pair<physics::BodyId, std::string>> names);

        // Screen rectangles that overlays drawn over the scene will occupy, such as selection
        // handles and the gravity compass. Stage labels are placed clear of them until the
        // areas are replaced.
        void set_overlay_areas(const std::vector<std::pair<Vec2, Vec2>>& areas);
        [[nodiscard]] const std::vector<std::pair<Vec2, Vec2>>& overlay_areas() const
        {
            return overlay_label_areas_;
        }
        // Circles an overlay strokes over the scene, such as a selection's rotation ring, given as
        // centre and radius in pixels. Plates prefer places off them.
        void set_overlay_rings(const std::vector<std::pair<Vec2, double>>& rings);
        // The whole stage left in view between the panels, which can be larger than the camera's
        // framing area. Arrows and plates may use it up to a short margin beyond the framing area,
        // which keeps them out from under a sheet laid over the stage beside it; an empty
        // rectangle means the framing area.
        void set_visible_stage(const ScreenRect& stage);

        // Every body of a multiple selection. The settings' selection stays the primary one; the
        // rest are drawn selected too, and none of their vectors recede.
        void set_selected_bodies(const std::vector<physics::BodyId>& bodies);

        // Call after each physics substep. Repeating a step index is deliberately a no-op: a
        // paused view or several render passes cannot create another burst or age a particle.
        void record_visual_sample(const physics::World& world, double time_step_s, const SceneRenderSettings& settings);
        void clear_visual_effects();
        [[nodiscard]] SceneEffectStatistics effect_statistics() const;
        // Logical pixels per newton the last frame drew force arrows at; manual scales are refitted
        // to the scene's weights, so this can differ from the requested factor.
        [[nodiscard]] double drawn_force_scale() const
        {
            return drawn_force_scale_ / drawn_pixel_scale_;
        }
        // Device pixels per metre per second the last frame drew velocity arrows at, after display
        // density and automatic lengths; handles that sit on an arrow's tip use the same factor.
        [[nodiscard]] double drawn_velocity_scale() const
        {
            return drawn_velocity_scale_;
        }
        // Advance cosmetic theme/selection easing once per application frame. Render consumes
        // this interval once, so export passes and repeated draws remain exactly repeatable.
        void advance_presentation(double real_delta_s);

        // The special-relativity apparatus to draw with the next frames, already interpolated, or
        // std::nullopt for a Newtonian experiment, which also clears what the last stage drew.
        void set_relativity_stage(std::optional<RelativityStage> stage);
        [[nodiscard]] const std::optional<RelativityStage>& relativity_stage() const
        {
            return relativity_stage_;
        }
        // Where the last frame drew the apparatus, in screen pixels.
        [[nodiscard]] const std::optional<RelativityStageLayout>& relativity_stage_layout() const
        {
            return relativity_layout_;
        }
        // The band, clocks, track and fixed plates the last frame drew, which the scale key keeps clear of.
        [[nodiscard]] const std::vector<std::pair<Vec2, Vec2>>& stage_instrument_areas() const
        {
            return stage_instrument_areas_;
        }

    private:
        void draw_background(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_grid(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_body(physics::BodyId id, const physics::RigidBody& body, const math::Transform2& placement, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list, bool selected, std::string_view display_name) const;
        void draw_body_vectors(const physics::World& world, physics::BodyId id, const physics::RigidBody& body, const Vec2& center, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_contacts(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_springs(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_joints(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_trajectories(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_visual_motion(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_visual_impacts(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_contact_shadows(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list) const;
        void draw_center_markers(const SceneRenderSettings& settings, DrawList& list) const;
        // The special-relativity apparatus: the instrument band, the track with its marks, the light
        // pulse, the probe and both clocks, and the fixed lab and race plates. Records what it drew
        // in relativity_layout_ and the areas the scale key and other plates keep clear of.
        void draw_relativity_stage(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list);
        // The scale bar and force key, in the first corner of the framing area that no arrow,
        // moving body, overlay or stage instrument occupies; it keeps its corner while that stays free.
        void draw_scale_key(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list);
        // Resolves every queued annotation together so labels avoid one another, the objects
        // they describe and the vectors already drawn. Secondary text is dropped first.
        void place_labels(const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list);
        [[nodiscard]] Vec2 deform_point(physics::BodyId id, const Vec2& point, const Vec2& center, bool enabled, std::size_t budget) const;

        struct VectorOptions
        {
            // Where the shaft begins, measured from the anchor along the vector: clear of the
            // centre-of-mass symbol, or of a body too small to carry one.
            double start_offset_px {};
            // Distance from the anchor to the owner's outline along the vector. An arrow that
            // would end inside its own body is drawn beyond the outline, joined to its anchor by
            // a thin solid lead, wherever that outside stretch is clear of other bodies.
            double exit_offset_px {};
            // The tail stays on the anchor whatever the length, as velocity's does where the
            // selection's velocity knob expects it.
            bool keeps_tail {};
            // A value that must stay visible however small it scales, such as a weight: it is
            // then drawn as a short stub marked as not to scale instead of being dropped.
            bool significant {};
            // Drawn at its scaled length however short, as a component is, so that the parts add
            // up to the whole.
            bool true_length {};
            // The arrow may step sideways to stay clear of one already drawn along its line, by
            // this many pixels a step, or the house step when zero.
            bool may_shift {};
            double shift_step_px {};
            // Another body is selected, so this one's arrows recede.
            bool dimmed {};
            // A selected body's reading: its plate is guaranteed and placed first.
            bool focus {};
            // The head rests on the anchor instead of the tail, as for a push on a face. The
            // shaft then lies in the body exerting the push and stops before entering any other.
            bool ends_at_anchor {};
            std::uint64_t pusher {};
            // The shaft stops before it would enter another body, as for a joint's pull.
            bool stops_at_bodies {};
            // A drawn length the arrow is cut back to, marked as shortened; zero for none.
            double maximum_length_px {};
            // Added to the stub of a value too small to draw to scale, where part of the arrow
            // lies under the centre-of-mass symbol.
            double stub_extra_px {};
            // The plate gives way rather than crowd the stage, as for an unattended joint.
            bool optional_label {};
            // The body the vector describes, so that its plate keeps off that body.
            std::uint64_t owner {};
            math::Aabb owner_box;
        };

        // Draws a vector from a world anchor, applying the scale and the length limits; true when
        // the arrow is drawn and its plate queued.
        bool draw_scaled_vector(const Vec2& world_anchor_m, const Vec2& quantity, double pixels_per_unit, const Color& color, const Camera2D& camera, const SceneRenderSettings& settings,
            DrawList& list, std::string_view label, core::DisplayQuantity unit, double signed_value, ArrowHead head, float weight, std::uint64_t key,
            const VectorOptions& options) const;
        // How far a ray runs from a point before entering a body other than the owner and the one
        // allowed: `length` when it never does.
        [[nodiscard]] double clear_length(const Vec2& from, const Vec2& direction, double length, std::uint64_t owner, std::uint64_t allowed) const;
        // The contact resultant on every body, averaged over the last few solved substeps; a
        // sleeping body that touches a support carries exactly what balances its other loads.
        void gather_contact_forces(const physics::World& world) const;
        // The force factor actually drawn with: the requested one unless the scene's weights
        // would draw too long or too short to compare, and short enough that bodies resting
        // on one another keep their arrows apart.
        [[nodiscard]] double fitted_force_scale(const physics::World& world, const Camera2D& camera, const SceneRenderSettings& settings) const;
        [[nodiscard]] Vec2 contact_force_n(physics::BodyId id) const;
        [[nodiscard]] bool is_selected(physics::BodyId id, const SceneRenderSettings& settings) const;
        // The selected bodies whose readings keep their full privileges: all of a small
        // selection, only the primary one of a large selection.
        [[nodiscard]] bool is_focus(physics::BodyId id, const SceneRenderSettings& settings) const;
        // Force readings use one unit for the whole scene, the one its force key is given in.
        [[nodiscard]] std::string force_text(double value_n, const SceneRenderSettings& settings) const;
        [[nodiscard]] std::string body_display_name(const physics::World& world, physics::BodyId id) const;

        // One named force acting on a body, from a single source.
        struct BodyLoad
        {
            std::string_view name;
            Vec2 force_n;
            ArrowHead head { ArrowHead::filled };
        };
        struct BodyLoads
        {
            std::array<BodyLoad, 8> items {};
            std::size_t count {};
        };
        // Splits a body's applied load into its sources. Each spring's share is read at the
        // present state, the same instant the spring's own plate reports.
        void collect_loads(physics::BodyId id, const physics::RigidBody& body, BodyLoads& loads) const;
        // Keeps the loads worth an arrow: readable, and not lost beside the body's largest one.
        // A compact list keeps the weight and folds every other source into one resultant.
        static void keep_shown_loads(BodyLoads& loads, bool compact);
        // Joint and spring loads per body slot, refreshed once per frame.
        void gather_connection_loads(const physics::World& world, const SceneRenderSettings& settings) const;
        void draw_body_contacts(const physics::World& world, physics::BodyId id, const Camera2D& camera, const SceneRenderSettings& settings, DrawList& list, const VectorOptions& options) const;

        // Scene meshes are recycled between frames: a mesh no list still references is cleared
        // in place, so a steady frame builds its shaded surfaces without new allocations.
        [[nodiscard]] std::shared_ptr<IndexedMesh> acquire_mesh() const;

        enum class LabelPlacement
        {
            around_box,
            along_vector,
            around_point,
            inside_box,
            // Beside a drawn connection from origin to origin + direction, clear of it by the
            // request's clearance on either side.
            beside_line
        };
        // How an arrow's drawn length departs from its scale, so its plate can say so.
        enum class ScaleNote
        {
            none,
            shorter,
            longer
        };
        struct LabelRequest
        {
            std::uint64_t key {};
            int priority {};
            // Required labels are placed even when crowded; essential ones (the selection's)
            // stay required however many vector values compete for space.
            bool required {};
            bool essential {};
            // A thin line marker is named at either end, level with the line.
            bool marker {};
            // A value repeated word for word on several bodies: said once, for all of them,
            // after the names that tell the bodies apart.
            bool repeated {};
            LabelPlacement placement { LabelPlacement::around_point };
            // For a vector's value, the anchor is the arrowhead and the origin its tail.
            Vec2 anchor, direction, origin;
            double clearance {};
            math::Aabb box;
            // The body a value describes; zero for labels that name their own object.
            std::uint64_t owner {};
            std::string primary, secondary;
            Color primary_color, secondary_color;
            // Set for an arrow not drawn to scale. The words go first when the plate is crowded;
            // a small copy of the arrow's mark then keeps the note.
            ScaleNote scale_note { ScaleNote::none };
        };
        struct LabelObstacle
        {
            std::size_t offset {}, count {};
            std::uint64_t owner {};
            // For an arrow, the key of the plate that reads it: the other arrows of the same body
            // are kept clear like any other arrow.
            std::uint64_t plate {};
            // A receding arrow, which only the values of receding bodies keep clear of.
            bool receding {};
            Vec2 minimum, maximum;
        };
        void queue_label(LabelRequest request) const;
        void add_obstacle(const Vec2* points, std::size_t count, std::uint64_t owner, std::uint64_t plate = 0, bool receding = false) const;

        mutable std::vector<LabelRequest> label_requests_;
        mutable std::vector<LabelObstacle> label_obstacles_;
        // Areas already taken by fixed annotations such as the scale bar.
        mutable std::vector<std::pair<Vec2, Vec2>> reserved_label_areas_;
        std::optional<RelativityStage> relativity_stage_;
        std::optional<RelativityStageLayout> relativity_layout_;
        // The relativity band, clock faces, track and fixed plates, which the scale key keeps clear of.
        std::vector<std::pair<Vec2, Vec2>> stage_instrument_areas_;
        // How far the finish detector has filled after the light arrived; it fades with the
        // presentation clock, so a repeated pass draws it the same.
        double detector_fill_ {};
        std::vector<std::pair<Vec2, Vec2>> overlay_label_areas_;
        std::vector<std::pair<Vec2, double>> overlay_rings_;
        ScreenRect visible_stage_;
        // The corner of the framing area the scale key last stood in, and for how many frames in
        // a row its own lower-left corner has been free while it stood elsewhere.
        int key_corner_ {};
        int key_home_free_frames_ {};
        mutable std::vector<Vec2> obstacle_points_;
        struct CenterMarker
        {
            Vec2 point;
            float radius {};
            // Set when the symbol is wider than the body under it, which then needs a rim in
            // the stage colour to read as a mark rather than as part of the body.
            bool halo {};
        };
        mutable std::vector<CenterMarker> center_markers_;
        struct DrawnArrow
        {
            Vec2 start, end;
            std::uint64_t owner {};
        };
        mutable std::vector<DrawnArrow> drawn_arrows_;
        // Per body slot for the current frame; the storage is reused from frame to frame.
        struct BodyFrame
        {
            math::Aabb screen_box;
            float smaller_extent_px {}, marker_radius_px {};
            bool movable {};
        };
        mutable std::vector<BodyFrame> body_frames_;
        mutable std::vector<Vec2> contact_forces_n_;
        // Per body slot this frame: the first body pressing on it, whether another does too, and
        // where the presses act, weighted by their impulses.
        struct ContactPartners
        {
            physics::BodyId first;
            bool several {};
            Vec2 weighted_point_m;
            double weight {};
        };
        mutable std::vector<ContactPartners> contact_partners_;
        // Joint reactions and present spring forces acting on each body slot this frame.
        mutable std::vector<Vec2> joint_forces_n_, spring_forces_n_;
        // Per body slot this frame: how many springs pull on it, and whether its spring arrow was
        // drawn with its plate, which then states the force its one spring's plate leaves out.
        mutable std::vector<std::uint8_t> spring_counts_, spring_stated_;
        // The substep the latest contact impulses were solved over, from record_visual_sample.
        double contact_step_s_ {};
        // The last few substeps' contact resultant per body slot. A resting solver can settle
        // into a cycle that alternates between two solutions; their average is the support.
        struct ContactHistory
        {
            std::uint32_t generation {};
            std::array<Vec2, 8> force_n {};
            std::array<double, 8> step_s {};
            std::size_t count {}, next {};
        };
        std::vector<ContactHistory> contact_history_;
        std::vector<Vec2> contact_scratch_n_;
        std::vector<physics::BodyId> selected_bodies_;
        mutable bool force_milli_ {};
        // Heaviest weight among the bodies with arrows, and the force factor drawn this frame.
        mutable double heaviest_weight_n_ {};
        mutable double drawn_force_scale_ {};
        double drawn_velocity_scale_ {};
        double drawn_pixel_scale_ { 1.0 };
        // Without gravity the largest applied load since the run began sets the force scale, so
        // an oscillating spring keeps one scale rather than rescaling every frame. Once every
        // load has stayed far below it for a second of simulated time, it drops to the present
        // largest load in one step, which the key announces for a while.
        mutable double peak_applied_n_ {};
        double quiet_loads_s_ {}, rescaled_note_s_ {};
        mutable std::vector<std::shared_ptr<IndexedMesh>> mesh_pool_;
        mutable std::size_t mesh_pool_used_ {};
        // One mesh per frame holds the casing of every arrow in focus, listed before the first of
        // them so that the casings lie beneath them all and cost one draw item between them.
        mutable std::shared_ptr<IndexedMesh> casing_mesh_;
        // Set when more vector values compete than a stage can show: only the selection's
        // values are then formatted and placed.
        bool sparse_vector_labels_ {};
        // Set when every body's every load would crowd the stage: bodies outside the selection
        // then show their weight and one resultant of their other loads.
        bool compact_loads_ {};
        // Last chosen candidate per label, so a label keeps its side while space allows it.
        std::vector<std::pair<std::uint64_t, int>> label_memory_, next_label_memory_;

        // Trajectory history keyed by body slot. The generation is stored alongside so that a
        // reused slot starts a fresh trail rather than continuing the trail of a destroyed body.
        struct TrajectoryTrail
        {
            std::uint32_t generation { 0 };
            std::deque<Vec2> samples;
        };

        std::unordered_map<std::uint32_t, TrajectoryTrail> trajectories_;
        // Sorted by slot index for lookup.
        std::vector<std::pair<physics::BodyId, std::string>> body_names_;
        mutable std::vector<Vec2> scratch_points_;
        struct MotionSample
        {
            physics::BodyId body;
            std::array<Vec2, 12> points {};
            std::size_t count {};
            double age_s {};
        };
        struct Particle
        {
            Vec2 position, velocity;
            Color color;
            double age_s {}, lifetime_s {}, strength {};
        };
        struct Deformation
        {
            physics::BodyId body;
            Vec2 normal;
            double age_s {}, lifetime_s {}, strength {};
        };
        std::array<MotionSample, 128> motion_ {};
        std::array<Particle, 48> flashes_ {};
        std::array<Particle, 256> sparks_ {};
        std::array<Particle, 128> dust_ {};
        std::array<Deformation, 64> deformations_ {};
        std::uint64_t last_visual_step_ {}, impact_bursts_ {};
        bool has_visual_sample_ { false };
        Theme presentation_theme_;
        physics::BodyId presentation_selection_;
        double presentation_delta_s_ {}, selection_opacity_ { 1.0 };
        // The fraction of the way cosmetic easing moves this frame: the step render() consumed from
        // advance_presentation, 1 when transitions are off or on the first frame, 0 on a repeat pass.
        double presentation_fraction_ {};
        bool presentation_initialized_ { false };
    };

} // namespace rigidbodies::render
