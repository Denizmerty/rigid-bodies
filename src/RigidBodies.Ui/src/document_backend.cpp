#include <rigidbodies/ui/document_backend.hpp>
#include <rigidbodies/core/display_units.hpp>
#include <rigidbodies/core/log.hpp>
#include <rigidbodies/core/text_format.hpp>
#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

#include <RmlUi/Core.h>
#include <RmlUi/Core/ElementText.h>
#include <RmlUi/Core/Elements/ElementFormControl.h>
#include <RmlUi/Core/Elements/ElementFormControlSelect.h>
#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_map>
#include <unordered_set>

namespace rigidbodies::ui
{
    namespace
    {
        std::string px(double value)
        {
            return std::to_string(value) + "px";
        }

        std::string_view string_attribute(const Rml::Element* element, const char* name)
        {
            const auto* attribute = element ? element->GetAttribute(name) : nullptr;
            if (!attribute || attribute->GetType() != Rml::Variant::STRING)
                return {};
            return attribute->GetReference<Rml::String>();
        }

        std::string_view row_kind_name(PanelRowKind kind)
        {
            switch (kind)
            {
            case PanelRowKind::title:
                return "title";
            case PanelRowKind::heading:
                return "heading";
            case PanelRowKind::label:
                return "label";
            case PanelRowKind::paragraph:
                return "paragraph";
            case PanelRowKind::separator:
                return "separator";
            case PanelRowKind::spacer:
                return "spacer";
            case PanelRowKind::action:
                return "action";
            case PanelRowKind::meter:
                return "meter";
            case PanelRowKind::number:
                return "number";
            case PanelRowKind::stepper:
                return "stepper";
            case PanelRowKind::select:
                return "select";
            case PanelRowKind::radio_list:
                return "radio_list";
            case PanelRowKind::checklist:
                return "checklist";
            case PanelRowKind::segmented:
                return "segmented";
            case PanelRowKind::switch_control:
                return "switch_control";
            case PanelRowKind::checkbox:
                return "checkbox";
            case PanelRowKind::tabs:
                return "tabs";
            case PanelRowKind::section:
                return "section";
            case PanelRowKind::notice:
                return "notice";
            case PanelRowKind::readout:
                return "readout";
            case PanelRowKind::list_item:
                return "list_item";
            case PanelRowKind::text_field:
                return "text_field";
            case PanelRowKind::plot:
                return "plot";
            }
            return "label";
        }

        // A row presented in a horizontal strip, kept for the fitting pass that runs after layout.
        struct StripRow
        {
            Rml::Element* element { nullptr };
            PanelRow row;
            std::string id, key;
        };

        bool is_interactive(PanelRowKind kind)
        {
            switch (kind)
            {
            case PanelRowKind::action:
            case PanelRowKind::number:
            case PanelRowKind::stepper:
            case PanelRowKind::select:
            case PanelRowKind::radio_list:
            case PanelRowKind::checklist:
            case PanelRowKind::segmented:
            case PanelRowKind::switch_control:
            case PanelRowKind::checkbox:
            case PanelRowKind::tabs:
            case PanelRowKind::section:
            case PanelRowKind::list_item:
            case PanelRowKind::text_field:
                return true;
            default:
                return false;
            }
        }

        std::optional<double> option_number(std::string_view option)
        {
            double value = 0.0;
            const auto [end, error] = std::from_chars(option.data(), option.data() + option.size(), value);
            if (error != std::errc {} || end != option.data() + option.size() || !std::isfinite(value))
                return std::nullopt;
            return value;
        }

        // Numeric presets carry their number in the option id. Custom playback opens its editor;
        // descriptive Custom entries in other lists leave the current value unchanged.
        std::optional<UiCommand> choice_command(UiCommand command, const ControlSpec* spec, std::string_view option)
        {
            if (spec && spec->key == "bar.speed.choice" && option == "custom")
            {
                UiCommand edit;
                edit.detail = "view:playback_speed";
                return edit;
            }
            command.id = std::string(option);
            command.phase = UiEditPhase::commit;
            if (const auto number = option_number(option))
            {
                command.value = *number;
                return command;
            }
            if (spec && std::any_of(spec->options.begin(), spec->options.end(), [](const OptionSpec& candidate)
                            {
                                return option_number(candidate.id).has_value();
                            }))
                return std::nullopt;
            return command;
        }

        bool takes_keyboard_focus(const Rml::Element& element)
        {
            const auto& tag = element.GetTagName();
            return tag == "button" || tag == "input" || tag == "select" || tag == "textarea" || element.HasAttribute("tabindex");
        }

        std::string slug(std::string_view text)
        {
            std::string result;
            bool underscore = false;
            for (const auto c : text)
            {
                if (std::isalnum(static_cast<unsigned char>(c)))
                {
                    result.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
                    underscore = false;
                }
                else if (!result.empty() && !underscore)
                {
                    result.push_back('_');
                    underscore = true;
                }
            }
            while (!result.empty() && result.back() == '_')
                result.pop_back();
            if (result.empty() || !std::isalpha(static_cast<unsigned char>(result.front())))
                result.insert(0, "row_");
            return result;
        }

        std::string command_key(const PanelRow& row, std::string_view panel)
        {
            if (!row.key.empty())
                return row.key;
            if (row.text == "Keyboard reference")
                return "tools.reference";
            if (panel == "simulation_controls")
            {
                if (row.text == "Elapsed")
                    return "bar.time.readout";
                if (row.text == "Time skipped")
                    return "world.statistics.running_behind";
                if (row.text == "Step")
                    return "world.advanced.time_step";
                if (row.text == "Sleeping")
                    return "world.statistics.objects";
                if (row.text == "Limit warnings")
                    return "world.statistics.numeric_limits";
            }
            if (panel == "status_line" && row.text == "Time")
                return "bar.time.readout";
            if (panel == "object_inspector")
            {
                if (row.text == "Mass")
                    return "object.properties.mass";
                if (row.text == "Inertia")
                    return "object.properties.inertia";
                if (row.text == "Centre of mass")
                    return "object.properties.centre_of_mass";
                if (row.text == "Position")
                    return "object.motion.position";
                if (row.text == "Orientation")
                    return "object.motion.rotation";
                if (row.text == "Velocity")
                    return "object.motion.velocity";
                if (row.text == "Speed")
                    return "object.motion.speed";
                if (row.text == "Spin")
                    return "object.motion.spin";
            }
            if (panel == "material")
            {
                if (row.text == "Restitution")
                    return "object.properties.bounciness";
                if (row.text == "Static friction")
                    return "object.properties.grip_rest";
                if (row.text == "Kinetic friction")
                    return "object.properties.grip_sliding";
                if (row.text == "Drag coefficient")
                    return "object.properties.drag_coefficient";
            }
            using K = UiCommandKind;
            switch (row.command.kind)
            {
            case K::toggle_pause:
                return "transport.play";
            case K::single_step:
                return "transport.step";
            case K::reset_scenario:
                return "transport.back_to_start";
            case K::set_time_scale:
                return "bar.speed.choice";
            case K::frame_subject:
                return "camera.frame.subject";
            case K::frame_all:
                return "camera.frame.everything";
            case K::frame_selection:
                return "camera.frame.selection";
            case K::set_selected_mass:
                return "object.properties.mass";
            case K::use_selected_density_mass:
                return "object.properties.use_density";
            case K::set_selected_material:
                return "object.properties.material";
            case K::set_selected_gravity_scale:
                return "object.properties.gravity_scale";
            case K::set_selected_velocity_x:
                return "object.motion.velocity_x";
            case K::set_selected_velocity_y:
                return "object.motion.velocity_y";
            case K::set_selected_angular_velocity:
                return "object.motion.spin";
            case K::set_joint_motor_enabled:
                return "joint.motor.enabled";
            case K::reverse_joint_motor:
                return "joint.motor.reverse";
            case K::set_joint_limits_enabled:
                return "joint.stops.enabled";
            case K::set_gravity_enabled:
                return "world.gravity.enabled";
            case K::set_gravity_magnitude:
                return "world.gravity.strength";
            case K::set_gravity_angle_degrees:
                return "world.gravity.tilt";
            case K::set_gravity_preset:
                return "world.gravity.preset";
            case K::set_drag_enabled:
                return "world.air.resistance";
            case K::set_angular_drag_enabled:
                return "world.air.spin_slowing";
            case K::set_magnus_enabled:
                return "world.air.spin_lift";
            case K::set_continuous_collision:
                return "world.collisions.catch_fast";
            case K::set_warm_starting:
                return "world.advanced.reuse_impulses";
            case K::set_constraint_graph:
                return "world.advanced.solve_joints_together";
            case K::set_fixed_step:
                return "world.advanced.time_step";
            case K::set_vector_auto_scale:
                return "show.arrows.auto_length";
            case K::set_vector_scale:
                return "show.arrows.scale";
            case K::set_component_angle_degrees:
                return "show.arrows.direction";
            case K::set_layer_mask:
                return "show.presets.preset";
            case K::set_layer:
                return "show.layer.item";
            case K::set_display_units:
                return "prefs.units.system";
            case K::set_theme:
                return "prefs.appearance.theme";
            case K::set_ui_scale:
                return "prefs.appearance.text_size";
            case K::set_camera_zoom_sensitivity:
                return "prefs.controls.zoom_speed";
            case K::set_pause_on_impact:
                return "measure.collisions.pause_each";
            case K::clear_energy_history:
                return "measure.graph.clear";
            case K::select_impact:
                return "measure.collisions.list";
            case K::select_authored_part:
                return "object.shape.parts";
            case K::start_new_shape:
                return "tools.draw";
            case K::edit_selected_shape:
                return "object.shape.edit";
            case K::export_shape:
                return "object.shape.export";
            case K::split_selected_body:
                return "object.shape.separate";
            case K::set_shape_snap_grid:
                return "draw.snap.grid";
            case K::set_shape_snap_vertices:
                return "draw.snap.points";
            case K::set_shape_snap_angles:
                return "draw.snap.angles";
            case K::set_shape_grid_spacing:
                return "draw.snap.grid_spacing";
            case K::set_shape_vertex_budget:
                return "draw.precision.vertex_budget";
            case K::set_shape_render_tolerance:
                return "draw.precision.drawing";
            case K::set_shape_collision_tolerance:
                return "draw.precision.collision";
            case K::set_shape_simplification_tolerance:
                return "draw.precision.simplify";
            case K::set_shape_concavity_tolerance:
                return "draw.precision.hollows";
            case K::set_shape_edge:
                return "draw.node.edge";
            case K::set_shape_continuity:
                return "draw.node.join";
            case K::set_shape_material:
                return "draw.bar.material";
            case K::insert_shape_node:
                return "draw.node.insert";
            case K::remove_shape_node:
                return "draw.node.remove";
            case K::close_shape_outline:
                return "draw.actions.close";
            case K::commit_shape_outline:
                return "draw.actions.apply";
            case K::cancel_shape_outline:
                return "draw.actions.discard";
            case K::undo:
                return "bar.history.undo";
            case K::redo:
                return "bar.history.redo";
            case K::set_interaction_mode:
                return row.command.id.empty() ? "tools.select" : std::string("tools.") + row.command.id;
            case K::load_scenario:
                return "library.cards.card";
            default:
                break;
            }
            auto panel_segment = slug(panel);
            auto control_segment = slug(row.text.empty() ? row_kind_name(row.kind) : row.text);
            return "legacy_" + panel_segment + "." + control_segment;
        }

        bool contains_element(const Rml::Element* ancestor, const Rml::Element* element)
        {
            for (auto* current = element; current; current = current->GetParentNode())
                if (current == ancestor)
                    return true;
            return false;
        }

        Rml::Input::KeyIdentifier rml_key(UiKey key)
        {
            using R = Rml::Input::KeyIdentifier;
            switch (key)
            {
            case UiKey::escape:
                return R::KI_ESCAPE;
            case UiKey::space:
                return R::KI_SPACE;
            case UiKey::enter:
                return R::KI_RETURN;
            case UiKey::backspace:
                return R::KI_BACK;
            case UiKey::delete_key:
                return R::KI_DELETE;
            case UiKey::arrow_left:
                return R::KI_LEFT;
            case UiKey::arrow_right:
                return R::KI_RIGHT;
            case UiKey::arrow_up:
                return R::KI_UP;
            case UiKey::arrow_down:
                return R::KI_DOWN;
            case UiKey::period:
                return R::KI_OEM_PERIOD;
            case UiKey::a:
                return R::KI_A;
            case UiKey::c:
                return R::KI_C;
            case UiKey::d:
                return R::KI_D;
            case UiKey::f:
                return R::KI_F;
            case UiKey::g:
                return R::KI_G;
            case UiKey::h:
                return R::KI_H;
            case UiKey::i:
                return R::KI_I;
            case UiKey::j:
                return R::KI_J;
            case UiKey::k:
                return R::KI_K;
            case UiKey::l:
                return R::KI_L;
            case UiKey::m:
                return R::KI_M;
            case UiKey::n:
                return R::KI_N;
            case UiKey::o:
                return R::KI_O;
            case UiKey::p:
                return R::KI_P;
            case UiKey::q:
                return R::KI_Q;
            case UiKey::r:
                return R::KI_R;
            case UiKey::s:
                return R::KI_S;
            case UiKey::t:
                return R::KI_T;
            case UiKey::v:
                return R::KI_V;
            case UiKey::w:
                return R::KI_W;
            case UiKey::x:
                return R::KI_X;
            case UiKey::y:
                return R::KI_Y;
            case UiKey::z:
                return R::KI_Z;
            case UiKey::digit_0:
                return R::KI_0;
            case UiKey::comma:
                return R::KI_OEM_COMMA;
            case UiKey::slash:
                return R::KI_OEM_2;
            case UiKey::left_bracket:
                return R::KI_OEM_4;
            case UiKey::right_bracket:
                return R::KI_OEM_6;
            case UiKey::minus:
                return R::KI_OEM_MINUS;
            case UiKey::equals:
                return R::KI_OEM_PLUS;
            case UiKey::keypad_plus:
                return R::KI_ADD;
            case UiKey::keypad_minus:
                return R::KI_SUBTRACT;
            case UiKey::home:
                return R::KI_HOME;
            case UiKey::end:
                return R::KI_END;
            case UiKey::page_up:
                return R::KI_PRIOR;
            case UiKey::page_down:
                return R::KI_NEXT;
            case UiKey::f1:
                return R::KI_F1;
            case UiKey::f5:
                return R::KI_F5;
            case UiKey::f6:
                return R::KI_F6;
            case UiKey::f9:
                return R::KI_F9;
            case UiKey::f10:
                return R::KI_F10;
            case UiKey::f12:
                return R::KI_F12;
            default:
                return R::KI_UNKNOWN;
            }
        }
        struct DocumentSystem final : Rml::SystemInterface
        {
            ClipboardReader clipboard_reader;
            ClipboardWriter clipboard_writer;
            Rect keyboard_area;
            CursorShape cursor_shape { CursorShape::arrow };
            double GetElapsedTime() override
            {
                return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            }
            bool LogMessage(Rml::Log::Type type, const Rml::String& message) override
            {
                if (type <= Rml::Log::LT_WARNING)
                    core::log_warning("RmlUi: {}", message);
                return true;
            }
            void SetClipboardText(const Rml::String& value) override
            {
                if (clipboard_writer)
                    clipboard_writer(value);
            }
            void GetClipboardText(Rml::String& value) override
            {
                value = clipboard_reader ? clipboard_reader() : std::string {};
            }
            void ActivateKeyboard(Rml::Vector2f caret_position, float line_height) override
            {
                keyboard_area = { { caret_position.x, caret_position.y }, { caret_position.x + 2.0, caret_position.y + line_height } };
            }
            void DeactivateKeyboard() override
            {
                keyboard_area = {};
            }
            void SetMouseCursor(const Rml::String& name) override
            {
                if (name == "pointer")
                    cursor_shape = CursorShape::pointer;
                else if (name == "text")
                    cursor_shape = CursorShape::text;
                else if (name == "crosshair")
                    cursor_shape = CursorShape::crosshair;
                else if (name == "move")
                    cursor_shape = CursorShape::move;
                else if (name == "ns-resize")
                    cursor_shape = CursorShape::resize_ns;
                else if (name == "ew-resize")
                    cursor_shape = CursorShape::resize_ew;
                else if (name == "unavailable" || name == "not-allowed")
                    cursor_shape = CursorShape::not_allowed;
                else
                    cursor_shape = CursorShape::arrow;
            }
        };
        struct DocumentRenderer final : Rml::RenderInterface
        {
            struct Geometry
            {
                std::vector<Rml::Vertex> vertices;
                std::vector<int> indices;
            };
            std::unordered_map<Rml::CompiledGeometryHandle, Geometry> geometry;
            std::unordered_map<Rml::TextureHandle, std::shared_ptr<const render::TexturePixels>> textures;
            std::size_t next_handle { 1 };
            DrawList* output { nullptr };
            bool scissor { false };
            bool transformed { false };
            Rml::Matrix4f transform;
            render::MeshClip clip;
            Rml::CompiledGeometryHandle CompileGeometry(Rml::Span<const Rml::Vertex> vertices, Rml::Span<const int> indices) override
            {
                const auto handle = next_handle++;
                geometry.emplace(handle, Geometry { { vertices.begin(), vertices.end() }, { indices.begin(), indices.end() } });
                return handle;
            }
            void RenderGeometry(Rml::CompiledGeometryHandle handle, Rml::Vector2f translation, Rml::TextureHandle texture) override
            {
                if (!output)
                    return;
                const auto found = geometry.find(handle);
                if (found == geometry.end())
                    return;
                auto mesh = std::make_shared<render::IndexedMesh>();
                mesh->indices = found->second.indices;
                mesh->vertices.reserve(found->second.vertices.size());
                for (const auto& vertex : found->second.vertices)
                {
                    Vec2 position { vertex.position.x + translation.x, vertex.position.y + translation.y };
                    if (transformed)
                    {
                        // Transforms are affine in practice (rotated labels), but the divide keeps
                        // a projective matrix from producing a skewed vertex.
                        const auto projected = transform * Rml::Vector4f(static_cast<float>(position.x), static_cast<float>(position.y), 0.0f, 1.0f);
                        const auto w = std::abs(projected.w) > 1.0e-6f ? projected.w : 1.0f;
                        position = { projected.x / w, projected.y / w };
                    }
                    mesh->vertices.push_back({ position, { vertex.tex_coord.x, vertex.tex_coord.y }, Color::from_bytes(vertex.colour.red, vertex.colour.green, vertex.colour.blue, vertex.colour.alpha) });
                }
                if (const auto pixels = textures.find(texture); pixels != textures.end())
                    mesh->texture = pixels->second;
                if (scissor)
                    mesh->clip = clip;
                // Laid-out boxes sit on whole pixels; a rotated label does not.
                mesh->pixel_aligned = !transformed;
                output->add_indexed_mesh(std::move(mesh));
            }
            void ReleaseGeometry(Rml::CompiledGeometryHandle handle) override
            {
                geometry.erase(handle);
            }
            Rml::TextureHandle LoadTexture(Rml::Vector2i&, const Rml::String&) override
            {
                return 0;
            }
            Rml::TextureHandle GenerateTexture(Rml::Span<const Rml::byte> source, Rml::Vector2i size) override
            {
                if (size.x <= 0 || size.y <= 0)
                    return 0;
                auto pixels = std::make_shared<render::TexturePixels>();
                pixels->width = size.x;
                pixels->height = size.y;
                pixels->rgba.assign(source.begin(), source.end());
                const auto handle = next_handle++;
                textures.emplace(handle, std::move(pixels));
                return handle;
            }
            void ReleaseTexture(Rml::TextureHandle texture) override
            {
                textures.erase(texture);
            }
            void EnableScissorRegion(bool value) override
            {
                scissor = value;
            }
            void SetScissorRegion(Rml::Rectanglei region) override
            {
                clip = { region.Left(), region.Top(), region.Width(), region.Height() };
            }
            void SetTransform(const Rml::Matrix4f* matrix) override
            {
                transformed = matrix != nullptr;
                if (matrix)
                    transform = *matrix;
            }
        };
        struct Runtime
        {
            DocumentSystem system;
            DocumentRenderer renderer;
            std::size_t references { 0 }, serial { 0 };
        };
        Runtime& runtime()
        {
            static Runtime instance;
            return instance;
        }
        Rect bounds(Rml::Element* element)
        {
            const auto origin = element->GetAbsoluteOffset(Rml::BoxArea::Border);
            const auto size = element->GetBox().GetSize(Rml::BoxArea::Border);
            return { { origin.x, origin.y }, { origin.x + size.x, origin.y + size.y } };
        }
        void place(Rml::Element* element, const Rect& rect)
        {
            element->SetProperty("left", px(rect.minimum.x));
            element->SetProperty("top", px(rect.minimum.y));
            element->SetProperty("width", px(std::max(0.0, rect.width())));
            element->SetProperty("height", px(std::max(0.0, rect.height())));
        }
        void shadow(DrawList& list, const Rect& rect, float scale)
        {
            // Concentric rounded rings form a soft key shadow offset downwards without an off-screen
            // blur. Coverage builds towards the card's edge and fades to nothing 16 dp beyond it.
            constexpr int rings = 12;
            auto mesh = std::make_shared<render::IndexedMesh>();
            mesh->vertices.reserve(rings * 37);
            mesh->indices.reserve(rings * 108);
            for (int ring = rings; ring >= 1; --ring)
            {
                const double spread = ring * 1.35 * scale, radius = 10.0 * scale + spread;
                const Rect outer { { rect.minimum.x - spread, rect.minimum.y - spread + 5.0 * scale }, { rect.maximum.x + spread, rect.maximum.y + spread + 5.0 * scale } };
                const auto start = static_cast<int>(mesh->vertices.size());
                const auto color = Color { 0, 0, 0, 0.022f };
                mesh->vertices.push_back({ (outer.minimum + outer.maximum) * 0.5, {}, color });
                const Vec2 centers[] = { { outer.maximum.x - radius, outer.minimum.y + radius }, { outer.maximum.x - radius, outer.maximum.y - radius }, { outer.minimum.x + radius, outer.maximum.y - radius }, { outer.minimum.x + radius, outer.minimum.y + radius } };
                for (int corner = 0; corner < 4; ++corner)
                    for (int segment = 0; segment <= 8; ++segment)
                    {
                        const auto angle = (-0.5 + corner * 0.5 + segment / 16.0) * math::pi;
                        mesh->vertices.push_back({ centers[corner] + Vec2 { std::cos(angle), std::sin(angle) } * radius, {}, color });
                    }
                for (int i = 1; i <= 36; ++i)
                {
                    mesh->indices.push_back(start);
                    mesh->indices.push_back(start + i);
                    mesh->indices.push_back(start + (i % 36) + 1);
                }
            }
            list.add_indexed_mesh(std::move(mesh));
        }
    }

    struct DocumentBackend::Impl final : Rml::EventListener
    {
        struct PlotStorage
        {
            std::vector<std::vector<float>> times, values;
            PlotData data;
        };

        struct Binding
        {
            UiCommand command;
            std::shared_ptr<const ControlSpec> spec;
            std::string key;
            double model_value { 0.0 };
            core::DisplayUnits units { core::DisplayUnits::si };
            bool slider { false };
            bool scrub { false };
            // The unit shown beside the field when it was last synchronised.
            std::string unit {};
            // The field stands for several differing values and shows none; it has no value to
            // restore, step from or scrub from.
            bool mixed { false };
            // Selection-backed rows can keep their DOM identity while changing subjects.
            std::vector<physics::BodyId> selection;
            std::string document_id;
            std::shared_ptr<const void> edit_document;
        };

        static UiCommand number_command(const Binding& binding, double value, UiEditPhase phase)
        {
            auto command = binding.command;
            command.value = binding.key == "world.gravity.tilt" ? -90.0 + value : value;
            command.phase = phase;
            return command;
        }

        static bool same_number_subject(const Binding& before, const Binding& after)
        {
            const auto& a = before.command;
            const auto& b = after.command;
            return a.kind == b.kind && a.id == b.id && a.body == b.body && a.bodies == b.bodies &&
                a.detail == b.detail && before.selection == after.selection && before.document_id == after.document_id && before.edit_document == after.edit_document;
        }

        // A multiplier's sign stays with its figure ("1×"): printed alone beside the field it reads
        // as the field's clear button.
        static bool sign_in_field(const ControlSpec& spec)
        {
            return spec.number.quantity == core::DisplayQuantity::multiplier;
        }

        static std::string field_value(double value_si, const ControlSpec& spec, core::DisplayUnits units)
        {
            return sign_in_field(spec) ? core::format_quantity(value_si, spec.number.quantity, units, spec.number.decimals) : core::format_value(value_si, spec.number.quantity, units, spec.number.decimals);
        }

        static std::string field_text(const Binding& binding)
        {
            return binding.mixed ? std::string {} : field_value(binding.model_value, *binding.spec, binding.units);
        }

        // Unitless text means the unit printed beside the field, so a value shown as "2.0 mm"
        // stays 2 mm when it is confirmed unchanged instead of becoming 2 m.
        static std::optional<double> parse_field(const Binding& binding, std::string_view text)
        {
            const auto quantity = binding.spec->number.quantity;
            const auto last = text.find_last_not_of(" \t");
            const auto unitless = last != std::string_view::npos && (std::isdigit(static_cast<unsigned char>(text[last])) || text[last] == '.' || text[last] == ',');
            if (unitless && !binding.unit.empty())
                if (const auto value = core::parse_quantity(std::string(text) + binding.unit, quantity, binding.units))
                    return value;
            return core::parse_quantity(text, quantity, binding.units);
        }

        enum class ViewBindingKind
        {
            tab,
            section,
            checklist_toggle,
            checklist_option,
            text
        };
        struct ViewBinding
        {
            ViewBindingKind kind {};
            std::string key, value;
        };

        Rml::Context* context { nullptr };
        Rml::ElementDocument* document { nullptr };
        std::string context_name, text;
        std::map<std::string, UiCommand> commands;
        std::map<std::string, UiCommand> alternate_commands;
        std::map<std::string, UiCommand> shift_commands;
        std::vector<std::string> controls, selects;
        std::vector<UiCommand> pending;
        std::unordered_map<std::string, std::string> contents, structures;
        std::unordered_map<std::string, Binding> bindings;
        std::unordered_map<std::string, ViewBinding> view_bindings;
        // What each checklist shows, so the first toggle adds to its defaults rather than
        // replacing them.
        std::unordered_map<std::string, std::vector<std::string>> checklist_shown;
        std::unordered_map<std::string, std::vector<UiCommand>> commands_by_key;
        std::unordered_map<std::string, double> value_changed_at;
        std::unordered_map<std::string, double> text_written_at;
        std::unordered_map<std::string, PlotStorage> plot_storage;
        std::unordered_map<std::string, Rect> placements;
        std::unordered_map<std::string, bool> displays;
        std::vector<Rect> regions;
        // Floating surfaces that cast a soft shadow on the stage; docked chrome sits flush.
        std::vector<Rect> shadows;
        // The window's own title bar, while the document draws it: the strip's surface, the app
        // mark and the window controls, what each control sends, and the frame as last applied.
        struct TitleBar
        {
            Rml::Element *surface { nullptr }, *brand { nullptr }, *controls { nullptr };
            Rml::Element *command_bar { nullptr }, *present_strip { nullptr }, *scrim { nullptr };
            // Minimize, maximize or restore, and close, in that order.
            std::array<Rml::Element*, 3> buttons {};
            std::map<std::string, UiCommand> commands;
            WindowFrameState frame;
            bool applied { false };
            Rect strip;
            // Where the strip is a control rather than title bar: each visible control's span,
            // across the strip's full height.
            std::vector<std::pair<double, double>> client_columns;
        } title_bar;
        double caption_height_px { 0.0 };
        std::optional<Rect> hover_card_px;
        ViewportSize viewport { 1600, 900 };
        float scale { 1 };
        bool initialized { false }, compact { false }, short_present { false };
        bool layout_initialized { false }, light_theme { false }, projector_theme { false };
        bool reduce_motion { false };
        std::map<std::string, std::string> strip_signatures;
        std::map<std::string, int> strip_steps;
        // A strip title as last fitted: its full text, the text shown in the width it was given,
        // that width, and the full text's own width, which it asks for as its flex basis.
        struct FittedTitle
        {
            std::string full, shown;
            float width { -1.0f }, basis { -1.0f };
        };
        std::unordered_map<std::string, FittedTitle> fitted_titles;
        std::string overflow_signature;
        bool measure_drag { false };
        Rect measure_grip;
        std::string tooltip_target;
        double tooltip_since { 0.0 };
        bool tooltip_suppressed { false };
        Rect tooltip_anchor;
        bool pointer_captured { false };
        bool pointer_activation_queued { false };
        bool focus_keyboard_visible { false };
        std::string keyboard_scope;
        struct FocusReturn
        {
            std::string scope, element;
            bool keyboard { false };
        };
        std::vector<FocusReturn> focus_returns;
        std::string search_highlight;
        bool has_change_serial { false };
        bool preview_sent_this_frame { false };
        bool synchronizing_control { false };
        bool cancelling_pointer_edit { false };
        std::uint64_t change_serial { 0 };
        std::string active_slider;
        double drag_start_x { 0.0 }, drag_start_value { 0.0 };
        bool drag_moved { false };
        ViewState* view { nullptr };
        std::size_t structure_changes { 0 };
        std::unordered_map<std::string, std::size_t> text_write_counts;
        bool content_initialized { false };
        std::uint64_t content_change_serial { 0 };
        std::int64_t content_refresh_bucket { -1 };
        std::string content_view_signature, content_panel_signature, content_toast_signature;
        ViewportSize content_viewport {};
        float content_scale { 0.0f };
        bool content_window_controls { false };
        Vec2 pointer;

        bool keyboard_candidate(Rml::Element* element, std::string_view scope = {}) const
        {
            if (!element || !element->IsVisible(true) || !takes_keyboard_focus(*element))
                return false;
            bool in_scope = scope.empty();
            for (auto* ancestor = element; ancestor; ancestor = ancestor->GetParentNode())
            {
                if (ancestor->IsClassSet("is-disabled") || ancestor->HasAttribute("disabled"))
                    return false;
                in_scope = in_scope || ancestor->GetId() == scope || (scope == "panel-main_menu" && ancestor->GetId() == "menu-overflow");
            }
            return in_scope;
        }

        std::vector<std::string> keyboard_controls() const
        {
            std::vector<std::string> result;
            for (const auto& id : controls)
                if (keyboard_candidate(document->GetElementById(id), keyboard_scope))
                    result.push_back(id);
            return result;
        }

        std::vector<std::string> search_results() const
        {
            auto result = keyboard_controls();
            result.erase(std::remove_if(result.begin(), result.end(), [&](const std::string& id)
                             {
                                 return string_attribute(document->GetElementById(id), "data-control-key") != "search.result";
                             }),
                result.end());
            return result;
        }

        void highlight_search_result(std::string id)
        {
            search_highlight = std::move(id);
            for (const auto& result : search_results())
                if (auto* element = document->GetElementById(result))
                {
                    element->SetClass("is-selected", result == search_highlight);
                    element->SetAttribute("aria-selected", result == search_highlight ? "true" : "false");
                }
            if (auto* field = document->GetElementById(element_id("legacy", "search.field.query") + "--field"))
            {
                if (search_highlight.empty())
                    field->RemoveAttribute("aria-activedescendant");
                else
                    field->SetAttribute("aria-activedescendant", search_highlight);
            }
        }

        bool cancel_confirmation()
        {
            if (keyboard_scope != "panel-confirmation")
                return false;
            for (const auto& id : keyboard_controls())
                if (const auto command = commands.find(id); command != commands.end() && command->second.detail == "cancel")
                {
                    pending.push_back(command->second);
                    return true;
                }
            return false;
        }

        void change_keyboard_scope(std::string next_scope, const std::string& previous_focus)
        {
            if (keyboard_scope == next_scope)
                return;
            Rml::Element* next = nullptr;
            bool keyboard = true;
            const auto saved = std::find_if(focus_returns.rbegin(), focus_returns.rend(), [&](const FocusReturn& item)
                {
                    return item.scope == next_scope;
                });
            if (saved != focus_returns.rend())
            {
                next = document->GetElementById(saved->element);
                keyboard = saved->keyboard;
                focus_returns.erase(saved.base() - 1, focus_returns.end());
            }
            else if (!next_scope.empty())
                focus_returns.push_back({ keyboard_scope, previous_focus, focus_keyboard_visible });
            else
                focus_returns.clear();
            keyboard_scope = std::move(next_scope);
            if (keyboard_scope != "panel-command_search")
                search_highlight.clear();
            if (!keyboard_candidate(next, keyboard_scope))
                next = nullptr;
            if (!next && !keyboard_scope.empty())
            {
                const auto candidates = keyboard_controls();
                // Search and Save are ready for typing. A destructive confirmation starts on
                // Cancel so opening it can never turn a repeated Enter into data loss.
                for (const auto& id : candidates)
                {
                    auto* candidate = document->GetElementById(id);
                    const auto command = commands.find(id);
                    if (candidate->IsClassSet("text-field") || (keyboard_scope == "panel-confirmation" && command != commands.end() && command->second.detail == "cancel"))
                    {
                        next = candidate;
                        break;
                    }
                }
                if (!next && !candidates.empty())
                    next = document->GetElementById(candidates.front());
            }
            if (next)
            {
                focus_keyboard_visible = keyboard;
                next->Focus(keyboard);
                next->ScrollIntoView(false);
            }
            else
            {
                if (auto* focused = context->GetFocusElement())
                    focused->Blur();
                focus_keyboard_visible = false;
            }
        }

        void number_error(Rml::Element* field, const Binding& binding, bool invalid)
        {
            if (!field)
                return;
            field->SetClass("is-invalid", invalid);
            auto error_id = field->GetId();
            if (const auto suffix = error_id.rfind("--field"); suffix != std::string::npos)
                error_id.replace(suffix, std::string::npos, "--field-error");
            if (auto* error = document->GetElementById(error_id))
                set_text(error, invalid ? core::substitute("{} must be {} to {}.", binding.spec->label, core::format_quantity(binding.spec->number.minimum, binding.spec->number.quantity, binding.units), core::format_quantity(binding.spec->number.maximum, binding.spec->number.quantity, binding.units)) : "");
        }

        void synchronize_number_field(Rml::Element* element, Binding& binding, double value)
        {
            binding.model_value = value;
            binding.mixed = false;
            binding.unit = core::format_unit(value, binding.spec->number.quantity, binding.units, binding.spec->number.decimals);
            if (auto* field = dynamic_cast<Rml::ElementFormControl*>(element))
            {
                synchronizing_control = true;
                field->SetValue(field_text(binding));
                synchronizing_control = false;
                if (auto* line = field->GetParentNode(); line && line->GetNumChildren() > 1)
                    set_text(line->GetChild(1), sign_in_field(*binding.spec) ? std::string {} : binding.unit);
            }
            number_error(element, binding, false);
        }

        void cancel_pointer_gesture()
        {
            std::optional<UiCommand> cancel;
            if (const auto binding = bindings.find(active_slider); binding != bindings.end())
            {
                cancel = binding->second.command;
                cancel->phase = UiEditPhase::cancel;
            }
            // Releasing a native range input normally emits dragend and commits. Cancellation
            // must release its internal capture without committing or restarting the preview.
            cancelling_pointer_edit = true;
            context->ProcessMouseButtonUp(0, 0);
            cancelling_pointer_edit = false;
            active_slider.clear();
            drag_moved = false;
            pointer_captured = false;
            measure_drag = false;
            if (cancel)
                pending.push_back(std::move(*cancel));
        }

        void set_text(Rml::Element* element, std::string_view value)
        {
            if (!element)
                return;
            auto* text_node = element->GetNumChildren() > 0 ? dynamic_cast<Rml::ElementText*>(element->GetChild(0)) : nullptr;
            if (!text_node)
            {
                while (element->GetNumChildren() > 0)
                    element->RemoveChild(element->GetChild(0));
                element->AppendChild(document->CreateTextNode(std::string(value)));
                ++structure_changes;
                if (!element->GetId().empty())
                    ++text_write_counts[element->GetId()];
            }
            else if (text_node->GetText() != value)
            {
                text_node->SetText(std::string(value));
                if (!element->GetId().empty())
                    ++text_write_counts[element->GetId()];
            }
        }

        // A strip title asks for its full width, whatever it currently shows, so that a shortened
        // title grows back as soon as the strip has room. Returns whether the basis changed.
        bool request_title_width(Rml::Element* element, const std::string& id, const std::string& full)
        {
            auto& fitted = fitted_titles[id];
            if (fitted.full != full)
                fitted = { full, full, -1.0f, -1.0f };
            const auto basis = static_cast<float>(Rml::ElementUtilities::GetStringWidth(element, full)) + 1.0f;
            if (basis == fitted.basis)
                return false;
            fitted.basis = basis;
            fitted.width = -1.0f;
            element->SetProperty("flex-basis", px(basis));
            return true;
        }

        // Shortens a strip title to the width the strip left it, on a character boundary and
        // ending in an ellipsis; the full title becomes its tooltip. Returns whether the text
        // changed.
        bool fit_title(Rml::Element* element, const std::string& id)
        {
            auto& fitted = fitted_titles[id];
            if (element->IsClassSet("is-overflowed"))
                return false;
            const auto available = element->GetBox().GetSize(Rml::BoxArea::Content).x;
            if (std::abs(available - fitted.width) < 0.5f)
                return false;
            fitted.width = available;
            auto shown = fitted.full;
            if (static_cast<float>(Rml::ElementUtilities::GetStringWidth(element, shown)) > available + 0.5f)
            {
                constexpr std::string_view ellipsis = "\xE2\x80\xA6";
                std::vector<std::size_t> boundaries;
                for (std::size_t index = 0; index < fitted.full.size(); ++index)
                    if ((static_cast<unsigned char>(fitted.full[index]) & 0xC0) != 0x80)
                        boundaries.push_back(index);
                const auto candidate = [&](std::size_t count)
                {
                    auto text = fitted.full.substr(0, count < boundaries.size() ? boundaries[count] : fitted.full.size());
                    while (!text.empty() && text.back() == ' ')
                        text.pop_back();
                    return text + std::string(ellipsis);
                };
                std::size_t low = 0, high = boundaries.size();
                while (low < high)
                {
                    const auto middle = (low + high + 1) / 2;
                    if (static_cast<float>(Rml::ElementUtilities::GetStringWidth(element, candidate(middle))) <= available + 0.5f)
                        low = middle;
                    else
                        high = middle - 1;
                }
                shown = candidate(low);
            }
            if (shown == fitted.shown)
                return false;
            fitted.shown = shown;
            set_text(element, shown);
            if (shown != fitted.full)
                element->SetAttribute("data-tooltip", fitted.full);
            else
                element->RemoveAttribute("data-tooltip");
            return true;
        }

        Rml::Element* append_element(Rml::Element* parent, std::string_view tag, std::string_view id = {}, std::string_view class_name = {})
        {
            auto child = document->CreateElement(std::string(tag));
            if (!id.empty())
                child->SetId(std::string(id));
            for (std::size_t start = 0; start < class_name.size();)
            {
                const auto end = std::min(class_name.find(' ', start), class_name.size());
                if (end > start)
                    child->SetClass(std::string(class_name.substr(start, end - start)), true);
                start = end + 1;
            }
            ++structure_changes;
            return parent->AppendChild(std::move(child));
        }

        Rml::Element* append_text(Rml::Element* parent, std::string_view tag, std::string_view id, std::string_view class_name, std::string_view text)
        {
            auto* child = append_element(parent, tag, id, class_name);
            set_text(child, text);
            return child;
        }

        Rml::Element* create_row(Rml::Element* parent, const PanelRow& row, const std::string& id)
        {
            const auto button_outer = row.kind == PanelRowKind::action || row.kind == PanelRowKind::switch_control || row.kind == PanelRowKind::checkbox || row.kind == PanelRowKind::section || row.kind == PanelRowKind::list_item;
            auto created = document->CreateElement(button_outer ? "button" : "div");
            ++structure_changes;
            created->SetId(id);
            created->SetClass("row", true);
            created->SetClass("row-" + std::string(row_kind_name(row.kind)), true);
            created->SetAttribute("data-row-kind", std::string(row_kind_name(row.kind)));
            if (button_outer)
                created->SetAttribute("tabindex", "0");
            auto* element = parent->AppendChild(std::move(created));

            switch (row.kind)
            {
            case PanelRowKind::readout:
                append_text(element, "span", id + "--label", "name", row.text);
                append_element(element, "span", id + "--icon", "icon readout-icon");
                append_text(element, "span", id + "--value", "value", row.value);
                break;
            case PanelRowKind::meter:
            {
                append_text(element, "span", id + "--label", "name", row.text);
                append_text(element, "span", {}, "meter-value", {});
                auto* meter = append_element(element, "div", {}, "meter");
                append_element(meter, "div");
                break;
            }
            case PanelRowKind::number:
            {
                append_text(element, "label", id + "--label", "control-label scrub-label", row.text);
                auto* line = append_element(element, "div", {}, "control-line");
                auto* field = append_element(line, "input", id + "--field", "number-field");
                field->SetAttribute("type", "text");
                field->SetAttribute("tabindex", "0");
                append_text(line, "span", {}, "unit", row.spec && !sign_in_field(*row.spec) ? core::display_unit(row.spec->number.quantity, core::DisplayUnits::si) : std::string_view {});
                append_text(line, "span", {}, "editing-tag", {});
                append_text(line, "span", {}, "field-placeholder", "Mixed");
                append_text(element, "div", id + "--field-error", "field-error", {});
                if (row.spec && (row.spec->number.slider || row.spec->number.dial))
                {
                    auto* slider = append_element(element, "input", id + "--slider", row.spec->number.dial ? "dial" : "slider");
                    slider->SetAttribute("type", "range");
                    slider->SetAttribute("min", "0");
                    slider->SetAttribute("max", "1000");
                    slider->SetAttribute("step", "1");
                    slider->SetAttribute("tabindex", "-1");
                }
                break;
            }
            case PanelRowKind::stepper:
                append_text(element, "span", id + "--label", "control-label", row.text);
                append_text(element, "button", id + "--minus", "stepper-button", "−");
                append_text(element, "span", {}, "stepper-value", {});
                append_text(element, "button", id + "--plus", "stepper-button", "+");
                append_text(element, "div", {}, "control-range", {});
                break;
            case PanelRowKind::select:
                append_text(element, "label", id + "--label", "control-label", row.text);
                append_element(element, "select", id + "--field", "select")->SetAttribute("tabindex", "0");
                break;
            case PanelRowKind::checklist:
            {
                append_text(element, "label", id + "--label", "control-label", row.text);
                auto* summary = append_element(element, "button", id + "--field", "checklist-summary");
                summary->SetAttribute("tabindex", "0");
                append_text(summary, "span", id + "--field-text", "checklist-text", "Choose…");
                append_element(summary, "span", {}, "checklist-chevron");
                append_element(element, "div", id + "--options", "checklist-menu");
                break;
            }
            case PanelRowKind::segmented:
            case PanelRowKind::tabs:
            case PanelRowKind::radio_list:
                append_text(element, "div", id + "--label", "control-label", row.text);
                append_element(element, "div", id + "--options", row.kind == PanelRowKind::tabs ? "tabs" : row.kind == PanelRowKind::radio_list ? "radio-list"
                                                                                                                                                : "segmented");
                break;
            case PanelRowKind::switch_control:
            {
                append_text(element, "span", id + "--label", "switch-label", row.text);
                auto* track = append_element(element, "span", id + "--indicator", "switch-track");
                append_element(track, "span", {}, "switch-knob");
                break;
            }
            case PanelRowKind::checkbox:
                append_text(element, "span", id + "--indicator", "checkbox-box icon", "");
                append_text(element, "span", id + "--label", "checkbox-label", row.text);
                break;
            case PanelRowKind::action:
                append_element(element, "span", id + "--icon", "icon");
                append_text(element, "span", id + "--text", "label", row.text);
                append_element(element, "span", id + "--shortcut", "shortcut");
                append_element(element, "span", id + "--badge", "badge");
                break;
            case PanelRowKind::section:
                append_element(element, "span", id + "--icon", "icon caret");
                append_text(element, "span", id + "--text", "label", row.text);
                break;
            case PanelRowKind::notice:
                append_text(element, "span", {}, "notice-text", row.text);
                append_element(element, "div", id + "--actions", "notice-actions");
                break;
            case PanelRowKind::list_item:
                append_text(element, "span", id + "--label", "list-title", row.text);
                append_text(element, "span", id + "--secondary", "list-secondary", row.hint);
                append_element(element, "span", id + "--badges", "list-badges");
                append_element(element, "div", id + "--actions", "list-actions");
                break;
            case PanelRowKind::text_field:
            {
                append_text(element, "label", id + "--label", "control-label", row.text);
                auto* wrap = append_element(element, "div", {}, "field-wrap");
                append_element(wrap, "input", id + "--field", "text-field")->SetAttribute("tabindex", "0");
                append_element(wrap, "span", id + "--icon", "icon field-icon");
                append_text(wrap, "span", id + "--placeholder", "field-placeholder", row.hint);
                break;
            }
            case PanelRowKind::plot:
                append_element(element, "plot", id + "--plot", "plot-canvas")->SetAttribute("role", "img");
                break;
            default:
                set_text(element, row.text);
                break;
            }
            return element;
        }

        bool activate_view_binding(std::string_view id)
        {
            const auto local = view_bindings.find(std::string(id));
            if (local == view_bindings.end() || !view)
                return false;
            const auto& binding = local->second;
            // Text fields update through their change events. Treating a click as a view
            // activation would blur the field immediately after it receives focus.
            if (binding.kind == ViewBindingKind::text)
                return false;
            if (binding.kind == ViewBindingKind::tab)
                view->set_active_tab(binding.key, binding.value);
            else if (binding.kind == ViewBindingKind::section)
                view->set_section_open(binding.key, binding.value != "true");
            else if (binding.kind == ViewBindingKind::checklist_toggle)
            {
                if (auto* target = document->GetElementById(std::string(id)); target && target->GetParentNode())
                    target->GetParentNode()->SetClass("is-open", !target->GetParentNode()->IsClassSet("is-open"));
            }
            else
            {
                std::vector<std::string_view> shown;
                if (const auto found = checklist_shown.find(binding.key); found != checklist_shown.end())
                    shown.assign(found->second.begin(), found->second.end());
                auto values = view->checklist(binding.key, shown);
                const auto selected = std::find(values.begin(), values.end(), binding.value);
                if (selected == values.end())
                    values.push_back(binding.value);
                else
                    values.erase(selected);
                view->set_checklist(binding.key, std::move(values));
            }
            return true;
        }
        bool queue_command(std::string_view id)
        {
            const UiCommand* command = nullptr;
            if (const auto found = commands.find(std::string(id)); found != commands.end())
                command = &found->second;
            else if (const auto window = title_bar.commands.find(std::string(id)); title_bar.frame.controls && window != title_bar.commands.end() && (title_bar.frame.resizable || id != "window-maximize"))
                command = &window->second;
            if (!command)
                return false;
            pending.push_back(*command);
            // Choosing a menu item closes its menu; popovers such as Show stay open for further
            // adjustment.
            if (view)
                if (auto* element = document->GetElementById(std::string(id)))
                    for (auto* ancestor = element; ancestor; ancestor = ancestor->GetParentNode())
                    {
                        const auto& ancestor_id = ancestor->GetId();
                        if (ancestor_id == "panel-main_menu" || ancestor_id == "menu-overflow")
                        {
                            view->close_transient("main_menu");
                            break;
                        }
                        if (ancestor_id == "panel-add_menu" || ancestor_id == "panel-context_menu")
                        {
                            view->close_transient(ancestor_id == "panel-add_menu" ? "add_menu" : "context_menu");
                            break;
                        }
                    }
            constexpr std::string_view toast_prefix { "toast-" };
            constexpr std::string_view action_suffix { "--action" };
            if (id.size() > toast_prefix.size() + action_suffix.size() && id.substr(0, toast_prefix.size()) == toast_prefix && id.substr(id.size() - action_suffix.size()) == action_suffix)
            {
                const auto serial_text = id.substr(toast_prefix.size(), id.size() - toast_prefix.size() - action_suffix.size());
                std::uint64_t serial {};
                const auto [end, error] = std::from_chars(serial_text.data(), serial_text.data() + serial_text.size(), serial);
                if (error == std::errc {} && end == serial_text.data() + serial_text.size())
                {
                    UiCommand dismiss;
                    dismiss.detail = "dismiss_toast:" + std::to_string(serial);
                    pending.push_back(std::move(dismiss));
                }
            }
            return true;
        }
        void ProcessEvent(Rml::Event& event) override
        {
            if (cancelling_pointer_edit)
                return;
            const auto& type = event.GetType();
            if (type == "dragstart")
            {
                const auto* target = event.GetTargetElement();
                const auto found = bindings.find(target ? target->GetId() : std::string {});
                if (target && found != bindings.end() && found->second.slider)
                    active_slider = target->GetId();
                return;
            }
            if (type == "dragend")
            {
                const auto* target = event.GetTargetElement();
                const auto found = bindings.find(target ? target->GetId() : std::string {});
                if (target && found != bindings.end() && found->second.slider)
                {
                    pending.push_back(number_command(found->second, found->second.model_value, UiEditPhase::commit));
                    active_slider.clear();
                }
                return;
            }
            if (type == "dblclick")
            {
                for (auto* target = event.GetTargetElement(); target; target = target->GetParentNode())
                    if (const auto found = alternate_commands.find(target->GetId()); found != alternate_commands.end())
                    {
                        pending.push_back(found->second);
                        return;
                    }
                return;
            }
            if (type == "click")
            {
                if (event.GetParameter<int>("button", 0) != 0)
                    return;
                if (const auto* target = event.GetTargetElement(); target && target->GetId() == "region-scrim")
                {
                    if (cancel_confirmation())
                        return;
                    UiCommand close;
                    close.detail = "view:close";
                    pending.push_back(std::move(close));
                    return;
                }
                for (auto* target = event.GetTargetElement(); target; target = target->GetParentNode())
                {
                    if (activate_view_binding(target->GetId()))
                    {
                        pointer_activation_queued = true;
                        focus_keyboard_visible = false;
                        return;
                    }
                    if (event.GetParameter<bool>("shift_key", false))
                        if (const auto found = shift_commands.find(target->GetId()); found != shift_commands.end())
                        {
                            pending.push_back(found->second);
                            target->Blur();
                            pointer_activation_queued = true;
                            focus_keyboard_visible = false;
                            return;
                        }
                    if (queue_command(target->GetId()))
                    {
                        target->Blur();
                        pointer_activation_queued = true;
                        focus_keyboard_visible = false;
                        break;
                    }
                }
                return;
            }
            if (type == "input" || type == "change")
            {
                if (synchronizing_control)
                    return;
                auto* target = event.GetTargetElement();
                if (target)
                    if (const auto local = view_bindings.find(target->GetId()); local != view_bindings.end() && local->second.kind == ViewBindingKind::text && view)
                    {
                        if (auto* form = dynamic_cast<Rml::ElementFormControl*>(target))
                        {
                            view->set_value(local->second.key, form->GetValue());
                            if (local->second.key == "search.query")
                                highlight_search_result({});
                            for (auto* ancestor = target->GetParentNode(); ancestor; ancestor = ancestor->GetParentNode())
                                if (ancestor->IsClassSet("row-text_field"))
                                {
                                    ancestor->SetClass("is-empty", form->GetValue().empty());
                                    break;
                                }
                        }
                        return;
                    }
                const auto found = bindings.find(target ? target->GetId() : std::string {});
                auto* form = dynamic_cast<Rml::ElementFormControl*>(target);
                if (found == bindings.end() || !form)
                    return;
                if (!found->second.slider && found->second.spec && found->second.spec->kind == ControlKind::number)
                    return;
                auto command = found->second.command;
                if (found->second.slider && found->second.spec)
                {
                    const auto position = std::clamp(std::stod(form->GetValue()), 0.0, 1000.0) / 1000.0;
                    const auto& number = found->second.spec->number;
                    command.value = number.scale == NumberScale::logarithmic
                        ? number.soft_minimum * std::pow(number.soft_maximum / number.soft_minimum, position)
                        : number.soft_minimum + (number.soft_maximum - number.soft_minimum) * position;
                    if (!event.GetParameter<bool>("alt_key", false))
                        for (const auto detent : number.detents)
                        {
                            const auto detent_position = number.scale == NumberScale::logarithmic
                                ? std::log(detent / number.soft_minimum) / std::log(number.soft_maximum / number.soft_minimum)
                                : (detent - number.soft_minimum) / (number.soft_maximum - number.soft_minimum);
                            if (std::abs(position - detent_position) * bounds(target).width() <= 6.0 * scale)
                            {
                                command.value = detent;
                                break;
                            }
                        }
                    if (number.dial && event.GetParameter<bool>("shift_key", false))
                        command.value = std::round(command.value / 15.0) * 15.0;
                    if (number.decimals >= 0)
                    {
                        const auto precision = std::pow(10.0, number.decimals);
                        command.value = std::round(command.value * precision) / precision;
                    }
                    found->second.model_value = command.value;
                    active_slider = target->GetId();
                    if (!preview_sent_this_frame)
                    {
                        pending.push_back(number_command(found->second, command.value, UiEditPhase::preview));
                        preview_sent_this_frame = true;
                    }
                }
                else if (auto chosen = choice_command(std::move(command), found->second.spec.get(), form->GetValue()))
                    pending.push_back(std::move(*chosen));
            }
            if (type == "blur" && !synchronizing_control)
            {
                auto* target = event.GetTargetElement();
                const auto found = bindings.find(target ? target->GetId() : std::string {});
                auto* field = dynamic_cast<Rml::ElementFormControl*>(target);
                if (found != bindings.end() && field && found->second.spec && found->second.spec->kind == ControlKind::number && !found->second.slider && !found->second.scrub)
                {
                    const auto parsed = parse_field(found->second, field->GetValue());
                    const auto valid = parsed && *parsed >= found->second.spec->number.minimum && *parsed <= found->second.spec->number.maximum;
                    if (valid && field->GetValue() != field_text(found->second) && std::abs(*parsed - found->second.model_value) > 1.0e-12)
                    {
                        pending.push_back(number_command(found->second, *parsed, UiEditPhase::commit));
                    }
                    else if (!valid)
                    {
                        synchronizing_control = true;
                        field->SetValue(field_text(found->second));
                        synchronizing_control = false;
                    }
                    number_error(target, found->second, false);
                }
            }
        }
        bool select_box_open() const
        {
            for (const auto& id : selects)
                if (auto* select = dynamic_cast<Rml::ElementFormControlSelect*>(document->GetElementById(id)); select && select->IsVisible(true) && select->IsSelectBoxVisible())
                    return true;
            return false;
        }
        bool over(Vec2 point) const
        {
            // An open dropdown hangs outside its dock and takes every press, on an option to
            // choose it and elsewhere to close it, the way menus do, so none reaches the stage.
            if (select_box_open())
                return true;
            for (const auto& region : regions)
                if (region.contains(point))
                    return true;
            return false;
        }

        // The window control the platform reports under the pointer, where the platform owns the
        // controls' pointer input. Maximize then belongs to the platform's own snap layouts.
        Rml::Element* platform_hover() const
        {
            if (!title_bar.frame.controls)
                return nullptr;
            if (title_bar.frame.hot == WindowPart::minimize)
                return title_bar.buttons[0];
            if (title_bar.frame.hot == WindowPart::close)
                return title_bar.buttons[2];
            return nullptr;
        }

        // Brings the title bar up to date with the window; returns whether anything changed.
        bool apply_window_frame(const WindowFrameState& reported, const LayoutResult& layout)
        {
            auto next = reported;
            const auto* strip = layout.find(RegionId::title_bar);
            next.controls = reported.controls && strip;
            title_bar.strip = strip ? strip->bounds : Rect {};
            const auto& last = title_bar.frame;
            if (title_bar.applied && next.controls == last.controls && next.maximized == last.maximized && next.resizable == last.resizable && next.active == last.active && next.hot == last.hot && next.pressed == last.pressed)
                return false;
            document->SetClass("window-title-bar", next.controls);
            document->SetClass("window-inactive", next.controls && !next.active);
            if (!title_bar.applied || next.maximized != last.maximized || next.resizable != last.resizable)
            {
                auto* maximize = title_bar.buttons[1];
                maximize->SetClass("is-disabled", !next.resizable);
                if (next.resizable)
                    maximize->SetAttribute("data-tooltip", next.maximized ? "Restore down" : "Maximize");
                else
                    maximize->RemoveAttribute("data-tooltip");
                if (auto* glyph = maximize->GetFirstChild())
                    glyph->SetInnerRML(icons::utf8(next.maximized ? icons::copy : icons::box));
            }
            constexpr std::array parts { WindowPart::minimize, WindowPart::maximize, WindowPart::close };
            for (std::size_t index = 0; index < parts.size(); ++index)
            {
                title_bar.buttons[index]->SetClass("is-hot", next.hot == parts[index]);
                // As with native caption buttons, a pressed control looks pressed only while the
                // pointer is still over it.
                title_bar.buttons[index]->SetClass("is-pressed", next.pressed == parts[index] && next.hot == parts[index]);
            }
            title_bar.frame = next;
            title_bar.applied = true;
            return true;
        }

        [[nodiscard]] bool in_client_column(double x) const
        {
            return std::any_of(title_bar.client_columns.begin(), title_bar.client_columns.end(), [x](const auto& column)
                {
                    return x >= column.first && x < column.second;
                });
        }

        // What a point of the window is, regardless of any open menu: an open list box still takes
        // every press. The topmost element decides, so a toast or card over the strip is not the
        // title bar; a dialog's scrim dims the strip but leaves the window movable by it.
        [[nodiscard]] WindowPart classify(Vec2 point) const
        {
            if (!title_bar.strip.contains(point) || select_box_open())
                return WindowPart::client;
            constexpr std::array parts { WindowPart::minimize, WindowPart::maximize, WindowPart::close };
            for (const auto* element = context->GetElementAtPoint({ static_cast<float>(point.x), static_cast<float>(point.y) }); element; element = element->GetParentNode())
            {
                for (std::size_t index = 0; index < parts.size(); ++index)
                    if (element == title_bar.buttons[index])
                        return parts[index] == WindowPart::maximize && !title_bar.frame.resizable ? WindowPart::title_bar : parts[index];
                if (element == title_bar.command_bar || element == title_bar.present_strip)
                    return in_client_column(point.x) ? WindowPart::client : WindowPart::title_bar;
                if (element == title_bar.surface || element == title_bar.brand || element == title_bar.controls || element == title_bar.scrim)
                    return WindowPart::title_bar;
            }
            return WindowPart::client;
        }

        // A short delay before the tooltip appears keeps quick pointer passes over the toolbar
        // quiet; pressing hides it until the pointer moves to another control.
        void update_tooltip(double now, float frame_scale, ViewportSize frame_viewport)
        {
            auto* tooltip = document ? document->GetElementById("tooltip") : nullptr;
            if (!tooltip)
                return;
            Rml::Element* target = nullptr;
            for (auto* element = context->GetHoverElement(); element; element = element->GetParentNode())
                if (element->HasAttribute("data-tooltip"))
                {
                    target = element;
                    break;
                }
            if (!target)
                target = platform_hover();
            const auto target_id = target ? target->GetId() : std::string {};
            if (target_id != tooltip_target)
            {
                tooltip_target = target_id;
                tooltip_since = now;
                tooltip_suppressed = false;
            }
            // A press on a control the platform owns never reaches handle_event, so it is noted here.
            if (title_bar.frame.controls && title_bar.frame.pressed != WindowPart::client)
                tooltip_suppressed = true;
            const auto anchor = target ? bounds(target) : Rect {};
            const auto visible = target && !target_id.empty() && !tooltip_suppressed && now - tooltip_since >= 0.5 && anchor.width() > 0.0;
            const auto shown = displays.find("tooltip");
            if (!visible)
            {
                if (shown == displays.end() || shown->second)
                {
                    tooltip->SetProperty("display", "none");
                    displays["tooltip"] = false;
                }
                return;
            }
            set_text(document->GetElementById("tooltip-text"), string_attribute(target, "data-tooltip"));
            set_text(document->GetElementById("tooltip-key"), string_attribute(target, "data-tooltip-key"));
            if (shown == displays.end() || !shown->second || anchor.minimum.x != tooltip_anchor.minimum.x || anchor.minimum.y != tooltip_anchor.minimum.y)
            {
                tooltip->SetProperty("display", "block");
                displays["tooltip"] = true;
                context->Update();
                const auto size = bounds(tooltip);
                const auto margin = 8.0 * frame_scale;
                const auto left = std::clamp((anchor.minimum.x + anchor.maximum.x - size.width()) * 0.5, margin, std::max(margin, frame_viewport.width - size.width() - margin));
                auto top = anchor.maximum.y + 6.0 * frame_scale;
                if (top + size.height() > frame_viewport.height - margin)
                    top = anchor.minimum.y - size.height() - 6.0 * frame_scale;
                tooltip->SetProperty("left", px(left));
                tooltip->SetProperty("top", px(top));
                tooltip_anchor = anchor;
            }
            context->Update();
        }
    };

    DocumentBackend::DocumentBackend() : impl_(std::make_unique<Impl>())
    {
    }
    DocumentBackend::~DocumentBackend()
    {
        shutdown();
    }
    std::string_view DocumentBackend::name() const
    {
        return "rmlui";
    }
    bool DocumentBackend::initialize(const std::filesystem::path& asset_root)
    {
        shutdown();
        constexpr std::array font_files { "fonts/Inter-Regular.ttf", "fonts/Inter-Medium.ttf", "fonts/Inter-SemiBold.ttf", "fonts/Phosphor-Regular-subset.ttf" };
        constexpr std::array required_files { "licenses/Phosphor-MIT.txt", "fonts/OFL-Inter.txt", "ui/playground.rml", "ui/playground.rcss", "ui/geometry.rcss", "ui/components.rcss", "ui/themes.rcss" };
        for (const auto* file : font_files)
            if (!std::filesystem::is_regular_file(asset_root / file))
                return false;
        for (const auto* file : required_files)
            if (!std::filesystem::is_regular_file(asset_root / file))
                return false;
        auto& run = runtime();
        if (!run.references)
        {
            Rml::SetSystemInterface(&run.system);
            Rml::SetRenderInterface(&run.renderer);
            if (!Rml::Initialise())
                return false;
            register_plot_element();
            for (const auto* file : font_files)
                if (!Rml::LoadFontFace((asset_root / file).generic_string()))
                {
                    Rml::Shutdown();
                    return false;
                }
        }
        ++run.references;
        impl_->initialized = true;
        impl_->context_name = "rigid-bodies-" + std::to_string(++run.serial);
        impl_->context = Rml::CreateContext(impl_->context_name, { 1600, 900 });
        if (!impl_->context)
        {
            shutdown();
            return false;
        }
        impl_->document = impl_->context->LoadDocument((asset_root / "ui/playground.rml").generic_string());
        if (!impl_->document)
        {
            shutdown();
            return false;
        }
        for (const auto* id : { "region-title-bar", "region-window-brand", "region-window-controls", "region-command-bar", "region-draw-bar", "region-present-strip", "region-guide-rail", "region-guide", "region-stage", "region-banner", "region-inspector-rail", "region-inspector", "region-measure", "region-status-line", "region-scrim", "region-sheets", "region-popovers" })
        {
            if (!impl_->document->GetElementById(id))
            {
                shutdown();
                return false;
            }
        }
        // A collapsed side surface is a purpose-built affordance rather than the full panel
        // squeezed into a 28 dp column: one large expand control with an icon and a short label.
        const auto build_rail = [&](const char* region_id, const char* button_id, std::string_view icon_name, const char* label)
        {
            auto* rail = impl_->document->GetElementById(region_id);
            auto button = impl_->document->CreateElement("button");
            button->SetId(button_id);
            button->SetClass("rail-expand", true);
            button->SetAttribute("tabindex", "0");
            auto icon = impl_->document->CreateElement("span");
            icon->SetClass("icon", true);
            icon->SetClass("icon-mirror", icon_name == icons::sidebar_right);
            icon->SetInnerRML(icons::utf8(icon_name));
            button->AppendChild(std::move(icon));
            auto text = impl_->document->CreateElement("span");
            text->SetClass("rail-label", true);
            text->SetInnerRML(label);
            button->AppendChild(std::move(text));
            // The narrow window's ribbon reads as a control that opens something: a trailing
            // chevron, shown only in the horizontal ribbon.
            auto chevron = impl_->document->CreateElement("span");
            chevron->SetClass("icon", true);
            chevron->SetClass("rail-chevron", true);
            chevron->SetInnerRML(icons::utf8(icons::caret_down));
            button->AppendChild(std::move(chevron));
            rail->AppendChild(std::move(button));
        };
        build_rail("region-guide-rail", "rail-guide", icons::guide, "Guide");
        build_rail("region-inspector-rail", "rail-inspector", icons::sidebar_right, "Inspector");
        // The window's own title bar. Its controls are caption buttons rather than toolbar
        // controls: a divider sets them off from the toolbar, and they stay out of the Tab order,
        // as the platform's do.
        auto& title_bar = impl_->title_bar;
        title_bar.surface = impl_->document->GetElementById("region-title-bar");
        title_bar.brand = impl_->document->GetElementById("region-window-brand");
        title_bar.controls = impl_->document->GetElementById("region-window-controls");
        title_bar.command_bar = impl_->document->GetElementById("region-command-bar");
        title_bar.present_strip = impl_->document->GetElementById("region-present-strip");
        title_bar.scrim = impl_->document->GetElementById("region-scrim");
        auto divider = impl_->document->CreateElement("div");
        divider->SetClass("window-divider", true);
        title_bar.controls->AppendChild(std::move(divider));
        const auto build_window_control = [&](std::size_t index, const char* id, std::string_view icon_name, const char* tooltip, UiCommandKind kind)
        {
            auto button = impl_->document->CreateElement("button");
            button->SetId(id);
            button->SetClass("window-control", true);
            button->SetClass(id, true);
            button->SetAttribute("data-tooltip", tooltip);
            auto glyph = impl_->document->CreateElement("span");
            glyph->SetClass("icon", true);
            glyph->SetClass("window-glyph", true);
            glyph->SetInnerRML(icons::utf8(icon_name));
            button->AppendChild(std::move(glyph));
            title_bar.buttons[index] = title_bar.controls->AppendChild(std::move(button));
            UiCommand command;
            command.kind = kind;
            title_bar.commands[id] = std::move(command);
        };
        build_window_control(0, "window-minimize", icons::minus, "Minimize", UiCommandKind::minimize_window);
        build_window_control(1, "window-maximize", icons::box, "Maximize", UiCommandKind::toggle_maximize_window);
        build_window_control(2, "window-close", icons::close, "Close", UiCommandKind::quit);
        title_bar.buttons[2]->SetAttribute("data-tooltip-key", "Alt+F4");
        for (const auto* event : { "click", "input", "change", "dragstart", "dragend", "keydown", "focus", "mouseover", "mouseout", "dblclick" })
            impl_->document->AddEventListener(event, impl_.get());
        // Blur does not bubble, so the document hears a field losing focus in the capture phase.
        impl_->document->AddEventListener("blur", impl_.get(), true);
        impl_->document->Show();
        return true;
    }
    void DocumentBackend::shutdown()
    {
        if (impl_->context)
            Rml::RemoveContext(impl_->context_name);
        if (impl_->initialized && --runtime().references == 0)
        {
            Rml::Shutdown();
            runtime().renderer.geometry.clear();
            runtime().renderer.textures.clear();
        }
        impl_ = std::make_unique<Impl>();
    }
    FocusOwner DocumentBackend::focus_owner() const
    {
        if (!impl_->context)
            return FocusOwner::scene;
        if (impl_->select_box_open())
            return FocusOwner::transient;
        auto* focus = impl_->context->GetFocusElement();
        // RmlUi never clears focus: it starts on the document and a blur hands it to the parent,
        // so a dock, a card or the document often holds it with no control focused.
        if (!focus || !takes_keyboard_focus(*focus))
            return FocusOwner::scene;
        if (focus->IsClassSet("number-field") || focus->IsClassSet("text-field"))
            return FocusOwner::text_field;
        for (auto* element = focus; element; element = element->GetParentNode())
            if (element->IsClassSet("row-checklist") && element->IsClassSet("is-open"))
                return FocusOwner::transient;
        return FocusOwner::keyboard_control;
    }

    EscapeTarget DocumentBackend::escape_target() const
    {
        if (!impl_->active_slider.empty() || impl_->measure_drag)
            return EscapeTarget::text_field;
        if (impl_->keyboard_scope == "panel-confirmation")
            return EscapeTarget::control;
        const auto focus = focus_owner();
        if (focus == FocusOwner::text_field)
        {
            const auto* field = impl_->context->GetFocusElement();
            if (impl_->keyboard_scope.empty() || (field && field->IsClassSet("number-field")))
                return EscapeTarget::text_field;
        }
        if (focus == FocusOwner::transient)
            return EscapeTarget::control;
        return EscapeTarget::none;
    }

    bool DocumentBackend::wants_text_input() const
    {
        return focus_owner() == FocusOwner::text_field;
    }

    Rect DocumentBackend::text_input_area() const
    {
        return runtime().system.keyboard_area;
    }

    CursorShape DocumentBackend::cursor() const
    {
        return runtime().system.cursor_shape;
    }

    bool DocumentBackend::pointer_captured() const
    {
        return impl_->pointer_captured;
    }

    std::optional<bool> DocumentBackend::covers(const Vec2& point) const
    {
        if (!impl_->document)
            return std::nullopt;
        return impl_->over(point);
    }

    bool DocumentBackend::draws_window_controls() const
    {
        return impl_->document != nullptr;
    }

    // Asked by the platform on every pointer move over the window. Outside the title strip the
    // strip's bounds answer; inside it the document says what is topmost.
    std::optional<WindowPart> DocumentBackend::window_part(const Vec2& point) const
    {
        const auto& state = *impl_;
        if (!state.context || !state.title_bar.frame.controls)
            return std::nullopt;
        // An open menu or popover takes the press: on one of its items, or anywhere else to close.
        if (state.title_bar.strip.contains(point) && state.view && !state.view->transients().empty())
            return WindowPart::client;
        return state.classify(point);
    }

    bool DocumentBackend::bare_title_bar(const Vec2& point) const
    {
        return impl_->context && impl_->title_bar.frame.controls && impl_->classify(point) == WindowPart::title_bar;
    }
    std::optional<std::uint64_t> DocumentBackend::hovered_notification() const
    {
        if (!impl_->document)
            return std::nullopt;
        auto* stack = impl_->document->GetElementById("toast-stack");
        if (!stack)
            return std::nullopt;
        for (int index = 0; index < stack->GetNumChildren(); ++index)
        {
            const auto* toast = stack->GetChild(index);
            if (!toast || !toast->IsPseudoClassSet("hover"))
                continue;
            const auto id = std::string_view(toast->GetId());
            if (id.size() < 6 || id.substr(0, 6) != "toast-")
                continue;
            std::uint64_t serial {};
            const auto number = id.substr(6);
            const auto [end, error] = std::from_chars(number.data(), number.data() + number.size(), serial);
            if (error == std::errc {} && end == number.data() + number.size())
                return serial;
        }
        return std::nullopt;
    }

    void DocumentBackend::set_clipboard_hooks(const ClipboardReader& reader, const ClipboardWriter& writer)
    {
        runtime().system.clipboard_reader = reader;
        runtime().system.clipboard_writer = writer;
    }

    void DocumentBackend::reveal(std::string_view key, std::string_view instance)
    {
        if (!impl_->document)
            return;
        const auto found = impl_->commands_by_key.find(std::string(key));
        if (found == impl_->commands_by_key.end() || found->second.empty())
            return;
        // A setting can be shown twice (the Guide repeats the one its lesson changes); the reveal
        // goes to the copy the learner can see, never to one in a surface that just closed. The
        // setting's own home (the Inspector, Show, Preferences) comes before a lesson's copy, so
        // Open World or a "World > ..." search result lands on World, not on the Guide.
        const auto lesson_copy = [](std::string_view row_instance)
        {
            return row_instance.rfind("guide", 0) == 0 || row_instance.rfind("present", 0) == 0;
        };
        for (const auto lesson_pass : { false, true })
            for (const auto& id : impl_->controls)
                if (auto* element = impl_->document->GetElementById(id); element && element->IsVisible(true))
                {
                    auto* row = element;
                    while (row && string_attribute(row, "data-control-key") != key)
                        row = row->GetParentNode();
                    if (!row)
                        continue;
                    const auto row_instance = string_attribute(row, "data-instance");
                    if ((instance.empty() && lesson_copy(row_instance) == lesson_pass) || (!instance.empty() && row_instance == instance))
                    {
                        element->Focus(true);
                        element->ScrollIntoView(false);
                        impl_->focus_keyboard_visible = true;
                        return;
                    }
                }
    }
    void DocumentBackend::focus_field(std::string_view key)
    {
        if (!impl_->document)
            return;
        if (auto* field = impl_->document->GetElementById(element_id("legacy", key) + "--field"); field && field->IsVisible(true))
        {
            field->Focus(true);
            impl_->focus_keyboard_visible = false;
        }
    }

    bool DocumentBackend::handle_event(const UiEvent& event, std::vector<UiCommand>& commands)
    {
        auto& state = *impl_;
        if (!state.context)
            return false;
        const auto mods = (event.modifiers.shift ? Rml::Input::KM_SHIFT : 0) | (event.modifiers.control ? Rml::Input::KM_CTRL : 0) | (event.modifiers.alt ? Rml::Input::KM_ALT : 0);
        bool consumed = false;
        const auto button = event.button == PointerButton::primary ? 0 : event.button == PointerButton::secondary ? 1
                                                                                                                  : 2;
        switch (event.kind)
        {
        case UiEventKind::pointer_move:
        case UiEventKind::pointer_down:
        case UiEventKind::pointer_up:
        case UiEventKind::wheel:
            state.pointer = event.pointer_px;
            if (state.measure_drag || (event.kind == UiEventKind::pointer_down && event.button == PointerButton::primary && state.measure_grip.contains(event.pointer_px)))
            {
                // Dragging the grip sets the drawer height; the layout clamps it to its limits.
                state.measure_drag = event.kind != UiEventKind::pointer_up;
                state.pointer_captured = state.measure_drag;
                if (state.view && event.kind != UiEventKind::wheel)
                    state.view->set_number("measure.height", std::max(0.0, (static_cast<double>(state.viewport.height) - event.pointer_px.y) / std::max(0.01f, state.scale)));
                return true;
            }
            state.context->ProcessMouseMove(static_cast<int>(event.pointer_px.x), static_cast<int>(event.pointer_px.y), mods);
            consumed = state.over(event.pointer_px) || state.pointer_captured;
            if (event.kind == UiEventKind::pointer_down)
                state.pointer_captured = state.over(event.pointer_px);
            if (event.kind == UiEventKind::pointer_down)
            {
                state.tooltip_suppressed = true;
                state.context->ProcessMouseButtonDown(button, mods);
                if (event.button == PointerButton::secondary)
                {
                    auto* row = state.context->GetElementAtPoint({ static_cast<float>(event.pointer_px.x), static_cast<float>(event.pointer_px.y) });
                    while (row && string_attribute(row, "data-control-key").empty())
                        row = row->GetParentNode();
                    if (row)
                    {
                        const auto kind = string_attribute(row, "data-row-kind");
                        if (kind == "readout" || kind == "plot")
                        {
                            UiCommand open;
                            open.id = std::string(string_attribute(row, "data-control-key"));
                            std::string value;
                            if (kind == "readout")
                                if (const auto* value_element = state.document->GetElementById(row->GetId() + "--value"))
                                    value = value_element->GetInnerRML();
                            open.detail = "context-ui:" + std::string(kind == "plot" ? "graph" : "readout") + '\x1f' + value;
                            state.pending.push_back(std::move(open));
                            consumed = true;
                        }
                    }
                }
                if (event.button == PointerButton::primary)
                {
                    for (auto& [id, binding] : state.bindings)
                        if (binding.scrub)
                            if (auto* label = state.document->GetElementById(id); label && bounds(label).contains(event.pointer_px))
                            {
                                state.active_slider = id;
                                state.drag_start_x = event.pointer_px.x;
                                state.drag_start_value = binding.model_value;
                                state.drag_moved = false;
                                state.pointer_captured = true;
                                consumed = true;
                                break;
                            }
                    state.focus_keyboard_visible = false;
                    if (!state.over(event.pointer_px))
                        if (auto* focused = state.context->GetFocusElement())
                            focused->Blur();
                }
            }
            if (event.kind == UiEventKind::pointer_move && !state.active_slider.empty())
                if (auto binding = state.bindings.find(state.active_slider); binding != state.bindings.end() && binding->second.scrub && binding->second.spec)
                {
                    const auto logical_delta = (event.pointer_px.x - state.drag_start_x) / std::max(0.01f, state.scale);
                    const auto steps = static_cast<int>(logical_delta / 4.0);
                    if (steps != 0 || state.drag_moved)
                    {
                        const auto& number = binding->second.spec->number;
                        double value = state.drag_start_value;
                        if (number.scale == NumberScale::logarithmic)
                        {
                            const auto factor = event.modifiers.shift ? number.coarse : event.modifiers.alt ? number.fine
                                                                                                            : number.step;
                            value *= std::pow(factor, steps);
                        }
                        else
                        {
                            const auto multiplier = event.modifiers.shift ? number.coarse : event.modifiers.alt ? number.fine
                                                                                                                : 1.0;
                            value += steps * number.step * multiplier;
                        }
                        value = std::clamp(value, number.minimum, number.maximum);
                        if (number.decimals >= 0)
                        {
                            const auto precision = std::pow(10.0, number.decimals);
                            value = std::round(value * precision) / precision;
                        }
                        binding->second.model_value = value;
                        state.drag_moved = true;
                        if (!state.preview_sent_this_frame)
                        {
                            state.pending.push_back(Impl::number_command(binding->second, value, UiEditPhase::preview));
                            state.preview_sent_this_frame = true;
                        }
                    }
                }
            if (event.kind == UiEventKind::pointer_up)
            {
                state.context->ProcessMouseButtonUp(button, mods);
                if (event.button == PointerButton::primary && !state.active_slider.empty())
                {
                    if (const auto binding = state.bindings.find(state.active_slider); binding != state.bindings.end())
                    {
                        if (!binding->second.scrub || state.drag_moved)
                        {
                            state.pending.push_back(Impl::number_command(binding->second, binding->second.model_value, UiEditPhase::commit));
                        }
                        else
                        {
                            auto field_id = state.active_slider;
                            if (const auto suffix = field_id.rfind("--label"); suffix != std::string::npos)
                                field_id.replace(suffix, std::string::npos, "--field");
                            if (auto* field = state.document->GetElementById(field_id))
                                field->Focus(true);
                        }
                    }
                    state.active_slider.clear();
                    state.drag_moved = false;
                }
                if (event.button == PointerButton::primary && state.pointer_activation_queued)
                {
                    if (auto* focused = state.context->GetFocusElement())
                        focused->Blur();
                    state.pointer_activation_queued = false;
                    state.focus_keyboard_visible = false;
                }
            }
            if (event.kind == UiEventKind::pointer_up && (event.button == PointerButton::primary || state.active_slider.empty()))
                state.pointer_captured = false;
            if (event.kind == UiEventKind::wheel)
                state.context->ProcessMouseWheel(static_cast<float>(-event.wheel_delta), mods);
            break;
        case UiEventKind::key_down:
            if ((event.key == UiKey::escape || event.key == UiKey::f12) && (!state.active_slider.empty() || state.measure_drag))
            {
                state.cancel_pointer_gesture();
                if (event.key == UiKey::escape)
                {
                    consumed = true;
                    break;
                }
            }
            if (event.repeat && (event.key == UiKey::tab || event.key == UiKey::enter || event.key == UiKey::space))
                break;
            if ((event.key == UiKey::tab || event.key == UiKey::f6) && !state.controls.empty())
            {
                const auto controls = state.keyboard_controls();
                auto* next = static_cast<Rml::Element*>(nullptr);
                if (event.key == UiKey::f6 && state.keyboard_scope.empty())
                {
                    const char* docks[] = { "region-command-bar", "region-guide", "region-inspector", "region-measure" };
                    constexpr int scene = 4;
                    auto current = scene;
                    if (auto* focus = state.context->GetFocusElement(); focus && takes_keyboard_focus(*focus))
                        for (auto* ancestor = focus; ancestor; ancestor = ancestor->GetParentNode())
                            for (int i = 0; i < scene; ++i)
                                if (ancestor->GetId() == docks[i])
                                    current = i;
                    const auto direction = event.modifiers.shift ? -1 : 1;
                    for (int attempt = 1; attempt <= scene + 1 && !next; ++attempt)
                    {
                        const auto region = (current + direction * attempt + 2 * (scene + 1)) % (scene + 1);
                        if (region == scene)
                        {
                            if (auto* focus = state.context->GetFocusElement())
                                focus->Blur();
                            state.focus_keyboard_visible = false;
                            break;
                        }
                        for (const auto& id : controls)
                        {
                            if (auto* candidate = state.document->GetElementById(id))
                                for (auto* ancestor = candidate; ancestor; ancestor = ancestor->GetParentNode())
                                    if (ancestor->GetId() == docks[region])
                                    {
                                        next = candidate;
                                        break;
                                    }
                            if (next)
                                break;
                        }
                    }
                }
                else if (!controls.empty())
                {
                    const auto* focus = state.context->GetFocusElement();
                    const auto found = std::find(controls.begin(), controls.end(), focus ? focus->GetId() : "");
                    auto index = found == controls.end() ? (event.modifiers.shift ? 0 : -1) : static_cast<int>(found - controls.begin());
                    index = (index + (event.modifiers.shift ? -1 : 1) + static_cast<int>(controls.size())) % static_cast<int>(controls.size());
                    next = state.document->GetElementById(controls[static_cast<std::size_t>(index)]);
                }
                if (next)
                {
                    state.focus_keyboard_visible = true;
                    next->Focus(true);
                    next->ScrollIntoView(false);
                }
                consumed = true;
            }
            else if (event.key == UiKey::enter || (event.key == UiKey::space && focus_owner() == FocusOwner::keyboard_control && !event.modifiers.control && !event.modifiers.alt))
            {
                auto* focus = state.context->GetFocusElement();
                const auto local = state.view_bindings.find(focus ? focus->GetId() : "");
                if (local != state.view_bindings.end() && local->second.kind == Impl::ViewBindingKind::text && local->second.key == "search.query")
                {
                    const auto results = state.search_results();
                    if (!results.empty())
                        consumed = state.queue_command(std::find(results.begin(), results.end(), state.search_highlight) != results.end() ? state.search_highlight : results.front());
                    if (consumed)
                        break;
                }
                const auto binding = state.bindings.find(focus ? focus->GetId() : "");
                const auto alternate = state.alternate_commands.find(focus ? focus->GetId() : "");
                if (focus && alternate != state.alternate_commands.end())
                {
                    state.pending.push_back(alternate->second);
                    consumed = true;
                }
                else if (focus && state.activate_view_binding(focus->GetId()))
                {
                    consumed = true;
                }
                else if (focus && binding != state.bindings.end() && !binding->second.slider && binding->second.spec && binding->second.spec->kind == ControlKind::number)
                {
                    auto* field = dynamic_cast<Rml::ElementFormControl*>(focus);
                    if (binding->second.mixed && field && field->GetValue().find_first_not_of(" \t") == Rml::String::npos)
                    {
                        consumed = true;
                        break;
                    }
                    const auto parsed = field ? Impl::parse_field(binding->second, field->GetValue()) : std::optional<double> {};
                    const auto valid = parsed && *parsed >= binding->second.spec->number.minimum && *parsed <= binding->second.spec->number.maximum;
                    state.number_error(focus, binding->second, !valid);
                    if (valid && field->GetValue() != Impl::field_text(binding->second) && std::abs(*parsed - binding->second.model_value) > 1.0e-12)
                    {
                        state.pending.push_back(Impl::number_command(binding->second, *parsed, UiEditPhase::commit));
                    }
                    if (valid)
                        state.synchronize_number_field(focus, binding->second, field->GetValue() == Impl::field_text(binding->second) ? binding->second.model_value : *parsed);
                    consumed = true;
                }
                else if (state.focus_keyboard_visible)
                {
                    if (state.queue_command(focus ? focus->GetId() : ""))
                    {
                        consumed = true;
                    }
                    else
                        consumed = !state.context->ProcessKeyDown(rml_key(event.key), mods);
                }
                else
                    consumed = !state.context->ProcessKeyDown(rml_key(event.key), mods);
            }
            else if (event.key == UiKey::escape)
            {
                if (state.cancel_confirmation())
                {
                    consumed = true;
                    break;
                }
                for (const auto& id : state.selects)
                    if (auto* select = dynamic_cast<Rml::ElementFormControlSelect*>(state.document->GetElementById(id)); select && select->IsSelectBoxVisible())
                    {
                        select->HideSelectBox();
                        consumed = true;
                    }
                if (consumed)
                    break;
                if (auto* focus = state.context->GetFocusElement())
                {
                    for (auto* ancestor = focus; ancestor; ancestor = ancestor->GetParentNode())
                        if (ancestor->IsClassSet("row-checklist") && ancestor->IsClassSet("is-open"))
                        {
                            ancestor->SetClass("is-open", false);
                            consumed = true;
                            break;
                        }
                    if (consumed)
                        break;
                    const auto binding = state.bindings.find(focus->GetId());
                    if (binding != state.bindings.end() && !binding->second.slider && binding->second.spec && binding->second.spec->kind == ControlKind::number)
                    {
                        if (auto* field = dynamic_cast<Rml::ElementFormControl*>(focus))
                        {
                            state.synchronizing_control = true;
                            field->SetValue(Impl::field_text(binding->second));
                            state.synchronizing_control = false;
                        }
                        auto command = binding->second.command;
                        command.phase = UiEditPhase::cancel;
                        state.pending.push_back(std::move(command));
                        if (binding->second.spec->kind == ControlKind::number)
                            state.number_error(focus, binding->second, false);
                    }
                    focus->Blur();
                    consumed = true;
                }
            }
            else if (event.key == UiKey::arrow_down || event.key == UiKey::arrow_up)
            {
                if (auto* focused = state.context->GetFocusElement())
                {
                    const auto local = state.view_bindings.find(focused->GetId());
                    if (local != state.view_bindings.end() && local->second.kind == Impl::ViewBindingKind::text && local->second.key == "search.query" && !event.modifiers.control && !event.modifiers.alt)
                    {
                        const auto results = state.search_results();
                        if (!results.empty())
                        {
                            const auto found = std::find(results.begin(), results.end(), state.search_highlight);
                            const auto count = static_cast<int>(results.size());
                            const auto current = found == results.end() ? (event.key == UiKey::arrow_down ? -1 : 0) : static_cast<int>(found - results.begin());
                            const auto index = (current + (event.key == UiKey::arrow_down ? 1 : -1) + count) % count;
                            state.highlight_search_result(results[static_cast<std::size_t>(index)]);
                            state.document->GetElementById(state.search_highlight)->ScrollIntoView(false);
                        }
                        consumed = true;
                        break;
                    }
                    auto* row = focused;
                    while (row && string_attribute(row, "data-row-kind").empty())
                        row = row->GetParentNode();
                    const auto row_kind = string_attribute(row, "data-row-kind");
                    if (row && row_kind == "stepper")
                    {
                        const auto id = row->GetId() + (event.key == UiKey::arrow_up ? "--plus" : "--minus");
                        if (const auto command = state.commands.find(id); command != state.commands.end())
                            state.pending.push_back(command->second);
                        consumed = true;
                        break;
                    }
                    if (row_kind == "radio_list")
                    {
                        auto* options = focused->GetParentNode();
                        auto current = -1;
                        if (options)
                            for (int i = 0; i < options->GetNumChildren(); ++i)
                                if (options->GetChild(i) == focused)
                                    current = i;
                        if (options && current >= 0 && options->GetNumChildren() > 0)
                        {
                            const auto next = std::clamp(current + (event.key == UiKey::arrow_up ? -1 : 1), 0, options->GetNumChildren() - 1);
                            options->GetChild(next)->Focus(true);
                            consumed = true;
                            break;
                        }
                    }
                    const auto binding = state.bindings.find(focused->GetId());
                    if (binding != state.bindings.end() && binding->second.mixed)
                        consumed = true;
                    else if (binding != state.bindings.end() && !binding->second.slider && binding->second.spec && binding->second.spec->kind == ControlKind::number)
                    {
                        const auto& spec = binding->second.spec->number;
                        auto value = binding->second.model_value;
                        if (auto* field = dynamic_cast<Rml::ElementFormControl*>(focused); field && field->GetValue() != Impl::field_text(binding->second))
                        {
                            const auto parsed = Impl::parse_field(binding->second, field->GetValue());
                            const auto valid = parsed && *parsed >= spec.minimum && *parsed <= spec.maximum;
                            state.number_error(focused, binding->second, !valid);
                            if (!valid)
                            {
                                consumed = true;
                                break;
                            }
                            value = *parsed;
                        }
                        const auto direction = event.key == UiKey::arrow_up ? 1.0 : -1.0;
                        const auto factor = event.modifiers.shift ? spec.coarse : event.modifiers.alt ? spec.fine
                                                                                                      : spec.step;
                        value = spec.scale == NumberScale::logarithmic ? value * (direction > 0 ? factor : 1.0 / factor) : value + direction * spec.step * (event.modifiers.shift ? spec.coarse : event.modifiers.alt ? spec.fine
                                                                                                                                                                                                                      : 1.0);
                        value = std::clamp(value, spec.minimum, spec.maximum);
                        state.pending.push_back(Impl::number_command(binding->second, value, UiEditPhase::commit));
                        // The focused field is preserved during model refreshes. Keep it in sync
                        // with keyboard steps so a later blur cannot restore its old value, and
                        // repeated presses in the same frame build on the preceding adjustment.
                        state.synchronize_number_field(focused, binding->second, value);
                        consumed = true;
                    }
                    else if (dynamic_cast<Rml::ElementFormControl*>(focused))
                        consumed = !state.context->ProcessKeyDown(rml_key(event.key), mods);
                    else if (auto* container = focused->GetClosestScrollableContainer())
                    {
                        container->SetScrollTop(container->GetScrollTop() + (event.key == UiKey::arrow_down ? 80.0f : -80.0f) * state.scale);
                        consumed = true;
                    }
                }
            }
            else if (event.key == UiKey::arrow_left || event.key == UiKey::arrow_right)
            {
                if (auto* focused = state.context->GetFocusElement())
                {
                    auto* row = focused;
                    while (row && string_attribute(row, "data-row-kind").empty())
                        row = row->GetParentNode();
                    const auto row_kind = string_attribute(row, "data-row-kind");
                    if (row && row_kind == "stepper")
                    {
                        const auto id = row->GetId() + (event.key == UiKey::arrow_right ? "--plus" : "--minus");
                        if (const auto command = state.commands.find(id); command != state.commands.end())
                            state.pending.push_back(command->second);
                        consumed = true;
                    }
                    else if (row_kind == "segmented" || row_kind == "tabs")
                    {
                        auto* options = focused->GetParentNode();
                        auto current = -1;
                        if (options)
                            for (int i = 0; i < options->GetNumChildren(); ++i)
                                if (options->GetChild(i) == focused)
                                    current = i;
                        if (options && current >= 0 && options->GetNumChildren() > 0)
                        {
                            const auto count = options->GetNumChildren();
                            const auto next = (current + (event.key == UiKey::arrow_right ? 1 : count - 1)) % count;
                            auto* next_element = options->GetChild(next);
                            next_element->Focus(true);
                            if (!state.activate_view_binding(next_element->GetId()))
                                if (const auto command = state.commands.find(next_element->GetId()); command != state.commands.end())
                                    state.pending.push_back(command->second);
                            consumed = true;
                        }
                    }
                }
                if (!consumed)
                    consumed = !state.context->ProcessKeyDown(rml_key(event.key), mods);
            }
            else if (event.key != UiKey::f1 && event.key != UiKey::f12)
                consumed = !state.context->ProcessKeyDown(rml_key(event.key), mods);
            break;
        case UiEventKind::key_up:
            if (event.key != UiKey::tab && event.key != UiKey::f6 && event.key != UiKey::f1 && event.key != UiKey::f12)
                consumed = !state.context->ProcessKeyUp(rml_key(event.key), mods);
            break;
        case UiEventKind::text_input:
            consumed = !state.context->ProcessTextInput(event.text);
            break;
        case UiEventKind::viewport_resized:
            state.context->SetDimensions({ event.viewport_width, event.viewport_height });
            break;
        case UiEventKind::focus_gained:
            // Regaining focus needs nothing from the document; the next input event resumes it.
            break;
        case UiEventKind::pointer_leave:
            // Nothing stays hovered once the pointer is on the title bar or out of the window; a
            // drag the document has captured carries on until its release.
            state.context->ProcessMouseLeave();
            break;
        case UiEventKind::focus_lost:
            state.pending.clear();
            state.cancel_pointer_gesture();
            state.cancelling_pointer_edit = true;
            state.context->ProcessMouseLeave();
            for (int released = 0; released < 3; ++released)
                state.context->ProcessMouseButtonUp(released, 0);
            if (auto* focused = state.context->GetFocusElement())
                focused->Blur();
            state.cancelling_pointer_edit = false;
            state.pointer_captured = false;
            break;
        }
        state.context->Update();
        state.pending.erase(std::remove_if(state.pending.begin(), state.pending.end(), [](const UiCommand& command)
                                {
                                    if (command.detail.rfind("copy-value:", 0) != 0)
                                        return false;
                                    runtime().system.SetClipboardText(command.detail.substr(11));
                                    return true;
                                }),
            state.pending.end());
        commands.insert(commands.end(), state.pending.begin(), state.pending.end());
        state.pending.clear();
        return consumed;
    }

    void DocumentBackend::release_focus(std::vector<UiCommand>& commands)
    {
        auto& state = *impl_;
        if (!state.context)
            return;
        if (auto* focused = state.context->GetFocusElement(); focused && takes_keyboard_focus(*focused))
            focused->Blur();
        state.focus_keyboard_visible = false;
        commands.insert(commands.end(), state.pending.begin(), state.pending.end());
        state.pending.clear();
    }

    void DocumentBackend::take_pending_commands(std::vector<UiCommand>& commands)
    {
        commands.insert(commands.end(), impl_->pending.begin(), impl_->pending.end());
        impl_->pending.clear();
    }

    void DocumentBackend::build(const UiFrameContext& frame, DrawList& list)
    {
        list.clear();
        auto& state = *impl_;
        if (!state.document || !frame.model || !frame.device || !frame.theme)
            return;
        if (!state.layout_initialized || state.viewport.width != frame.viewport.width || state.viewport.height != frame.viewport.height)
            state.context->SetDimensions({ frame.viewport.width, frame.viewport.height });
        if (!state.layout_initialized || state.scale != frame.scale)
        {
            state.context->SetDensityIndependentPixelRatio(frame.scale);
            // Small text shrinks less than body text: below 95 % of a reference pixel per dp the
            // caps labels, chips and hints step up (playground.rcss) to stay near 10 px.
            state.document->SetClass("text-floor-1", frame.scale < 0.95f && frame.scale >= 0.85f);
            state.document->SetClass("text-floor-2", frame.scale < 0.85f);
        }
        state.viewport = frame.viewport;
        state.scale = frame.scale;
        const auto computed = frame.layout ? *frame.layout : compute_layout({ frame.viewport, frame.scale });
        const auto region_bounds = [&](RegionId id)
        {
            const auto* region = computed.find(id);
            return region ? region->bounds : Rect {};
        };
        const auto compact = computed.mode == LayoutMode::narrow;
        const auto light_theme = frame.model->theme_id == "workbench_light";
        const auto projector_theme = frame.model->theme_id == "workbench_projector";
        if (!state.layout_initialized || state.light_theme != light_theme || state.projector_theme != projector_theme)
        {
            state.context->ActivateTheme("workbench_dark", !light_theme && !projector_theme);
            state.context->ActivateTheme("workbench_light", light_theme);
            state.context->ActivateTheme("workbench_projector", projector_theme);
            state.light_theme = light_theme;
            state.projector_theme = projector_theme;
        }
        if (!state.layout_initialized || state.compact != compact)
        {
            state.document->SetClass("compact", compact);
            state.compact = compact;
        }
        // A short projector window (or large text) gets the narrow window's lesson card, so the
        // card leaves the demonstration most of the height.
        constexpr double short_present_height = 560.0;
        const auto short_present = frame.view && frame.view->present().mode && computed.logical_height < short_present_height;
        if (!state.layout_initialized || state.short_present != short_present)
        {
            state.document->SetClass("present-short", short_present);
            state.short_present = short_present;
        }
        if (!state.layout_initialized || state.reduce_motion != frame.model->reduce_motion)
        {
            // Document-side animations and transitions are gated through this class so Reduce
            // Motion covers the interface as well as the scene effects.
            state.document->SetClass("reduce-motion", frame.model->reduce_motion);
            state.reduce_motion = frame.model->reduce_motion;
        }
        state.layout_initialized = true;
        const auto* focus = state.context->GetFocusElement();
        auto focus_id = focus ? focus->GetId() : "";
        std::string focus_group;
        for (auto* ancestor = focus; ancestor && focus_group.empty(); ancestor = ancestor->GetParentNode())
            focus_group = string_attribute(ancestor, "data-group");
        auto* command_bar = state.document->GetElementById("region-command-bar");
        auto* draw_bar = state.document->GetElementById("region-draw-bar");
        auto* present_strip = state.document->GetElementById("region-present-strip");
        auto* present_caption = state.document->GetElementById("region-present-caption");
        auto* guide = state.document->GetElementById("region-guide");
        auto* guide_rail = state.document->GetElementById("region-guide-rail");
        auto* stage = state.document->GetElementById("region-stage");
        auto* banner = state.document->GetElementById("region-banner");
        auto* inspector = state.document->GetElementById("region-inspector");
        auto* inspector_rail = state.document->GetElementById("region-inspector-rail");
        auto* measure = state.document->GetElementById("region-measure");
        auto* status_line = state.document->GetElementById("region-status-line");
        auto* hint = state.document->GetElementById("region-hint");
        auto* scrim = state.document->GetElementById("region-scrim");
        auto* sheets = state.document->GetElementById("region-sheets");
        auto* popovers = state.document->GetElementById("region-popovers");
        auto* toast_stack = state.document->GetElementById("toast-stack");
        const auto set_display = [&](Rml::Element* element, bool visible)
        {
            if (!element)
                return;
            if (!visible && !state.active_slider.empty() && contains_element(element, state.document->GetElementById(state.active_slider)))
                state.cancel_pointer_gesture();
            const auto id = element->GetId();
            const auto found = state.displays.find(id);
            if (found == state.displays.end() || found->second != visible)
            {
                // Showing removes the override so the stylesheet's display (flex strips, blocks)
                // applies; only hiding is forced inline.
                if (visible)
                    element->RemoveProperty("display");
                else
                    element->SetProperty("display", "none");
                state.displays[id] = visible;
            }
        };
        const auto place_cached = [&](Rml::Element* element, const Rect& bounds)
        {
            if (!element)
                return;
            const auto id = element->GetId();
            const auto found = state.placements.find(id);
            const auto unchanged = found != state.placements.end() && found->second.minimum.x == bounds.minimum.x && found->second.minimum.y == bounds.minimum.y && found->second.maximum.x == bounds.maximum.x && found->second.maximum.y == bounds.maximum.y;
            if (!unchanged)
            {
                place(element, bounds);
                state.placements[id] = bounds;
            }
        };
        const auto place_region = [&](Rml::Element* element, RegionId id)
        {
            const auto bounds = region_bounds(id);
            place_cached(element, bounds);
            set_display(element, bounds.width() > 0.0 && bounds.height() > 0.0);
            // A narrow window presents side surfaces as sheets over the stage and status line.
            const auto* region = computed.find(id);
            element->SetClass("is-sheet", region && region->presentation == RegionPresentation::sheet);
        };
        place_region(command_bar, RegionId::command_bar);
        place_region(draw_bar, RegionId::draw_bar);
        place_region(present_strip, RegionId::present_strip);
        // The window's state changes without the panels changing (a maximize, a focus change, the
        // pointer over a caption button), so the title bar is brought up to date every frame.
        place_region(state.title_bar.surface, RegionId::title_bar);
        place_region(state.title_bar.brand, RegionId::window_brand);
        place_region(state.title_bar.controls, RegionId::window_controls);
        const auto window_frame_changed = state.apply_window_frame(frame.window_frame, computed);
        // A collapsed side never squeezes the full panel into its 28 dp column; the dedicated rail
        // affordance takes the rail (or the narrow-mode ribbon) and the panel dock hides.
        place_region(guide, RegionId::guide_panel);
        if (computed.find(RegionId::guide_rail))
            place_region(guide_rail, RegionId::guide_rail);
        else
            place_region(guide_rail, RegionId::guide_ribbon);
        const auto ribbon = !computed.find(RegionId::guide_rail) && computed.find(RegionId::guide_ribbon) != nullptr;
        guide_rail->SetClass("rail-horizontal", ribbon);
        // The narrow window's ribbon has the width to say which experiment the Guide is for; the
        // command bar has usually shed the title by then.
        if (auto* rail_button = state.document->GetElementById("rail-guide"); rail_button && rail_button->GetNumChildren() > 1)
        {
            const auto& experiment = frame.model->scenario_content && !frame.model->scenario_content->title.empty() ? frame.model->scenario_content->title : frame.model->scenario_title;
            // It also says when the Guide is waiting for the prediction asked before the first run.
            const auto waiting = ribbon && prediction_pending(*frame.model) ? std::string { " \xC2\xB7 Predict first" } : std::string {};
            state.set_text(rail_button->GetChild(1), ribbon && !experiment.empty() ? "Guide \xC2\xB7 " + experiment + waiting : std::string { "Guide" });
        }
        place_region(stage, RegionId::stage);
        place_region(banner, RegionId::banner);
        place_region(inspector, RegionId::inspector);
        place_region(inspector_rail, RegionId::inspector_rail);
        place_region(measure, RegionId::measure_drawer);
        // The Measure drawer, docked or as a sheet, is resized from a grip on its top edge.
        if (auto* grip = state.document->GetElementById("measure-grip"))
        {
            const auto* drawer = computed.find(RegionId::measure_drawer);
            const auto resizable = drawer && (drawer->presentation == RegionPresentation::docked || drawer->presentation == RegionPresentation::sheet);
            state.measure_grip = resizable ? Rect { { drawer->bounds.minimum.x, drawer->bounds.minimum.y - 4.0 * frame.scale }, { drawer->bounds.maximum.x, drawer->bounds.minimum.y + 6.0 * frame.scale } } : Rect {};
            place_cached(grip, state.measure_grip);
            set_display(grip, resizable);
            grip->SetClass("is-sheet", resizable && drawer->presentation == RegionPresentation::sheet);
            grip->SetClass("is-dragging", state.measure_drag);
        }
        // The Graph tab may use the drawer's full width; the other tabs keep a readable measure.
        {
            static constexpr std::string_view graph_tab[] { "graph" };
            measure->SetClass("is-graph", frame.view && frame.view->active_tab("measure.header.tabs", graph_tab, "energy") == "graph");
        }
        place_region(status_line, RegionId::status_line);
        const auto panel_has_content = [&](const Panel* panel)
        {
            if (!panel || !panel->is_visible())
                return false;
            if (panel->id() == "command_search")
                return frame.view && frame.view->sheet_open("command_search");
            if (panel->id() == "context_menu")
                return frame.model->context_menu_request.has_value() && frame.view && frame.view->transient_open("context_menu");
            if (panel->id() == "hover_card")
                return frame.model->hover && !frame.model->hover->card_lines.empty();
            if (panel->id() == "add_menu")
                return frame.view && frame.view->transient_open("add_menu");
            if (panel->id() == "draw_options")
                return frame.view && frame.view->transient_open("draw_options");
            if (panel->id() == "playback_speed")
                return frame.view && frame.view->transient_open("playback_speed");
            if (panel->id() == "present")
                return frame.view && frame.view->present().mode;
            if (panel->id() == "present_caption")
                return frame.view && frame.view->present().mode && computed.find(RegionId::present_caption) != nullptr;
            if (panel->id() == "performance")
                return frame.view && frame.view->surface_open("performance.overlay", false);
            if (panel->id() == "hints")
                return frame.view && frame.view->surface_open("hints.enabled", false) && !frame.view->next_hint().empty();
            if (panel->id() == "confirmation")
                return frame.model->confirmation.has_value();
            if (panel->id() == "banner")
                return frame.model->banner.has_value();
            return true;
        };
        const auto library_visible = std::any_of(frame.panels.begin(), frame.panels.end(), [&](const Panel* panel)
            {
                return panel_has_content(panel) && panel->region() == RegionId::library_sheet;
            });
        const Panel* popover_panel = nullptr;
        for (const auto* panel : frame.panels)
            if (panel_has_content(panel) && panel->region() == RegionId::show_popover)
                popover_panel = panel;
        const auto popover_visible = popover_panel != nullptr;
        const auto modal_visible = std::any_of(frame.panels.begin(), frame.panels.end(), [&](const Panel* panel)
            {
                return panel_has_content(panel) && panel->region() == RegionId::modal;
            });
        const auto banner_visible = std::any_of(frame.panels.begin(), frame.panels.end(), [&](const Panel* panel)
            {
                return panel_has_content(panel) && panel->region() == RegionId::banner;
            });
        set_display(banner, banner_visible);
        place_region(sheets, library_visible ? RegionId::library_sheet : RegionId::modal);
        set_display(sheets, library_visible || modal_visible);
        // Sheets and dialogs dim the stage behind them; pressing the dimmed area closes the top one.
        // The library leaves the toolbar undimmed because it opens beneath it.
        const auto scrim_top = library_visible && !modal_visible ? region_bounds(RegionId::library_sheet).minimum.y : 0.0;
        place_cached(scrim, { { 0.0, scrim_top }, { static_cast<double>(frame.viewport.width), static_cast<double>(frame.viewport.height) } });
        set_display(scrim, library_visible || modal_visible);
        const auto hint_visible = std::any_of(frame.panels.begin(), frame.panels.end(), [&](const Panel* panel)
            {
                return panel_has_content(panel) && panel->region() == RegionId::toasts;
            });
        set_display(hint, hint_visible);
        if (hint_visible)
        {
            const auto width = std::min(300.0 * frame.scale, std::max(0.0, computed.stage.width() - 32.0 * frame.scale));
            // The card rests at the stage's lower right, clear of the toolbar's dropdowns and of
            // the subject framed in the focus rectangle.
            hint->SetProperty("left", px(computed.stage.maximum.x - width - 16.0 * frame.scale));
            hint->SetProperty("top", "auto");
            hint->SetProperty("bottom", px(static_cast<double>(frame.viewport.height) - computed.stage.maximum.y + 40.0 * frame.scale));
            hint->SetProperty("width", px(width));
            hint->SetProperty("height", "auto");
        }
        // The lesson caption grows upwards from the foot of its box, so a short step stays low.
        const auto caption_visible = std::any_of(frame.panels.begin(), frame.panels.end(), [&](const Panel* panel)
            {
                return panel_has_content(panel) && panel->region() == RegionId::present_caption;
            });
        set_display(present_caption, caption_visible);
        if (caption_visible)
        {
            const auto box = region_bounds(RegionId::present_caption);
            const Rect anchor { { box.minimum.x, box.maximum.y }, { box.maximum.x, box.maximum.y + box.height() } };
            const auto found = state.placements.find("region-present-caption");
            if (found == state.placements.end() || found->second.minimum.x != anchor.minimum.x || found->second.minimum.y != anchor.minimum.y || found->second.maximum.x != anchor.maximum.x || found->second.maximum.y != anchor.maximum.y)
            {
                present_caption->SetProperty("left", px(box.minimum.x));
                present_caption->SetProperty("width", px(box.width()));
                present_caption->SetProperty("top", "auto");
                present_caption->SetProperty("height", "auto");
                present_caption->SetProperty("max-height", px(box.height()));
                present_caption->SetProperty("bottom", px(static_cast<double>(frame.viewport.height) - box.maximum.y));
                state.placements["region-present-caption"] = anchor;
            }
        }
        // Every popover is contextual: menus open beneath the toolbar button that owns them, the
        // context menu at the pointer, and the hover card beside its target. Placement runs after
        // the strips are fitted and laid out, because a toolbar button may have moved or folded
        // into the menu, and the container height follows the content.
        set_display(popovers, popover_visible);
        state.hover_card_px.reset();
        const auto place_popover = [&]()
        {
            if (!popover_panel)
                return;
            const auto margin = 8.0 * frame.scale;
            const auto gap = 6.0 * frame.scale;
            // A popover never rises into the window's own title bar, whose controls draw above it.
            const auto* title_strip = computed.find(RegionId::title_bar);
            const auto top_floor = (title_strip ? title_strip->bounds.maximum.y : 0.0) + margin;
            const auto viewport_right = static_cast<double>(frame.viewport.width) - margin;
            const auto viewport_bottom = static_cast<double>(frame.viewport.height) - margin;
            const auto id = popover_panel->id();
            const auto anchor_for = [&](std::string_view key) -> std::optional<Rect>
            {
                auto* element = state.document->GetElementById(element_id("legacy", key));
                if (!element || element->IsClassSet("is-overflowed"))
                    return std::nullopt;
                const auto rect = bounds(element);
                if (rect.width() <= 0.0 || rect.height() <= 0.0)
                    return std::nullopt;
                return rect;
            };
            double width = 260.0 * frame.scale;
            // A hover card shrinks to its few lines rather than holding a menu's width.
            bool fit_content = false;
            Rect anchor { { viewport_right, computed.stage.minimum.y }, { viewport_right, computed.stage.minimum.y } };
            enum class Placement
            {
                below,
                pointer,
                beside
            } placement = Placement::below;
            if (id == "context_menu" && frame.model->context_menu_request)
            {
                const auto point = frame.model->context_menu_request->screen_position_px;
                anchor = { point, point };
                width = 240.0 * frame.scale;
                placement = Placement::pointer;
            }
            else if (id == "hover_card" && frame.model->hover)
            {
                const auto& target = frame.model->hover->screen_bounds;
                anchor = { { target.left, target.top }, { target.left + target.width, target.top + target.height } };
                width = 264.0 * frame.scale;
                placement = Placement::beside;
                fit_content = true;
            }
            else
            {
                const auto key = id == "main_menu" ? std::string_view { "view.menu" } : id == "add_menu" ? std::string_view { "view.add" }
                    : id == "draw_options"                                                               ? std::string_view { "draw.options.open" }
                    : id == "playback_speed"                                                             ? std::string_view { "bar.speed.choice" }
                                                                                                         : std::string_view { "view.show" };
                width = (id == "visualization" || id == "playback_speed" ? 320.0 : id == "draw_options" ? 340.0
                                : id == "main_menu"                                                     ? 280.0
                                                                                                        : 240.0) *
                    frame.scale;
                if (const auto found = anchor_for(key))
                    anchor = *found;
                else if (const auto menu = anchor_for("view.menu"))
                    anchor = *menu;
            }
            const auto place_at = [&](double left, double top, bool measuring)
            {
                left = std::clamp(left, margin, std::max(margin, viewport_right - width));
                top = std::clamp(top, top_floor, std::max(top_floor, viewport_bottom - 40.0 * frame.scale));
                popovers->SetProperty("left", px(left));
                popovers->SetProperty("top", px(top));
                if (fit_content)
                {
                    popovers->SetProperty("width", "auto");
                    popovers->SetProperty("min-width", px(140.0 * frame.scale));
                    popovers->SetProperty("max-width", px(width));
                }
                else
                {
                    popovers->SetProperty("width", px(width));
                    popovers->RemoveProperty("min-width");
                    popovers->RemoveProperty("max-width");
                }
                popovers->SetProperty("height", "auto");
                popovers->SetProperty("max-height", px(std::max(40.0 * frame.scale, viewport_bottom - (measuring ? top_floor : top))));
            };
            const auto initial_left = placement == Placement::beside ? anchor.maximum.x + gap * 2.0
                : placement == Placement::pointer                    ? anchor.minimum.x + 2.0 * frame.scale
                : anchor.minimum.x + width > viewport_right          ? anchor.maximum.x - width
                                                                     : anchor.minimum.x;
            const auto initial_top = placement == Placement::beside ? anchor.minimum.y
                : placement == Placement::pointer                   ? anchor.minimum.y + 2.0 * frame.scale
                                                                    : anchor.maximum.y + gap;
            // The first pass measures the content limited only by the window height; limiting it to
            // the room below the anchor would hide the overflow that decides the flip.
            place_at(initial_left, initial_top, true);
            state.context->Update();
            // A second pass flips a pointer menu upward or a hover card to the target's left when
            // the measured content would otherwise leave the window, and limits the final height
            // to the room left below the popover.
            const auto measured = bounds(popovers);
            if (fit_content)
                width = measured.width();
            auto left = initial_left, top = initial_top;
            if (placement == Placement::pointer && measured.maximum.y > viewport_bottom)
                top = std::max(top_floor, anchor.minimum.y - measured.height() - 2.0 * frame.scale);
            if (placement == Placement::beside)
            {
                // A hover card tries each side of its target in turn and keeps the first that
                // hides the least: its own target most of all, then the other objects, then
                // anything off the stage.
                const auto height = measured.height();
                const auto overlap = [](const Rect& first, const Rect& second)
                {
                    const auto across = std::min(first.maximum.x, second.maximum.x) - std::max(first.minimum.x, second.minimum.x);
                    const auto down = std::min(first.maximum.y, second.maximum.y) - std::max(first.minimum.y, second.minimum.y);
                    return across > 0.0 && down > 0.0 ? across * down : 0.0;
                };
                const std::array<Vec2, 4> corners { Vec2 { anchor.maximum.x + gap * 2.0, anchor.minimum.y }, Vec2 { anchor.minimum.x - width - gap * 2.0, anchor.minimum.y }, Vec2 { anchor.minimum.x, anchor.maximum.y + gap * 2.0 }, Vec2 { anchor.minimum.x, anchor.minimum.y - height - gap * 2.0 } };
                auto best = std::numeric_limits<double>::infinity();
                for (const auto& corner : corners)
                {
                    const auto x = std::clamp(corner.x, margin, std::max(margin, viewport_right - width));
                    const auto y = std::clamp(corner.y, top_floor, std::max(top_floor, viewport_bottom - height));
                    const Rect card { { x, y }, { x + width, y + height } };
                    auto cost = 4.0 * overlap(card, anchor) + (width * height - overlap(card, computed.stage));
                    for (const auto& item : frame.model->hover->avoid)
                        cost += overlap(card, { { item.left, item.top }, { item.left + item.width, item.top + item.height } });
                    if (cost < best - 0.5)
                    {
                        best = cost;
                        left = x;
                        top = y;
                    }
                }
            }
            if (left != initial_left || top != initial_top || measured.maximum.y > viewport_bottom)
            {
                place_at(left, top, false);
                state.context->Update();
            }
            if (id == "hover_card")
                state.hover_card_px = bounds(popovers);
            state.placements.erase("region-popovers");
        };
        if (toast_stack)
            place_cached(toast_stack, { { computed.stage.minimum.x + 16.0 * frame.scale, std::max(computed.stage.minimum.y, computed.stage.maximum.y - 260.0 * frame.scale) }, { std::min(computed.stage.maximum.x, computed.stage.minimum.x + 336.0 * frame.scale), computed.stage.maximum.y - 44.0 * frame.scale } });
        if (toast_stack)
        {
            std::unordered_set<std::string> wanted_toasts;
            for (const auto& toast : frame.toasts)
            {
                const auto id = "toast-" + std::to_string(toast.serial);
                wanted_toasts.insert(id);
                auto* element = state.document->GetElementById(id);
                if (!element)
                {
                    auto created = state.document->CreateElement("div");
                    created->SetId(id);
                    created->SetClass("toast", true);
                    element = toast_stack->AppendChild(std::move(created));
                    state.append_text(element, "span", id + "--text", "toast-text", toast.text);
                    state.append_text(element, "button", id + "--dismiss", "toast-dismiss", "×");
                }
                element->SetClass("error", toast.severity == Severity::error);
                element->SetClass("warning", toast.severity == Severity::warning);
                element->SetClass("success", toast.severity == Severity::success);
                state.set_text(state.document->GetElementById(id + "--text"), toast.text);
                if (toast.action && !toast.action->label.empty())
                {
                    if (!state.document->GetElementById(id + "--action"))
                        state.append_text(element, "button", id + "--action", "toast-action", toast.action->label);
                    state.set_text(state.document->GetElementById(id + "--action"), toast.action->label);
                }
                else if (auto* action = state.document->GetElementById(id + "--action"))
                    element->RemoveChild(action);
            }
            for (int index = toast_stack->GetNumChildren() - 1; index >= 0; --index)
            {
                auto* child = toast_stack->GetChild(index);
                if (!child || !wanted_toasts.count(child->GetId()))
                    toast_stack->RemoveChild(child);
            }
        }
        std::string panel_signature;
        for (const auto* panel : frame.panels)
            if (panel && panel->is_visible())
                panel_signature.append(panel->id()).push_back('\n');
        std::string toast_signature;
        for (const auto& toast : frame.toasts)
            toast_signature.append(std::to_string(toast.serial)).append(":").append(toast.text).push_back('\n');
        auto view_signature = frame.view ? frame.view->serialize() : std::string {};
        // Direct edits refresh immediately through change_serial. Live read-outs use their normal
        // 10 Hz cadence for ordinary experiments and back off in large diagnostic setups, where
        // rebuilding long object lists must not become part of the scene's hot path.
        const auto body_count = frame.model->world ? frame.model->world->contiguous_bodies().size() : 0;
        const auto refresh_hz = body_count >= 24 ? 0.5 : 10.0;
        const auto refresh_bucket = static_cast<std::int64_t>(std::floor(frame.wall_time_s * refresh_hz));
        const auto cache_large_setup = body_count >= 24;
        // A resized or rescaled window refits its strips at once, even in a large setup.
        const auto content_geometry_changed = state.content_viewport.width != frame.viewport.width || state.content_viewport.height != frame.viewport.height || state.content_scale != frame.scale || state.content_window_controls != state.title_bar.frame.controls;
        const auto rebuild_content = !cache_large_setup || !state.content_initialized || content_geometry_changed || state.content_change_serial != frame.model->change_serial || state.content_refresh_bucket != refresh_bucket || state.content_view_signature != view_signature || state.content_panel_signature != panel_signature || state.content_toast_signature != toast_signature;
        const auto draw_performance_overlay = [&]
        {
            const auto visible = frame.view && frame.view->surface_open("performance.overlay", false) && std::any_of(frame.panels.begin(), frame.panels.end(), [](const auto* panel)
                                                                                                             {
                                                                                                                 return panel && panel->is_visible() && panel->id() == "performance";
                                                                                                             });
            if (!visible)
                return;
            const auto region = region_bounds(RegionId::performance_overlay);
            const auto& performance = frame.model->performance;
            const auto units = frame.model->display_units;
            // The renderer string names the API and the graphics device; each gets its own line.
            const auto device_split = performance.renderer.find(" / ");
            const auto renderer = device_split == std::string::npos ? performance.renderer : performance.renderer.substr(0, device_split);
            const auto graphics = device_split == std::string::npos ? std::string {} : performance.renderer.substr(device_split + 3);
            std::array<std::string, 9> lines {
                "Performance",
                "Frame  " + core::fixed(performance.frame_time_s * 1000.0, 2) + " ms",
                "FPS  " + core::fixed(performance.frames_per_second, 0),
                "Substeps  " + std::to_string(performance.substeps),
                "Renderer  " + renderer,
                graphics.empty() ? std::string {} : "Graphics  " + graphics,
                "Interface  " + performance.interface_backend,
                "Pointer  " + core::substitute("{}, {}\xC2\xA0{}", core::format_value(performance.pointer_world_m.x, core::DisplayQuantity::length, units), core::format_value(performance.pointer_world_m.y, core::DisplayQuantity::length, units), core::display_unit(core::DisplayQuantity::length, units)),
                "View  " + core::format_quantity(performance.view_height_m, core::DisplayQuantity::length, units)
            };
            // Small interface text (12 dp), sized from the font itself so the box always holds it.
            const auto text_scale = frame.scale * 12.0f / 14.0f;
            const auto line_height = static_cast<double>(frame.device->text_line_height(text_scale));
            const auto padding_x = 9.0 * frame.scale, padding_y = 6.0 * frame.scale;
            const auto maximum_text_width = std::max(0.0, std::min(computed.stage.width() - 16.0 * frame.scale, 360.0 * frame.scale) - 2.0 * padding_x);
            double text_width = 0.0;
            std::size_t line_count = 0;
            for (auto& line : lines)
            {
                if (line.empty())
                    continue;
                ++line_count;
                // A line wider than the box is shortened on a character boundary, never drawn past it.
                if (static_cast<double>(frame.device->measure_text_width(line, text_scale)) > maximum_text_width)
                {
                    constexpr std::string_view ellipsis = "\xE2\x80\xA6";
                    while (!line.empty() && static_cast<double>(frame.device->measure_text_width(line + std::string(ellipsis), text_scale)) > maximum_text_width)
                    {
                        line.pop_back();
                        while (!line.empty() && (static_cast<unsigned char>(line.back()) & 0xC0) == 0x80)
                            line.pop_back();
                    }
                    line += ellipsis;
                }
                text_width = std::max(text_width, static_cast<double>(frame.device->measure_text_width(line, text_scale)));
            }
            const auto width = text_width + 2.0 * padding_x;
            const auto height = static_cast<double>(line_count) * line_height + 2.0 * padding_y;
            const Rect box { { region.maximum.x - width, region.minimum.y }, { region.maximum.x, region.minimum.y + height } };
            list.add_rounded_rectangle_fill(box.minimum, box.maximum, 6.0f * frame.scale, frame.theme->panel_background);
            list.add_rounded_rectangle_outline(box.minimum, box.maximum, 6.0f * frame.scale, frame.theme->panel_border, frame.scale);
            auto y = box.minimum.y + padding_y;
            for (std::size_t index = 0; index < lines.size(); ++index)
            {
                if (lines[index].empty())
                    continue;
                list.add_text({ box.minimum.x + padding_x, y }, lines[index], index == 0 ? frame.theme->panel_title : frame.theme->panel_text, text_scale);
                y += line_height;
            }
        };
        // RmlUi keeps focus on a control after its panel stops being displayed (Present, a closed
        // menu or sheet, a collapsed dock); it would go on taking keys from the scene unseen.
        const auto release_hidden_focus = [&]
        {
            if (auto* focused = state.context->GetFocusElement(); focused && takes_keyboard_focus(*focused) && !focused->IsVisible(true))
            {
                focused->Blur();
                state.focus_keyboard_visible = false;
            }
        };
        const auto tooltip_now = frame.wall_time_s > 0.0 ? frame.wall_time_s : runtime().system.GetElapsedTime();
        if (!rebuild_content)
        {
            if (window_frame_changed)
                state.context->Update();
            release_hidden_focus();
            state.update_tooltip(tooltip_now, frame.scale, frame.viewport);
            for (const auto region : state.shadows)
                shadow(list, region, frame.scale);
            runtime().renderer.output = &list;
            state.context->Render();
            runtime().renderer.output = nullptr;
            draw_performance_overlay();
            return;
        }
        // Cached frames keep the complete hit regions measured during the last layout, including
        // floating panels, captions and dialog scrims. Replacing them with just the dock regions
        // would let pointer input pass through those surfaces until the next content refresh.
        state.regions.clear();
        for (const auto& region : computed.regions)
            if (region.presentation != RegionPresentation::hidden && region.id != RegionId::stage && region.id != RegionId::performance_overlay && region.id != RegionId::show_popover && region.id != RegionId::present_caption && (region.id != RegionId::banner || banner_visible) && (region.id != RegionId::library_sheet || library_visible) && (region.id != RegionId::modal || modal_visible))
                state.regions.push_back(region.bounds);
        state.content_initialized = true;
        state.content_change_serial = frame.model->change_serial;
        state.content_refresh_bucket = refresh_bucket;
        state.content_view_signature = std::move(view_signature);
        state.content_panel_signature = std::move(panel_signature);
        state.content_toast_signature = std::move(toast_signature);
        state.content_viewport = frame.viewport;
        state.content_scale = frame.scale;
        state.content_window_controls = state.title_bar.frame.controls;
        state.commands.clear();
        state.alternate_commands.clear();
        state.shift_commands.clear();
        {
            UiCommand expand_guide;
            expand_guide.detail = "view:guide";
            state.commands["rail-guide"] = std::move(expand_guide);
            UiCommand expand_inspector;
            expand_inspector.detail = "view:inspector";
            state.commands["rail-inspector"] = std::move(expand_inspector);
        }
        for (const auto& toast : frame.toasts)
        {
            UiCommand dismiss;
            dismiss.detail = "dismiss_toast:" + std::to_string(toast.serial);
            state.commands["toast-" + std::to_string(toast.serial) + "--dismiss"] = std::move(dismiss);
            if (toast.action)
            {
                if (toast.action->command)
                    state.commands["toast-" + std::to_string(toast.serial) + "--action"] = *toast.action->command;
                else if (!toast.action->reveal_key.empty())
                {
                    UiCommand reveal;
                    reveal.id = toast.action->reveal_instance;
                    reveal.detail = "reveal:" + toast.action->reveal_key;
                    state.commands["toast-" + std::to_string(toast.serial) + "--action"] = std::move(reveal);
                }
            }
        }
        state.commands_by_key.clear();
        state.view_bindings.clear();
        state.controls.clear();
        state.selects.clear();
        state.text.clear();
        state.preview_sent_this_frame = false;
        // UiFrameContext exposes read-only view state to panel construction. Event handling is the
        // owner-authorized mutation boundary, retained here between build and the next input.
        state.view = const_cast<ViewState*>(frame.view);
        std::unordered_set<std::string> wanted_bindings;
        const auto discrete_change = state.has_change_serial && frame.model->change_serial != state.change_serial;
        std::unordered_set<std::string> visible_panels;
        visible_panels.reserve(frame.panels.size());
        for (const auto* panel : frame.panels)
            if (panel_has_content(panel) && panel->id() != "performance")
                visible_panels.insert("panel-" + std::string(panel->id()));
        for (auto* dock : { command_bar, draw_bar, present_strip, present_caption, guide, banner, inspector, measure, status_line, hint, sheets, popovers })
            for (int i = 0; i < dock->GetNumChildren(); ++i)
            {
                auto* child = dock->GetChild(i);
                if (child && child->GetId() != "menu-overflow")
                    set_display(child, visible_panels.find(std::string(child->GetId())) != visible_panels.end());
            }
        DrawList ignored;
        std::vector<Hotspot> ignored_hotspots;
        std::unordered_map<std::string, int> document_row_count;
        std::map<std::string, std::vector<StripRow>> strips;
        for (auto* panel : frame.panels)
        {
            if (!panel_has_content(panel) || panel->id() == "performance")
                continue;
            const auto panel_id = "panel-" + std::string(panel->id());
            auto* parent = panel->region() == RegionId::command_bar                                                                                ? command_bar
                : panel->region() == RegionId::draw_bar                                                                                            ? draw_bar
                : panel->region() == RegionId::present_strip                                                                                       ? present_strip
                : panel->region() == RegionId::present_caption                                                                                     ? present_caption
                : panel->region() == RegionId::guide_panel || panel->region() == RegionId::guide_rail || panel->region() == RegionId::guide_ribbon ? guide
                : panel->region() == RegionId::banner                                                                                              ? banner
                : panel->region() == RegionId::inspector || panel->region() == RegionId::performance_overlay                                       ? inspector
                : panel->region() == RegionId::measure_drawer                                                                                      ? measure
                : panel->region() == RegionId::status_line                                                                                         ? status_line
                : panel->region() == RegionId::toasts                                                                                              ? hint
                : panel->region() == RegionId::show_popover                                                                                        ? popovers
                                                                                                                                                   : sheets;
            auto* card = state.document->GetElementById(panel_id);
            if (!card)
            {
                auto created = state.document->CreateElement("section");
                ++state.structure_changes;
                created->SetId(panel_id);
                created->SetClass("panel", true);
                card = parent->AppendChild(std::move(created));
            }
            else if (card->GetParentNode() != parent)
            {
                ++state.structure_changes;
                parent->AppendChild(card->GetParentNode()->RemoveChild(card));
            }
            set_display(card, true);
            std::string previous_row_group;
            std::vector<StripRow>* strip_rows = nullptr;
            if (panel->region() == RegionId::command_bar || panel->region() == RegionId::status_line || panel->region() == RegionId::draw_bar || panel->region() == RegionId::present_strip)
            {
                strip_rows = &strips[std::string(panel->id())];
                strip_rows->clear();
            }
            std::vector<PanelRow> rows;
            PanelBuilder builder { *frame.theme, *frame.device, frame.scale, { {}, { 1000, 1000000 } }, ignored, ignored_hotspots, frame.view };
            builder.record_rows(rows);
            builder.set_layout(&computed);
            panel->build(*frame.model, builder);
            struct DesiredRow
            {
                const PanelRow* row;
                std::string key, instance, id;
            };
            std::vector<DesiredRow> desired;
            desired.reserve(rows.size());
            for (const auto& row : rows)
            {
                auto key = command_key(row, panel->id());
                auto instance = row.instance;
                if (instance.empty() && !row.command.id.empty() && (row.command.kind == UiCommandKind::load_scenario || row.command.kind == UiCommandKind::set_layer || row.command.kind == UiCommandKind::set_vector_scale))
                    instance = row.command.id;
                auto collision_key = key;
                collision_key.push_back('\x1f');
                collision_key.append(instance);
                const auto duplicate = document_row_count[collision_key]++;
                if (duplicate > 0)
                    instance += (instance.empty() ? "" : "-") + std::to_string(duplicate + 1);
                desired.push_back({ &row, std::move(key), std::move(instance), {} });
                desired.back().id = element_id("legacy", desired.back().key, desired.back().instance);
            }

            std::unordered_set<std::string> wanted;
            for (auto& item : desired)
            {
                wanted.insert(item.id);
                auto* element = state.document->GetElementById(item.id);
                // Row ids are unique within a frame, but a hidden panel keeps the rows it built
                // earlier. An id now claimed by this card makes any element elsewhere stale; it is
                // removed rather than adopted, so ids stay unique and only true children are moved.
                if (element && element->GetParentNode() != card)
                {
                    if (!state.active_slider.empty() && contains_element(element, state.document->GetElementById(state.active_slider)))
                        state.cancel_pointer_gesture();
                    if (auto* owner = element->GetParentNode())
                    {
                        ++state.structure_changes;
                        owner->RemoveChild(element);
                    }
                    element = nullptr;
                }
                const auto expected_kind = std::string(row_kind_name(item.row->kind));
                if (element && string_attribute(element, "data-row-kind") != expected_kind)
                {
                    if (!state.active_slider.empty() && contains_element(element, state.document->GetElementById(state.active_slider)))
                        state.cancel_pointer_gesture();
                    ++state.structure_changes;
                    card->RemoveChild(element);
                    element = nullptr;
                }
                if (!element)
                    state.create_row(card, *item.row, item.id);
            }
            for (int index = card->GetNumChildren() - 1; index >= 0; --index)
                if (auto* child = card->GetChild(index); wanted.find(child->GetId()) == wanted.end())
                {
                    if (!state.active_slider.empty() && contains_element(child, state.document->GetElementById(state.active_slider)))
                        state.cancel_pointer_gesture();
                    ++state.structure_changes;
                    // A row that is gone takes a focused control and any unconfirmed edit with it:
                    // the control must not keep taking keys, and its subject may no longer exist.
                    state.synchronizing_control = true;
                    card->RemoveChild(child);
                    state.synchronizing_control = false;
                }
            for (std::size_t index = 0; index < desired.size(); ++index)
            {
                auto* element = state.document->GetElementById(desired[index].id);
                if (element && element->GetParentNode() == card && static_cast<int>(index) < card->GetNumChildren() && card->GetChild(static_cast<int>(index)) != element && !contains_element(element, state.context->GetFocusElement()))
                {
                    ++state.structure_changes;
                    card->InsertBefore(card->RemoveChild(element), card->GetChild(static_cast<int>(index)));
                }
            }

            for (const auto& item : desired)
            {
                const auto& row = *item.row;
                const auto& row_id = item.id;
                auto* element = state.document->GetElementById(row_id);
                if (!element)
                    continue;
                state.text += row.text + " " + row.value + "\n";
                // For disclosures, switches and checkboxes `selected` means open or on, which they
                // show with their own indicator rather than a selection tint.
                const auto selection_tint = (row.selected && row.kind != PanelRowKind::section && row.kind != PanelRowKind::switch_control && row.kind != PanelRowKind::checkbox) || (item.key == "search.result" && row_id == state.search_highlight);
                element->SetClass("selected", selection_tint);
                element->SetAttribute("data-control-key", item.key);
                element->SetAttribute("data-instance", item.instance);
                element->SetClass("is-selected", selection_tint);
                element->SetClass("is-live", row.live);
                element->SetClass("is-disabled", !row.disabled_reason.empty());
                if (!row.disabled_reason.empty())
                    element->SetAttribute("aria-disabled", "true");
                else
                    element->RemoveAttribute("aria-disabled");
                element->SetClass("tone-positive", row.tone == RowTone::positive);
                element->SetClass("tone-normal", row.tone == RowTone::normal);
                element->SetClass("tone-info", row.tone == RowTone::info);
                element->SetClass("tone-warning", row.tone == RowTone::warning);
                element->SetClass("tone-danger", row.tone == RowTone::danger);
                element->SetClass("tone-muted", row.tone == RowTone::muted);
                if (!row.disabled_reason.empty())
                    element->SetAttribute("title", row.disabled_reason);
                else if (!row.tooltip.empty())
                    element->SetAttribute("title", row.tooltip);
                else
                    element->RemoveAttribute("title");
                if (!row.group.empty())
                    element->SetAttribute("data-group", row.group);
                else
                    element->RemoveAttribute("data-group");
                if (row.kind == PanelRowKind::spacer)
                    element->SetProperty("height", std::to_string(row.amount) + "dp");

                const auto& presentation = row.presentation;
                element->SetClass("has-icon", !presentation.icon.empty());
                element->SetClass("icon-only", presentation.icon_only);
                element->SetClass("is-primary", presentation.emphasis == RowEmphasis::primary);
                element->SetClass("is-quiet", presentation.emphasis == RowEmphasis::quiet);
                element->SetClass("is-danger", presentation.emphasis == RowEmphasis::danger);
                element->SetClass("header-action", presentation.header);
                element->SetClass("icon-after", presentation.icon_after);
                element->SetClass("has-badge", !presentation.badge.empty());
                const auto group_class = row.group.empty() ? std::string {} : "group-" + slug(row.group);
                const auto previous_group_class = std::string(string_attribute(element, "data-group-class"));
                if (previous_group_class != group_class)
                {
                    if (!previous_group_class.empty())
                        element->SetClass(previous_group_class, false);
                    if (!group_class.empty())
                        element->SetClass(group_class, true);
                    element->SetAttribute("data-group-class", group_class);
                }
                element->SetClass("group-start", !row.group.empty() && row.group != previous_row_group);
                previous_row_group = row.group;
                // Tooltips carry the full name and shortcut for icon-only and shortened controls;
                // an unavailable control explains why instead.
                const auto described_badge = !presentation.badge.empty() && !presentation.badge_description.empty();
                const auto badge_name = described_badge ? row.text + " · " + presentation.badge_description : std::string {};
                const auto& accessible_name = described_badge ? badge_name : row.text;
                const auto fitted_title = row.kind == PanelRowKind::title && strip_rows ? state.fitted_titles.find(row_id) : state.fitted_titles.end();
                const auto shortened_title = fitted_title != state.fitted_titles.end() && fitted_title->second.full == row.text && fitted_title->second.shown != row.text;
                const auto tooltip = !row.disabled_reason.empty() ? row.disabled_reason
                    : !row.tooltip.empty()                        ? row.tooltip
                    : shortened_title                             ? row.text
                    : (!presentation.icon.empty() || !presentation.shortcut.empty()) && is_interactive(row.kind) && row.kind != PanelRowKind::text_field
                    ? accessible_name
                    : std::string {};
                if (!tooltip.empty())
                {
                    element->SetAttribute("data-tooltip", tooltip);
                    element->SetAttribute("data-tooltip-key", row.disabled_reason.empty() ? presentation.shortcut : std::string {});
                }
                else
                {
                    element->RemoveAttribute("data-tooltip");
                    element->RemoveAttribute("data-tooltip-key");
                }
                if (strip_rows)
                    strip_rows->push_back({ element, row, row_id, item.key });

                const auto now = frame.wall_time_s > 0.0 ? frame.wall_time_s : runtime().system.GetElapsedTime();
                // Live read-outs refresh together at the start of each tenth of a second, so the
                // clock, the Runs duration and the energy figures always show one moment.
                const auto can_write_live = !row.live || std::floor(now * 10.0) != std::floor(state.text_written_at[row_id] * 10.0) || state.contents.find(row_id) == state.contents.end();
                const auto content_signature = row.text + "\n" + row.value + "\n" + row.selected_option + "\n" + std::to_string(row.number_si) + "\n" + (row.selected ? "1" : "0");
                if (!row.live && discrete_change && state.contents.find(row_id) != state.contents.end() && state.contents[row_id] != content_signature)
                    state.value_changed_at[row_id] = now;
                state.contents[row_id] = content_signature;
                if (row.kind == PanelRowKind::readout)
                {
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.text);
                    state.set_text(state.document->GetElementById(row_id + "--icon"), icons::utf8(presentation.icon));
                    if (auto* value = state.document->GetElementById(row_id + "--value"); value && can_write_live)
                    {
                        state.set_text(value, row.value);
                        state.text_written_at[row_id] = now;
                    }
                }
                else if (row.kind == PanelRowKind::meter)
                {
                    const auto fraction = math::clamp(row.amount, 0.0, 1.0);
                    const auto& value = row.value;
                    const auto percentage = core::format_quantity(fraction * 100.0, core::DisplayQuantity::percentage, core::DisplayUnits::si);
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.text);
                    // A live meter is written together with the live read-outs beside it.
                    if (can_write_live)
                    {
                        if (element->GetNumChildren() > 1)
                            state.set_text(element->GetChild(1), value);
                        if (element->GetNumChildren() > 2)
                        {
                            auto* meter = element->GetChild(2);
                            meter->SetAttribute("title", percentage);
                            for (std::uint8_t series = 1; series <= 3; ++series)
                                meter->SetClass("series-" + std::to_string(series), row.series == series);
                            if (meter->GetNumChildren() > 0)
                                meter->GetChild(0)->SetProperty("width", std::to_string(fraction * 100.0) + "%");
                        }
                        if (row.live)
                            state.text_written_at[row_id] = now;
                    }
                    state.text.append(value).append(" ").append(percentage).append("\n");
                }
                else if (row.kind == PanelRowKind::plot)
                {
                    auto* plot_element = state.document->GetElementById(row_id + "--plot");
                    if (row.plot)
                    {
                        // RmlUi renders after panel construction, so retain a deep copy rather
                        // than pointing the custom element at a panel-local PlotData or buffers
                        // that may be rebuilt before the render pass.
                        auto& stored = state.plot_storage[row_id];
                        stored.data = *row.plot;
                        stored.times.clear();
                        stored.values.clear();
                        stored.times.reserve(row.plot->series.size());
                        stored.values.reserve(row.plot->series.size());
                        for (std::size_t index = 0; index < row.plot->series.size(); ++index)
                        {
                            const auto& source = row.plot->series[index];
                            if (source.count > 0 && source.times && source.values)
                            {
                                stored.times.emplace_back(source.times, source.times + source.count);
                                stored.values.emplace_back(source.values, source.values + source.count);
                            }
                            else
                            {
                                stored.times.emplace_back();
                                stored.values.emplace_back();
                            }
                            stored.data.series[index].times = stored.times.back().data();
                            stored.data.series[index].values = stored.values.back().data();
                        }
                        set_plot_element_data(plot_element, &stored.data);
                        const auto summary = plot_summary(stored.data);
                        if (plot_element && plot_element->GetAttribute<Rml::String>("aria-label", Rml::String {}) != summary)
                            plot_element->SetAttribute("aria-label", summary);
                    }
                    else
                        set_plot_element_data(plot_element, nullptr);
                }
                else if (row.kind == PanelRowKind::number && row.spec)
                {
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.text);
                    const auto field_id = row_id + "--field";
                    auto* field = dynamic_cast<Rml::ElementFormControl*>(state.document->GetElementById(field_id));
                    const auto unit = row.mixed ? std::string(core::display_unit(row.spec->number.quantity, frame.model->display_units)) : core::format_unit(row.number_si, row.spec->number.quantity, frame.model->display_units, row.spec->number.decimals);
                    auto field_binding = Impl::Binding { row.command, row.spec, item.key, row.number_si, frame.model->display_units, false, false, unit };
                    field_binding.mixed = row.mixed;
                    field_binding.document_id = frame.model->scenario_id;
                    field_binding.edit_document = frame.model->edit_document;
                    if (item.key.rfind("object.", 0) == 0)
                    {
                        field_binding.selection = frame.model->selected_bodies;
                        if (item.key.rfind("object.shape.", 0) == 0)
                            field_binding.document_id += ":part:" + std::to_string(frame.model->authored_part_index);
                    }
                    if (item.key.rfind("draw.", 0) == 0)
                    {
                        field_binding.selection = frame.model->selected_bodies;
                        field_binding.document_id += ":draft:" + frame.model->shape_edit_target + ":part:" + std::to_string(frame.model->authored_part_index);
                    }
                    bool preserve_draft = false;
                    if (const auto previous = state.bindings.find(field_id); field && previous != state.bindings.end())
                    {
                        const auto same_subject = Impl::same_number_subject(previous->second, field_binding);
                        if ((!same_subject || !row.disabled_reason.empty()) && state.context->GetFocusElement() == field)
                        {
                            // Reusing a row for a different object must never submit the old
                            // draft to the new selection when it later loses focus.
                            state.synchronizing_control = true;
                            field->Blur();
                            state.synchronizing_control = false;
                            if (focus_id == field_id)
                                focus_id.clear();
                            state.number_error(field, previous->second, false);
                        }
                        if ((!same_subject || !row.disabled_reason.empty() || row.mixed) && (state.active_slider == row_id + "--slider" || state.active_slider == row_id + "--label"))
                            state.cancel_pointer_gesture();
                        preserve_draft = same_subject && state.context->GetFocusElement() == field && field->GetValue() != Impl::field_text(previous->second);
                        if (preserve_draft)
                        {
                            // Keep the displayed unit and precise edit baseline together. A
                            // live value or preference change must not reinterpret bare digits.
                            const auto current_command = field_binding.command;
                            field_binding = previous->second;
                            field_binding.command = current_command;
                        }
                    }
                    const auto formatted = Impl::field_text(field_binding);
                    if (field && !preserve_draft && state.active_slider != row_id + "--slider" && field->GetValue() != formatted)
                    {
                        state.synchronizing_control = true;
                        field->SetValue(formatted);
                        state.synchronizing_control = false;
                    }
                    element->SetClass("is-editing", state.context->GetFocusElement() == field);
                    element->SetClass("is-mixed", row.mixed);
                    if (field)
                        field->SetAttribute("aria-label", row.mixed ? row.text + ", mixed" : row.text);
                    if (element->GetNumChildren() > 1)
                    {
                        auto* line = element->GetChild(1);
                        if (line->GetNumChildren() > 1)
                            state.set_text(line->GetChild(1), Impl::sign_in_field(*row.spec) ? std::string {} : field_binding.unit);
                        if (line->GetNumChildren() > 2)
                            state.set_text(line->GetChild(2), state.context->GetFocusElement() == field ? "editing" : "");
                        if (line->GetNumChildren() > 3)
                            state.set_text(line->GetChild(3), row.mixed ? row.value : std::string {});
                    }
                    if (row.disabled_reason.empty())
                    {
                        state.bindings[field_id] = std::move(field_binding);
                        wanted_bindings.insert(field_id);
                        if (!row.mixed)
                        {
                            const auto label_id = row_id + "--label";
                            state.bindings[label_id] = { row.command, row.spec, item.key, row.number_si, frame.model->display_units, false, true, unit };
                            wanted_bindings.insert(label_id);
                        }
                        state.commands_by_key[item.key].push_back(row.command);
                    }
                    state.controls.push_back(field_id);
                    const auto slider_id = row_id + "--slider";
                    if (auto* slider = dynamic_cast<Rml::ElementFormControl*>(state.document->GetElementById(slider_id)))
                    {
                        const auto& number = row.spec->number;
                        const auto clamped = std::clamp(row.number_si, number.soft_minimum, number.soft_maximum);
                        const auto slider_position = number.scale == NumberScale::logarithmic ? std::log(clamped / number.soft_minimum) / std::log(number.soft_maximum / number.soft_minimum) : (clamped - number.soft_minimum) / (number.soft_maximum - number.soft_minimum);
                        if (state.active_slider != slider_id)
                        {
                            state.synchronizing_control = true;
                            slider->SetValue(std::to_string(std::clamp(slider_position, 0.0, 1.0) * 1000.0));
                            state.synchronizing_control = false;
                        }
                        if (row.disabled_reason.empty() && !row.mixed)
                        {
                            state.bindings[slider_id] = { row.command, row.spec, item.key, row.number_si, frame.model->display_units, true };
                            wanted_bindings.insert(slider_id);
                        }
                    }
                }
                else if (row.kind == PanelRowKind::select && row.spec)
                {
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.text);
                    auto* select = dynamic_cast<Rml::ElementFormControlSelect*>(state.document->GetElementById(row_id + "--field"));
                    // The select widget moves appended options out of its DOM children on the
                    // next update, so they are counted through the widget.
                    if (select && select->GetNumOptions() == 0)
                        for (const auto& option : row.spec->options)
                        {
                            auto* child = state.append_text(select, "option", {}, {}, option.label);
                            child->SetAttribute("value", std::string(option.id));
                            if (!option.secondary.empty())
                                child->SetAttribute("title", std::string(option.secondary));
                        }
                    // A mixed selection, or a value that is none of the options, is shown as one
                    // extra entry that cannot be chosen, never as the first option.
                    const auto extra_value = row.mixed ? std::string("--mixed") : row.value.empty() ? std::string {}
                                                                                                    : row.selected_option;
                    const auto selected_value = row.mixed ? extra_value : row.selected_option;
                    if (select)
                    {
                        int extra_index = -1;
                        for (int index = 0; index < select->GetNumOptions(); ++index)
                            if (auto* option = select->GetOption(index); option && option->HasAttribute("data-extra"))
                                extra_index = index;
                        const auto* extra = extra_index >= 0 ? select->GetOption(extra_index) : nullptr;
                        const auto current_extra = extra ? extra->GetAttribute<Rml::String>("value", Rml::String {}) + "\x1f" + extra->GetInnerRML() : std::string {};
                        const auto wanted_extra = extra_value.empty() ? std::string {} : extra_value + "\x1f" + row.value;
                        if (current_extra != wanted_extra)
                        {
                            state.synchronizing_control = true;
                            if (extra_index >= 0)
                                select->Remove(extra_index);
                            if (!extra_value.empty())
                            {
                                select->Add(row.value, extra_value, 0, false);
                                if (auto* added = select->GetOption(0))
                                    added->SetAttribute("data-extra", "true");
                            }
                            state.synchronizing_control = false;
                            ++state.structure_changes;
                        }
                    }
                    element->SetClass("is-mixed", row.mixed);
                    if (select && select->GetValue() != selected_value)
                    {
                        state.synchronizing_control = true;
                        select->SetValue(selected_value);
                        state.synchronizing_control = false;
                    }
                    const auto id = row_id + "--field";
                    if (row.disabled_reason.empty())
                    {
                        state.bindings[id] = { row.command, row.spec, item.key, 0.0, frame.model->display_units, false };
                        wanted_bindings.insert(id);
                        state.commands_by_key[item.key].push_back(row.command);
                    }
                    state.controls.push_back(id);
                    state.selects.push_back(id);
                }
                else if (row.kind == PanelRowKind::checklist && row.spec)
                {
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.text);
                    // The summary names what is chosen, two at most and then a count of the rest;
                    // its accessible name lists every choice.
                    std::string summary_text, full_text;
                    std::size_t named = 0, chosen = 0;
                    for (const auto& selected_id : row.selected_options)
                    {
                        const auto option = std::find_if(row.options.begin(), row.options.end(), [&](const auto& candidate)
                            {
                                return candidate.id == selected_id;
                            });
                        if (option == row.options.end())
                            continue;
                        full_text += (chosen++ == 0 ? "" : ", ") + std::string(option->label);
                        if (named < 2)
                            summary_text += (named++ == 0 ? "" : ", ") + std::string(option->label);
                    }
                    if (chosen > named)
                        summary_text += core::substitute(" +{}", chosen - named);
                    state.set_text(state.document->GetElementById(row_id + "--field-text"), summary_text.empty() ? "Choose…" : summary_text);
                    if (auto* summary = state.document->GetElementById(row_id + "--field"))
                    {
                        const auto accessible = row.text + ": " + (full_text.empty() ? std::string { "nothing chosen" } : full_text);
                        if (summary->GetAttribute<Rml::String>("aria-label", Rml::String {}) != accessible)
                            summary->SetAttribute("aria-label", accessible);
                    }
                    state.checklist_shown[item.key] = row.selected_options;
                    state.view_bindings[row_id + "--field"] = { Impl::ViewBindingKind::checklist_toggle, item.key, {} };
                    state.controls.push_back(row_id + "--field");
                    auto* option_parent = state.document->GetElementById(row_id + "--options");
                    if (option_parent && option_parent->GetNumChildren() == 0)
                        for (const auto& option : row.options)
                            state.append_text(option_parent, "button", row_id + "--option_" + slug(option.id), "checklist-option", option.label)->SetAttribute("tabindex", "0");
                    for (const auto& option : row.options)
                    {
                        const auto option_id = row_id + "--option_" + slug(option.id);
                        state.view_bindings[option_id] = { Impl::ViewBindingKind::checklist_option, item.key, std::string(option.id) };
                        if (auto* option_element = state.document->GetElementById(option_id))
                        {
                            const auto selected = std::find(row.selected_options.begin(), row.selected_options.end(), option.id) != row.selected_options.end();
                            option_element->SetClass("selected", selected);
                            option_element->SetClass("is-selected", selected);
                            option_element->SetAttribute("aria-pressed", selected ? "true" : "false");
                            state.controls.push_back(option_id);
                        }
                    }
                }
                else if (row.kind == PanelRowKind::segmented || row.kind == PanelRowKind::tabs || row.kind == PanelRowKind::radio_list)
                {
                    const auto options = !row.options.empty() ? math::Span<const OptionSpec>(row.options) : row.spec ? row.spec->options
                                                                                                                     : math::Span<const OptionSpec> {};
                    auto* option_parent = state.document->GetElementById(row_id + "--options");
                    if (option_parent && option_parent->GetNumChildren() == 0)
                        for (const auto& option : options)
                        {
                            const auto option_id = row_id + "--option_" + slug(option.id);
                            auto* button = state.append_element(option_parent, "button", option_id, "segment");
                            button->SetAttribute("tabindex", "0");
                            if (row.kind == PanelRowKind::radio_list)
                                state.append_element(button, "span", option_id + "--mark", "radio-mark");
                            auto* icon = state.append_element(button, "span", option_id + "--icon", "icon");
                            state.set_text(icon, icons::utf8(option.icon));
                            button->SetClass("has-icon", !option.icon.empty());
                            state.append_text(button, "span", option_id + "--text", "label", option.label);
                        }
                    for (const auto& option : options)
                    {
                        const auto option_id = row_id + "--option_" + slug(option.id);
                        auto command = row.command;
                        command.id = std::string(option.id);
                        if (row.kind == PanelRowKind::tabs)
                            state.view_bindings[option_id] = { Impl::ViewBindingKind::tab, item.key, std::string(option.id) };
                        else if (row.disabled_reason.empty())
                        {
                            state.commands[option_id] = command;
                            state.commands_by_key[item.key].push_back(command);
                        }
                        if (auto* option_element = state.document->GetElementById(option_id))
                        {
                            const auto selected = option.id == row.selected_option;
                            state.set_text(state.document->GetElementById(option_id + "--text"), option.label);
                            if (!option.secondary.empty())
                                option_element->SetAttribute("data-tooltip", std::string(option.secondary));
                            else if (!option.icon.empty())
                                option_element->SetAttribute("data-tooltip", std::string(option.label));
                            option_element->SetClass("selected", selected);
                            option_element->SetClass("is-selected", selected);
                            option_element->SetAttribute("aria-pressed", selected ? "true" : "false");
                            option_element->SetAttribute("aria-label", std::string(option.label));
                            state.controls.push_back(option_id);
                        }
                    }
                }
                else if (row.kind == PanelRowKind::stepper && row.spec)
                {
                    const auto value = core::format_value(row.number_si, row.spec->number.quantity, frame.model->display_units, row.spec->number.decimals);
                    if (element->GetNumChildren() > 2)
                        state.set_text(element->GetChild(2), value);
                    // Children: label, minus, value, plus, range.
                    if (element->GetNumChildren() > 4)
                        state.set_text(element->GetChild(4), core::substitute("{} to {}", core::format_quantity(row.spec->number.minimum, row.spec->number.quantity, frame.model->display_units), core::format_quantity(row.spec->number.maximum, row.spec->number.quantity, frame.model->display_units)));
                    auto minus = row.command, plus = row.command;
                    minus.value = std::max(row.spec->number.minimum, row.number_si - row.spec->number.step);
                    plus.value = std::min(row.spec->number.maximum, row.number_si + row.spec->number.step);
                    auto* minus_element = state.document->GetElementById(row_id + "--minus");
                    auto* plus_element = state.document->GetElementById(row_id + "--plus");
                    const auto at_minimum = row.number_si <= row.spec->number.minimum;
                    const auto at_maximum = row.number_si >= row.spec->number.maximum;
                    if (minus_element)
                        minus_element->SetAttribute("aria-disabled", at_minimum ? "true" : "false");
                    if (plus_element)
                        plus_element->SetAttribute("aria-disabled", at_maximum ? "true" : "false");
                    if (row.disabled_reason.empty())
                    {
                        if (!at_minimum)
                            state.commands[row_id + "--minus"] = minus;
                        if (!at_maximum)
                            state.commands[row_id + "--plus"] = plus;
                        state.commands_by_key[item.key].push_back(row.command);
                    }
                    state.controls.push_back(row_id + "--minus");
                    state.controls.push_back(row_id + "--plus");
                }
                else if (row.kind == PanelRowKind::notice)
                {
                    auto* actions = state.document->GetElementById(row_id + "--actions");
                    if (actions && actions->GetNumChildren() == 0)
                    {
                        if (row.primary_action)
                            state.append_text(actions, "button", row_id + "--primary", "notice-action", row.primary_action->label);
                        if (row.secondary_action)
                            state.append_text(actions, "button", row_id + "--secondary", "notice-action", row.secondary_action->label);
                    }
                    if (row.primary_action && row.primary_action->command)
                        state.commands[row_id + "--primary"] = *row.primary_action->command;
                    if (row.secondary_action && row.secondary_action->command)
                        state.commands[row_id + "--secondary"] = *row.secondary_action->command;
                }
                else if (row.kind == PanelRowKind::section)
                {
                    state.view_bindings[row_id] = { Impl::ViewBindingKind::section, item.key, row.selected ? "true" : "false" };
                    state.controls.push_back(row_id);
                    element->SetAttribute("aria-expanded", row.selected ? "true" : "false");
                    element->SetClass("is-open", row.selected);
                    state.set_text(state.document->GetElementById(row_id + "--icon"), icons::utf8(row.selected ? icons::caret_down : icons::next_step));
                    state.set_text(state.document->GetElementById(row_id + "--text"), row.text);
                }
                else if (row.kind == PanelRowKind::list_item)
                {
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.list_content ? row.list_content->title : row.text);
                    state.set_text(state.document->GetElementById(row_id + "--secondary"), row.list_content ? row.list_content->secondary : row.hint);
                    std::string badges;
                    if (row.list_content)
                        for (const auto& badge : row.list_content->badges)
                            badges += (badges.empty() ? "" : " · ") + badge;
                    state.set_text(state.document->GetElementById(row_id + "--badges"), badges);
                    element->SetAttribute("aria-selected", row.selected ? "true" : "false");
                    if (row.disabled_reason.empty())
                    {
                        state.commands[row_id] = row.command;
                        state.commands_by_key[item.key].push_back(row.command);
                    }
                    state.controls.push_back(row_id);
                    if (auto* actions = state.document->GetElementById(row_id + "--actions"))
                    {
                        const auto count = row.list_content ? row.list_content->actions.size() : std::size_t { 0 };
                        if (actions->GetNumChildren() != static_cast<int>(count))
                        {
                            while (actions->GetNumChildren() > 0)
                                actions->RemoveChild(actions->GetChild(0));
                            for (std::size_t index = 0; index < count; ++index)
                            {
                                auto* button = state.append_element(actions, "button", row_id + "--action_" + std::to_string(index), "row row-action list-action");
                                button->SetAttribute("tabindex", "0");
                                state.append_element(button, "span", {}, "icon");
                                state.append_element(button, "span", {}, "label");
                            }
                        }
                        element->SetClass("has-actions", count > 0);
                        for (std::size_t index = 0; index < count; ++index)
                        {
                            const auto& action = row.list_content->actions[index];
                            auto* button = actions->GetChild(static_cast<int>(index));
                            state.set_text(button->GetChild(0), icons::utf8(action.icon));
                            state.set_text(button->GetChild(1), action.label);
                            button->SetClass("has-icon", !action.icon.empty());
                            button->SetClass("is-primary", action.primary);
                            button->SetClass("is-quiet", !action.primary);
                            button->SetAttribute("aria-label", action.label);
                            state.commands[button->GetId()] = action.command;
                            state.controls.push_back(button->GetId());
                        }
                    }
                    if (row.alternate_command.kind != UiCommandKind::none)
                        state.alternate_commands[row_id] = row.alternate_command;
                    if (row.shift_command.kind != UiCommandKind::none)
                        state.shift_commands[row_id] = row.shift_command;
                }
                else if (row.kind == PanelRowKind::text_field)
                {
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.text);
                    const auto field_id = row_id + "--field";
                    auto* field = dynamic_cast<Rml::ElementFormControl*>(state.document->GetElementById(field_id));
                    if (field && state.context->GetFocusElement() != field && field->GetValue() != row.value)
                    {
                        state.synchronizing_control = true;
                        field->SetValue(row.value);
                        state.synchronizing_control = false;
                    }
                    state.set_text(state.document->GetElementById(row_id + "--icon"), icons::utf8(presentation.icon));
                    state.set_text(state.document->GetElementById(row_id + "--placeholder"), row.hint);
                    // The placeholder follows the field's own text, which changes as keys arrive
                    // between rebuilds of the view value.
                    element->SetClass("is-empty", field ? field->GetValue().empty() : row.value.empty());
                    if (field)
                        field->SetAttribute("aria-label", row.text);
                    state.view_bindings[field_id] = { Impl::ViewBindingKind::text, row.view_key.empty() ? item.key : row.view_key, {} };
                    state.controls.push_back(field_id);
                }
                else if (row.kind == PanelRowKind::switch_control)
                {
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.text);
                    if (auto* track = state.document->GetElementById(row_id + "--indicator"))
                        track->SetClass("is-on", row.selected);
                    element->SetClass("is-on", row.selected);
                    if (row.disabled_reason.empty())
                    {
                        state.commands[row_id] = row.command;
                        state.commands_by_key[item.key].push_back(row.command);
                    }
                    state.controls.push_back(row_id);
                    element->SetAttribute("role", "switch");
                    element->SetAttribute("aria-label", row.text);
                    element->SetAttribute("aria-checked", row.selected ? "true" : "false");
                }
                else if (row.kind == PanelRowKind::checkbox)
                {
                    state.set_text(state.document->GetElementById(row_id + "--indicator"), row.selected ? icons::utf8(icons::success) : std::string {});
                    element->SetClass("is-on", row.selected);
                    state.set_text(state.document->GetElementById(row_id + "--label"), row.text);
                    if (row.disabled_reason.empty())
                    {
                        state.commands[row_id] = row.command;
                        state.commands_by_key[item.key].push_back(row.command);
                    }
                    state.controls.push_back(row_id);
                    element->SetAttribute("role", "checkbox");
                    element->SetAttribute("aria-label", row.text);
                    element->SetAttribute("aria-checked", row.selected ? "true" : "false");
                }
                else if (is_interactive(row.kind))
                {
                    if (row.disabled_reason.empty())
                    {
                        state.commands[row_id] = row.command;
                        state.commands_by_key[item.key].push_back(row.command);
                    }
                    state.controls.push_back(row_id);
                    element->SetAttribute("aria-label", accessible_name);
                    element->SetAttribute("aria-pressed", row.selected ? "true" : "false");
                    if (row.kind == PanelRowKind::action)
                    {
                        if (auto* icon = state.document->GetElementById(row_id + "--icon"))
                        {
                            state.set_text(icon, icons::utf8(presentation.icon));
                            icon->SetClass("icon-mirror", presentation.icon == icons::sidebar_right);
                        }
                        state.set_text(state.document->GetElementById(row_id + "--text"), row.text);
                        state.set_text(state.document->GetElementById(row_id + "--shortcut"), presentation.shortcut);
                        state.set_text(state.document->GetElementById(row_id + "--badge"), presentation.badge);
                    }
                }
                else if (row.kind != PanelRowKind::readout && row.kind != PanelRowKind::meter && row.kind != PanelRowKind::plot)
                    state.set_text(element, shortened_title ? fitted_title->second.shown : row.text);

                const auto changed = state.value_changed_at.find(row_id);
                element->SetClass("is-flash", !row.live && changed != state.value_changed_at.end() && now - changed->second < 0.6);
            }
        }
        if (!state.active_slider.empty() && wanted_bindings.find(state.active_slider) == wanted_bindings.end())
            state.cancel_pointer_gesture();
        for (auto iterator = state.bindings.begin(); iterator != state.bindings.end();)
            if (wanted_bindings.find(iterator->first) == wanted_bindings.end())
                iterator = state.bindings.erase(iterator);
            else
                ++iterator;
        state.change_serial = frame.model->change_serial;
        state.has_change_serial = true;
        state.context->Update();
        bool title_basis_changed = false;
        for (auto& [panel_id, strip] : strips)
            for (const auto& entry : strip)
                if (entry.row.kind == PanelRowKind::title)
                    title_basis_changed = state.request_title_width(entry.element, entry.id, entry.row.text) || title_basis_changed;
        if (title_basis_changed)
            state.context->Update();

        // Strips shed detail in their declared order until the content fits the region. A fit is
        // reused until the region width, scale, or the strip's labels change.
        for (auto& [panel_id, strip] : strips)
        {
            auto* card = state.document->GetElementById("panel-" + panel_id);
            if (!card || strip.empty())
                continue;
            int maximum_step = 0;
            auto signature = std::to_string(card->GetClientWidth()) + "|" + std::to_string(frame.scale);
            for (const auto& entry : strip)
            {
                maximum_step = std::max({ maximum_step, entry.row.presentation.label_step, entry.row.presentation.shrink_step, entry.row.presentation.overflow_step });
                signature.append(entry.id).push_back('\x1f');
                signature.append(entry.row.text).push_back('\x1f');
                signature.append(entry.row.presentation.badge).push_back('\x1f');
                signature.append(entry.row.live ? std::string {} : entry.row.value).push_back('\n');
            }
            const auto apply = [&, &strip_entries = strip](int step)
            {
                bool first = true;
                for (const auto& entry : strip_entries)
                {
                    const auto& presentation = entry.row.presentation;
                    const auto overflowed = presentation.overflow_step < 0 || (presentation.overflow_step > 0 && step >= presentation.overflow_step);
                    entry.element->SetClass("is-overflowed", overflowed);
                    entry.element->SetClass("label-hidden", presentation.icon_only || (presentation.label_step > 0 && step >= presentation.label_step));
                    entry.element->SetClass("is-shrinkable", presentation.shrink_step > 0 && step >= presentation.shrink_step);
                    // A shed leading group must not leave its gap in front of the first control.
                    entry.element->SetClass("first-visible", first && !overflowed);
                    first = first && overflowed;
                }
            };
            const auto fits = [&]
            {
                return card->GetScrollWidth() <= card->GetClientWidth() + 1.0f;
            };
            if (const auto cached = state.strip_signatures.find(panel_id); cached != state.strip_signatures.end() && cached->second == signature)
            {
                apply(state.strip_steps[panel_id]);
                continue;
            }
            int step = 0;
            apply(step);
            state.context->Update();
            while (!fits() && step < maximum_step)
            {
                apply(++step);
                state.context->Update();
            }
            state.strip_signatures[panel_id] = signature;
            state.strip_steps[panel_id] = step;
        }
        // A title the strip squeezed is shortened to its width; one with room again grows back.
        bool title_text_changed = false;
        for (auto& [panel_id, strip] : strips)
            for (const auto& entry : strip)
                if (entry.row.kind == PanelRowKind::title)
                    title_text_changed = state.fit_title(entry.element, entry.id) || title_text_changed;
        if (title_text_changed)
            state.context->Update();
        // As the window's title bar the strip is dragged anywhere but its controls. Each control
        // keeps its column of the strip, widened a little, at the strip's full height, so a press
        // just above, below or beside a button never moves or maximizes the window.
        state.title_bar.client_columns.clear();
        if (state.title_bar.frame.controls)
            for (const auto& [panel_id, strip] : strips)
            {
                if (panel_id != "command_bar" && panel_id != "present")
                    continue;
                for (const auto& entry : strip)
                {
                    const auto kind = entry.row.kind;
                    // A title shortened to fit shows its full text as a tooltip, so it is hovered
                    // rather than dragged; one shown in full is title bar.
                    const auto shortened_title = kind == PanelRowKind::title && entry.element->HasAttribute("data-tooltip");
                    if (entry.element->IsClassSet("is-overflowed") || (!is_interactive(kind) && kind != PanelRowKind::readout && kind != PanelRowKind::plot && !shortened_title))
                        continue;
                    const auto box = bounds(entry.element);
                    const auto widen = 4.0 * frame.scale;
                    state.title_bar.client_columns.emplace_back(box.minimum.x - widen, box.maximum.x + widen);
                }
            }
        // Rows folded into a menu are not part of the bar's keyboard traversal.
        for (const auto& [panel_id, strip] : strips)
            for (const auto& entry : strip)
                if (entry.element->IsClassSet("is-overflowed"))
                    state.controls.erase(std::remove_if(state.controls.begin(), state.controls.end(), [&](const std::string& control)
                                             {
                                                 return control == entry.id || control.rfind(entry.id + "--", 0) == 0;
                                             }),
                        state.controls.end());

        // Command-bar rows that do not fit, and the low-frequency transport actions that never
        // sit in the bar, appear at the top of the main menu with their icons and shortcuts.
        if (auto overflow_found = strips.find("command_bar"); overflow_found != strips.end())
        {
            const auto menu_open = frame.view && frame.view->transient_open("main_menu") && popover_panel && popover_panel->id() == "main_menu";
            auto* overflow = state.document->GetElementById("menu-overflow");
            if (!overflow)
            {
                auto created = state.document->CreateElement("div");
                created->SetId("menu-overflow");
                created->SetClass("panel", true);
                created->SetClass("menu-overflow", true);
                overflow = popovers->InsertBefore(std::move(created), popovers->GetFirstChild());
                ++state.structure_changes;
            }
            else if (popovers->GetFirstChild() != overflow)
                popovers->InsertBefore(popovers->RemoveChild(overflow), popovers->GetFirstChild());
            struct OverflowItem
            {
                std::string heading, label, icon, shortcut, reason;
                UiCommand command;
                bool selected { false };
            };
            std::vector<OverflowItem> items;
            const auto heading_for = [](std::string_view group)
            {
                return group == "transport" ? "Playback" : group == "tools" ? "Tools"
                    : group == "history"                                    ? "Edit"
                    : group == "workspace"                                  ? "Panels"
                                                                            : "More";
            };
            for (const auto& entry : overflow_found->second)
            {
                if (!entry.element->IsClassSet("is-overflowed") || !is_interactive(entry.row.kind))
                    continue;
                const auto& row = entry.row;
                if ((row.kind == PanelRowKind::segmented || row.kind == PanelRowKind::select) && row.spec)
                {
                    const auto options = !row.options.empty() ? math::Span<const OptionSpec>(row.options) : row.spec->options;
                    for (const auto& option : options)
                    {
                        auto command = row.command;
                        command.id = std::string(option.id);
                        command.phase = UiEditPhase::commit;
                        if (row.kind == PanelRowKind::select)
                            command = choice_command(row.command, row.spec.get(), option.id).value_or(UiCommand {});
                        items.push_back({ std::string(row.spec->label), std::string(option.label), std::string(option.id == row.selected_option ? icons::success : option.icon), {}, row.disabled_reason, command, option.id == row.selected_option });
                    }
                    continue;
                }
                items.push_back({ heading_for(row.group), row.text, row.presentation.icon, row.presentation.shortcut, row.disabled_reason, row.command, row.selected });
            }
            std::string overflow_signature;
            for (const auto& item : items)
                overflow_signature += item.heading + "\x1f" + item.label + "\x1f" + item.icon + "\x1f" + item.shortcut + "\x1f" + item.reason + "\x1f" + (item.selected ? "1" : "0") + "\n";
            if (overflow_signature != state.overflow_signature)
            {
                while (overflow->GetNumChildren() > 0)
                    overflow->RemoveChild(overflow->GetChild(0));
                std::string heading;
                for (std::size_t index = 0; index < items.size(); ++index)
                {
                    const auto& item = items[index];
                    if (item.heading != heading)
                    {
                        heading = item.heading;
                        state.append_text(overflow, "div", {}, "row row-heading", heading);
                    }
                    const auto id = "menu-overflow--" + std::to_string(index);
                    auto* button = state.append_element(overflow, "button", id, "row row-action menu-item");
                    button->SetAttribute("tabindex", "0");
                    button->SetAttribute("aria-label", item.label);
                    button->SetClass("has-icon", !item.icon.empty());
                    button->SetClass("selected", item.selected);
                    button->SetClass("is-selected", item.selected);
                    button->SetClass("is-disabled", !item.reason.empty());
                    if (!item.reason.empty())
                        button->SetAttribute("data-tooltip", item.reason);
                    auto* item_icon = state.append_element(button, "span", {}, "icon");
                    state.set_text(item_icon, icons::utf8(item.icon));
                    item_icon->SetClass("icon-mirror", item.icon == icons::sidebar_right);
                    state.append_text(button, "span", {}, "label", item.label);
                    state.append_text(button, "span", {}, "shortcut", item.shortcut);
                }
                state.overflow_signature = overflow_signature;
            }
            for (std::size_t index = 0; index < items.size(); ++index)
            {
                const auto id = "menu-overflow--" + std::to_string(index);
                if (items[index].reason.empty())
                    state.commands[id] = items[index].command;
                if (menu_open)
                    state.controls.push_back(id);
            }
            set_display(overflow, menu_open && !items.empty());
        }

        place_popover();
        // A card taller than the room above its resting place keeps its top on the stage rather
        // than rising over the command bar.
        if (hint_visible && bounds(hint).minimum.y < computed.stage.minimum.y)
        {
            hint->SetProperty("top", px(computed.stage.minimum.y));
            hint->SetProperty("bottom", "auto");
        }
        state.context->Update();
        // A docked panel reserves its scrollbar's width while it does not scroll (geometry.rcss),
        // so the content keeps one width either way and the choice never flips back.
        bool gutter_changed = false;
        for (auto* dock : { guide, inspector, measure })
        {
            const auto scrolls = dock->GetScrollHeight() > dock->GetClientHeight() + 1.0f;
            if (dock->IsClassSet("gutter") == scrolls)
            {
                dock->SetClass("gutter", !scrolls);
                gutter_changed = true;
            }
        }
        if (gutter_changed)
            state.context->Update();
        state.shadows.clear();
        if (auto* popover_shadow = state.document->GetElementById("popover-shadow"))
        {
            if (popover_visible)
                place_cached(popover_shadow, bounds(popovers));
            set_display(popover_shadow, popover_visible);
        }
        if (popover_visible)
            state.regions.push_back(bounds(popovers));
        if (hint_visible)
        {
            state.regions.push_back(bounds(hint));
            state.shadows.push_back(bounds(hint));
        }
        state.caption_height_px = caption_visible ? bounds(present_caption).height() : 0.0;
        if (caption_visible)
        {
            state.regions.push_back(bounds(present_caption));
            state.shadows.push_back(bounds(present_caption));
        }
        if (banner_visible)
            state.shadows.push_back(bounds(banner));
        if (library_visible || modal_visible)
            for (int index = 0; index < sheets->GetNumChildren(); ++index)
                if (auto* card = sheets->GetChild(index); card && card->IsVisible())
                    state.shadows.push_back(bounds(card));
        if (state.measure_grip.width() > 0.0)
            state.regions.push_back(state.measure_grip);
        if (library_visible || modal_visible)
            state.regions.push_back({ { 0.0, 0.0 }, { static_cast<double>(frame.viewport.width), static_cast<double>(frame.viewport.height) } });
        if (toast_stack && toast_stack->GetNumChildren() > 0)
        {
            for (int index = 0; index < toast_stack->GetNumChildren(); ++index)
            {
                const auto card = bounds(toast_stack->GetChild(index));
                state.regions.push_back(card);
                state.shadows.push_back(card);
            }
        }
        if (!focus_id.empty())
        {
            if (auto* restored = state.document->GetElementById(focus_id))
                restored->Focus(state.focus_keyboard_visible);
            else if (state.focus_keyboard_visible)
            {
                auto* fallback = focus_group.empty() ? nullptr : state.document->GetElementById(element_id("legacy", focus_group));
                if (!fallback && !state.controls.empty())
                    fallback = state.document->GetElementById(state.controls.front());
                if (fallback)
                    fallback->Focus(state.focus_keyboard_visible);
            }
        }
        release_hidden_focus();
        std::string next_scope;
        for (const auto* panel : frame.panels)
            if (panel_has_content(panel) && (panel->region() == RegionId::modal || panel->region() == RegionId::library_sheet))
                next_scope = "panel-" + std::string(panel->id());
        // Sheets render above popovers that remain open beneath them. Their keyboard scope
        // must follow that same order (for example Ctrl+K from the custom speed popover).
        if (next_scope.empty() && popover_panel && popover_panel->id() != "hover_card" && !frame.model->confirmation)
            next_scope = "panel-" + std::string(popover_panel->id());
        if (frame.model->confirmation)
            next_scope = "panel-confirmation";
        state.change_keyboard_scope(std::move(next_scope), focus_id);
        state.update_tooltip(tooltip_now, frame.scale, frame.viewport);
        for (const auto region : state.shadows)
            shadow(list, region, frame.scale);
        runtime().renderer.output = &list;
        state.context->Render();
        runtime().renderer.output = nullptr;
        draw_performance_overlay();
    }
    double DocumentBackend::measured_height(RegionId id) const
    {
        return id == RegionId::present_caption ? impl_->caption_height_px : 0.0;
    }
    std::optional<Rect> DocumentBackend::hover_card_bounds() const
    {
        return impl_->hover_card_px;
    }
    std::string DocumentBackend::document_text() const
    {
        return impl_->text;
    }
    std::optional<Rect> DocumentBackend::element_bounds(std::string_view id) const
    {
        if (impl_->document)
            if (auto* element = impl_->document->GetElementById(std::string(id)))
                return bounds(element);
        return std::nullopt;
    }
    std::string DocumentBackend::focused_element() const
    {
        const auto* focus = impl_->context ? impl_->context->GetFocusElement() : nullptr;
        return focus && std::find(impl_->controls.begin(), impl_->controls.end(), focus->GetId()) != impl_->controls.end() ? focus->GetId() : "";
    }
    std::size_t DocumentBackend::control_count() const
    {
        return impl_->controls.size();
    }
    const std::vector<std::string>& DocumentBackend::control_ids() const
    {
        return impl_->controls;
    }
    bool DocumentBackend::element_visible(std::string_view id) const
    {
        const auto* element = impl_->document ? impl_->document->GetElementById(std::string(id)) : nullptr;
        return element && element->IsVisible(true);
    }
    std::vector<std::string> DocumentBackend::child_ids(std::string_view id) const
    {
        std::vector<std::string> result;
        if (const auto* element = impl_->document ? impl_->document->GetElementById(std::string(id)) : nullptr)
            for (int index = 0; index < element->GetNumChildren(); ++index)
                if (const auto* child = element->GetChild(index); child && !child->GetId().empty())
                    result.push_back(child->GetId());
        return result;
    }
    std::optional<std::string> DocumentBackend::element_attribute(std::string_view id, std::string_view name) const
    {
        if (const auto* element = impl_->document ? impl_->document->GetElementById(std::string(id)) : nullptr)
            if (const auto* value = element->GetAttribute(std::string(name)))
                return value->Get<std::string>();
        return std::nullopt;
    }
    std::string DocumentBackend::element_text(std::string_view id) const
    {
        const auto* element = impl_->document ? impl_->document->GetElementById(std::string(id)) : nullptr;
        return element ? element->GetInnerRML() : std::string {};
    }
    bool DocumentBackend::document_has_class(std::string_view class_name) const
    {
        return impl_->document && impl_->document->IsClassSet(std::string(class_name));
    }
    std::string DocumentBackend::element_at(const Vec2& point) const
    {
        if (!impl_->context)
            return {};
        for (const auto* element = impl_->context->GetElementAtPoint({ static_cast<float>(point.x), static_cast<float>(point.y) }); element; element = element->GetParentNode())
            if (!element->GetId().empty())
                return element->GetId();
        return {};
    }
    std::optional<Color> DocumentBackend::element_color(std::string_view id, std::string_view property) const
    {
        if (impl_->document)
            if (auto* element = impl_->document->GetElementById(std::string(id)))
                if (const auto* value = element->GetProperty(std::string(property)))
                {
                    const auto color = value->Get<Rml::Colourb>();
                    return Color::from_bytes(color.red, color.green, color.blue, color.alpha);
                }
        return std::nullopt;
    }
    std::optional<Color> DocumentBackend::effective_background(std::string_view id) const
    {
        auto* element = impl_->document ? impl_->document->GetElementById(std::string(id)) : nullptr;
        if (!element)
            return std::nullopt;
        std::vector<Rml::Colourb> layers;
        for (auto* ancestor = element; ancestor; ancestor = ancestor->GetParentNode())
            if (const auto* value = ancestor->GetProperty("background-color"))
                layers.push_back(value->Get<Rml::Colourb>());
        double red = 0.0, green = 0.0, blue = 0.0;
        for (auto layer = layers.rbegin(); layer != layers.rend(); ++layer)
        {
            const auto alpha = layer->alpha / 255.0;
            red = red * (1.0 - alpha) + layer->red / 255.0 * alpha;
            green = green * (1.0 - alpha) + layer->green / 255.0 * alpha;
            blue = blue * (1.0 - alpha) + layer->blue / 255.0 * alpha;
        }
        return Color { static_cast<float>(red), static_cast<float>(green), static_cast<float>(blue), 1.0f };
    }
    bool DocumentBackend::element_has_class(std::string_view id, std::string_view class_name) const
    {
        if (!impl_->document)
            return false;
        const auto* element = impl_->document->GetElementById(std::string(id));
        return element && element->IsClassSet(std::string(class_name));
    }

    std::optional<std::string> DocumentBackend::element_for_key(std::string_view host, std::string_view key, std::string_view instance) const
    {
        if (!impl_->document)
            return std::nullopt;
        const auto id = element_id(host, key, instance);
        return impl_->document->GetElementById(id) ? std::optional<std::string> { id } : std::nullopt;
    }

    std::vector<UiCommand> DocumentBackend::commands_for_key(std::string_view key) const
    {
        const auto found = impl_->commands_by_key.find(std::string(key));
        return found == impl_->commands_by_key.end() ? std::vector<UiCommand> {} : found->second;
    }

    std::vector<UiCommand> DocumentBackend::emitted_commands() const
    {
        std::vector<UiCommand> result;
        for (const auto& [key, commands] : impl_->commands_by_key)
        {
            (void)key;
            result.insert(result.end(), commands.begin(), commands.end());
        }
        for (const auto& [id, command] : impl_->commands)
        {
            (void)id;
            result.push_back(command);
        }
        for (const auto& [id, command] : impl_->alternate_commands)
        {
            (void)id;
            result.push_back(command);
        }
        for (const auto& [id, command] : impl_->shift_commands)
        {
            (void)id;
            result.push_back(command);
        }
        if (impl_->title_bar.frame.controls)
            for (const auto& [id, command] : impl_->title_bar.commands)
                if (impl_->title_bar.frame.resizable || id != "window-maximize")
                    result.push_back(command);
        return result;
    }

    std::optional<std::string> DocumentBackend::element_value(std::string_view id) const
    {
        if (!impl_->document)
            return std::nullopt;
        if (auto* element = impl_->document->GetElementById(std::string(id)))
        {
            if (auto* form = dynamic_cast<Rml::ElementFormControl*>(element))
                return form->GetValue();
            if (element->GetNumChildren() > 0)
                if (auto* text = dynamic_cast<Rml::ElementText*>(element->GetChild(0)))
                    return text->GetText();
        }
        return std::nullopt;
    }

    std::size_t DocumentBackend::structure_change_count() const
    {
        return impl_->structure_changes;
    }

    std::size_t DocumentBackend::text_write_count(std::string_view id) const
    {
        const auto found = impl_->text_write_counts.find(std::string(id));
        return found == impl_->text_write_counts.end() ? 0 : found->second;
    }
}
