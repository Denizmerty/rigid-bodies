#pragma once

#include <rigidbodies/app/input_router.hpp>
#include <rigidbodies/app/key_bindings.hpp>
#include <rigidbodies/app/simulation_session.hpp>
#include <rigidbodies/core/application_config.hpp>
#include <rigidbodies/render/sdl_render_device.hpp>
#include <rigidbodies/ui/document_backend.hpp>
#include <rigidbodies/ui/ui_context.hpp>

#include <SDL3/SDL.h>

#include "test_framework.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace rigidbodies::testing
{
    // One window-free software surface serves every viewport a test visits: the interface reads
    // the drawable size from here while text measurement and submission reach the real device.
    class ViewportDevice final : public render::RenderDevice
    {
    public:
        explicit ViewportDevice(render::RenderDevice& inner) : inner_(inner)
        {
        }
        std::string_view backend_name() const override
        {
            return inner_.backend_name();
        }
        render::ViewportSize drawable_size() const override
        {
            return size;
        }
        float display_scale() const override
        {
            return 1.0f;
        }
        void set_vertical_sync(bool) override
        {
        }
        void begin_frame(const render::Color& color) override
        {
            inner_.begin_frame(color);
        }
        void submit(const render::DrawList& list) override
        {
            inner_.submit(list);
        }
        void end_frame() override
        {
            inner_.end_frame();
        }
        float measure_text_width(std::string_view text, float scale) const override
        {
            return inner_.measure_text_width(text, scale);
        }
        float text_line_height(float scale) const override
        {
            return inner_.text_line_height(scale);
        }
        const std::string& last_error() const override
        {
            return inner_.last_error();
        }

        render::ViewportSize size { 1600, 900 };

    private:
        render::RenderDevice& inner_;
    };

    // Checks what a device would dereference when replaying the list and returns the number of
    // meshes inspected.
    inline std::size_t validate_draw_list(const render::DrawList& list)
    {
        std::size_t meshes = 0;
        const auto vertex_count = list.vertices().size();
        const auto text_size = list.text_buffer().size();
        for (const auto& command : list.commands())
        {
            RIGIDBODIES_EXPECT(command.vertex_offset + command.vertex_count <= vertex_count, "every command's points lie inside the shared vertex buffer");
            if (command.kind == render::DrawCommandKind::text)
                RIGIDBODIES_EXPECT(command.text_offset + command.text_length <= text_size, "every text command lies inside the shared text buffer");
            if (command.kind != render::DrawCommandKind::indexed_mesh && command.kind != render::DrawCommandKind::instanced_mesh)
                continue;
            RIGIDBODIES_EXPECT(command.mesh != nullptr, "every mesh command carries geometry");
            const auto& mesh = *command.mesh;
            RIGIDBODIES_EXPECT(mesh.indices.size() % 3 == 0, "mesh indices form whole triangles");
            const auto mesh_vertices = mesh.vertices.size();
            for (const auto index : mesh.indices)
                RIGIDBODIES_EXPECT(index >= 0 && static_cast<std::size_t>(index) < mesh_vertices, "every mesh index addresses one of its vertices");
            for (const auto& vertex : mesh.vertices)
                RIGIDBODIES_EXPECT(std::isfinite(vertex.position.x) && std::isfinite(vertex.position.y), "every mesh vertex is finite");
            if (mesh.texture)
                RIGIDBODIES_EXPECT(mesh.texture->rgba.size() == static_cast<std::size_t>(mesh.texture->width) * static_cast<std::size_t>(mesh.texture->height) * 4u, "mesh texture pixels match its size");
            if (mesh.clip)
                RIGIDBODIES_EXPECT(mesh.clip->width >= 0 && mesh.clip->height >= 0, "mesh clips have no negative extent");
            ++meshes;
        }
        return meshes;
    }

    // Drives the real learner interface (UiContext on the RmlUi document backend) and a real
    // SimulationSession the way Application does: platform-neutral events pass through
    // route_event, interface commands drain into the session, and each frame advances, renders
    // and rebuilds the document. File dialogs complete at once against in-memory content.
    class LearnerInterface
    {
    public:
        explicit LearnerInterface(const core::ApplicationConfig& config = {}, render::ViewportSize size = { 1600, 900 }, float scale = 1.0f)
        {
            surface_ = SDL_CreateSurface(2560, 1600, SDL_PIXELFORMAT_RGBA32);
            RIGIDBODIES_EXPECT(surface_ != nullptr, "window-free image surface is available");
            software_ = render::SdlRenderDevice::adopt_software_renderer(SDL_CreateSoftwareRenderer(surface_));
            RIGIDBODIES_EXPECT(software_ != nullptr, "window-free software device is available");
            RIGIDBODIES_EXPECT(software_->load_font(std::filesystem::path(RIGIDBODIES_SOURCE_ASSETS) / "fonts/Inter-Medium.ttf"), "the scene font loads");
            device_ = std::make_unique<ViewportDevice>(*software_);
            interface.set_scale(scale * static_cast<float>(config.interface_settings.interface_scale));
            interface.set_theme(render::theme_by_name(config.interface_settings.theme.c_str()));
            RIGIDBODIES_EXPECT(interface.initialize(ui::UiBackendKind::document, RIGIDBODIES_SOURCE_ASSETS), "the interface starts");
            RIGIDBODIES_EXPECT(interface.backend_name() == "rmlui", "the document backend is in use, not the overlay fallback");
            interface.set_clipboard_hooks([this]
                {
                    return clipboard;
                },
                [this](std::string_view value)
                {
                    clipboard = std::string(value);
                });
            session.configure(config);
            resize(size, scale);
            const auto initial = ui::compute_layout({ size, interface.scale() });
            session.set_focus_rect({ initial.focus.minimum.x, initial.focus.minimum.y, initial.focus.width(), initial.focus.height() });
            session.frame_subject();
            frame();
        }
        LearnerInterface(const LearnerInterface&) = delete;
        LearnerInterface& operator=(const LearnerInterface&) = delete;
        ~LearnerInterface()
        {
            interface.shutdown();
            device_.reset();
            software_.reset();
            if (surface_)
                SDL_DestroySurface(surface_);
        }

        [[nodiscard]] const ui::DocumentBackend& document() const
        {
            const auto* backend = dynamic_cast<const ui::DocumentBackend*>(interface.backend());
            RIGIDBODIES_EXPECT(backend != nullptr, "the live interface is the document backend");
            return *backend;
        }

        [[nodiscard]] render::ViewportSize viewport() const
        {
            return device_->size;
        }

        // The equivalent of a window resize or a move to a monitor with another pixel density.
        void resize(render::ViewportSize size, float scale)
        {
            device_->size = size;
            display_scale = scale;
            ui::UiEvent resized;
            resized.kind = ui::UiEventKind::viewport_resized;
            resized.viewport_width = size.width;
            resized.viewport_height = size.height;
            dispatch(resized);
            session.set_viewport(size);
            session.scene_settings().display_scale = scale;
            interface.set_scale(display_scale * static_cast<float>(session.build_model().ui_scale));
        }

        // One pass of Application::update preceded by the command drain that opens each frame.
        void frame(double frame_time_s = 1.0 / 60.0)
        {
            apply_pending();
            interface.set_scale(display_scale * static_cast<float>(session.build_model().ui_scale));
            const auto focus = interface.layout().focus;
            if (focus.width() > 0.0 && focus.height() > 0.0)
                session.set_focus_rect({ focus.minimum.x, focus.minimum.y, focus.width(), focus.height() });
            const auto& stage = interface.layout().stage;
            auto stage_bottom = stage.maximum.y;
            for (const auto& region : interface.layout().regions)
                if (region.id == ui::RegionId::status_line || (region.id == ui::RegionId::measure_drawer && region.presentation == ui::RegionPresentation::sheet))
                    stage_bottom = std::min(stage_bottom, region.bounds.minimum.y);
            session.set_visible_stage_rect({ stage.minimum.x, stage.minimum.y, stage.width(), std::max(0.0, stage_bottom - stage.minimum.y) });
            session.set_present_spotlight(interface.view_state().present().mode && interface.view_state().present().spotlight);
            if (const auto card = interface.hover_card_bounds())
                session.set_hover_card_area(render::ScreenRect { card->minimum.x, card->minimum.y, card->width(), card->height() });
            else
                session.set_hover_card_area(std::nullopt);
            session.set_frame_time(frame_time_s);
            wall_time_s += frame_time_s > 0.0 ? frame_time_s : 1.0 / 240.0;
            session.set_wall_time(wall_time_s);
            session.advance(frame_time_s);
            session.render(scene_list);
            const auto model = session.build_model();
            interface.set_scale(display_scale * static_cast<float>(model.ui_scale));
            interface.set_theme(session.scene_settings().theme);
            interface.set_window_frame(window_frame);
            interface.build(model, *device_, interface_list, wall_time_s);
            ++frames;
            meshes += validate_draw_list(scene_list);
            meshes += validate_draw_list(interface_list);
            poll_platform_state();
        }

        void frames_for(int count, double frame_time_s = 1.0 / 60.0)
        {
            for (int index = 0; index < count; ++index)
                frame(frame_time_s);
        }

        void present_to_device()
        {
            software_->begin_frame(session.scene_settings().theme.background);
            software_->submit(scene_list);
            software_->submit(interface_list);
            software_->end_frame();
        }

        void apply_pending()
        {
            for (const auto& command : interface.take_commands())
                apply_command(command);
            if (save_continuation_ && !interface.view_state().sheet_open("save_details"))
                save_continuation_.reset();
        }

        // Application::apply_command, with the file dialogs answered from memory.
        void apply_command(const ui::UiCommand& command)
        {
            ++commands;
            using K = ui::UiCommandKind;
            std::string error;
            switch (command.kind)
            {
            case K::toggle_developer_overlay:
            case K::export_still:
            case K::toggle_frame_capture:
            case K::open_captures_folder:
                return;
            case K::minimize_window:
            case K::toggle_maximize_window:
                interface.close_menus();
                window_requests.push_back(command.kind);
                return;
            case K::open_arrangement:
                if (command.flag || command.detail == "cancel")
                    session.apply(command);
                else if (saved_setup.empty())
                    session.notify(ui::Severity::info, "File operation cancelled.", "content");
                else if (!session.request_open_arrangement(saved_setup, error, command.id.empty() ? "memory/setup.rbscenario.json" : command.id))
                    session.notify(ui::Severity::error, error, "content");
                return;
            case K::save_arrangement:
            {
                if (command.detail.empty())
                    save_continuation_ = session.take_pending_departure_for_save();
                const auto model = session.build_model();
                const auto file = session.current_setup_file();
                const auto title = file.title.empty() ? (model.scenario_title.empty() ? std::string { "My setup" } : model.scenario_title) : file.title;
                if (command.detail.empty() && (command.flag || session.current_setup_path().empty()))
                {
                    interface.view_state().set_value("save.title", title + (file.path.empty() ? " (my version)" : " (copy)"));
                    interface.view_state().set_value("save.mode", file.current_moment ? "current" : "starting");
                    interface.view_state().set_value("save.include_guide", file.include_guide ? "true" : "false");
                    interface.request(ui::ViewRequest::open_save_details);
                    return;
                }
                if (!command.detail.empty())
                    interface.request(ui::ViewRequest::close_top_surface);
                const auto snapshot = command.detail.empty() ? session.capture_current_setup_save(error)
                                                             : session.capture_setup_save(error, command.detail, command.value >= 0.5, command.flag);
                if (!snapshot)
                {
                    save_continuation_.reset();
                    session.notify(ui::Severity::error, error, "content");
                }
                else
                {
                    const auto destination = command.detail.empty() ? file.path : "memory/setup.rbscenario.json";
                    if (!session.can_write_setup_save(*snapshot, destination, error))
                    {
                        save_continuation_.reset();
                        session.notify(ui::Severity::error, error, "content");
                        return;
                    }
                    saved_setup = snapshot->text;
                    const auto current = session.complete_setup_save(*snapshot, destination);
                    session.notify(ui::Severity::success, "Setup saved.", "content");
                    const auto continuation = std::move(save_continuation_);
                    save_continuation_.reset();
                    if (current && continuation)
                        session.complete_saved_departure(*continuation);
                }
                return;
            }
            case K::export_shape:
                if (!session.export_shape(saved_shape, error))
                    session.notify(ui::Severity::error, error, "content");
                return;
            case K::import_shape:
                if (saved_shape.empty())
                    session.notify(ui::Severity::info, "File operation cancelled.", "content");
                else if (!session.import_shape(saved_shape, error))
                    session.notify(ui::Severity::error, error, "content");
                return;
            default:
                session.apply(command);
            }
        }

        void handle_action(app::AppAction action)
        {
            using A = app::AppAction;
            switch (action)
            {
            case A::toggle_help:
                interface.request(ui::ViewRequest::open_shortcuts);
                return;
            case A::open_library:
                interface.request(ui::ViewRequest::open_library);
                return;
            case A::toggle_show:
                interface.request(ui::ViewRequest::toggle_show);
                return;
            case A::toggle_measure:
                interface.request(ui::ViewRequest::toggle_measure);
                return;
            case A::open_world:
                interface.request(ui::ViewRequest::open_world);
                return;
            case A::toggle_guide:
                interface.request(ui::ViewRequest::toggle_guide);
                return;
            case A::toggle_inspector_pin:
                interface.request(ui::ViewRequest::toggle_inspector_pin);
                return;
            case A::open_main_menu:
                interface.request(ui::ViewRequest::open_main_menu);
                return;
            case A::open_preferences:
                interface.request(ui::ViewRequest::open_preferences);
                return;
            case A::open_add_menu:
                interface.request(ui::ViewRequest::open_add_menu);
                return;
            case A::toggle_present:
                interface.request(ui::ViewRequest::toggle_present);
                return;
            case A::open_command_search:
                interface.request(ui::ViewRequest::open_command_search);
                return;
            case A::open_context_menu:
                session.request_context_menu_for_selection();
                interface.request(ui::ViewRequest::open_context_menu);
                return;
            case A::previous_guide_step:
            case A::next_guide_step:
            {
                if (!interface.view_state().present().mode)
                    return;
                const auto next = std::max(0.0, interface.view_state().number("present.guide_step", 0.0) + (action == A::next_guide_step ? 1.0 : -1.0));
                interface.view_state().set_number("present.guide_step", next);
                interface.view_state().set_value("present.guide_step", std::to_string(static_cast<int>(next)));
                return;
            }
            case A::previous_object:
            case A::next_object:
            {
                const auto model = session.build_model();
                std::vector<physics::BodyId> objects;
                for (const auto& object : model.objects)
                    if (object.role != "marker")
                        objects.push_back(object.id);
                if (objects.empty())
                    return;
                const auto found = std::find(objects.begin(), objects.end(), model.selection);
                auto index = found == objects.end() ? (action == A::previous_object ? objects.size() - 1 : 0) : static_cast<std::size_t>(found - objects.begin());
                if (found != objects.end())
                    index = action == A::previous_object ? (index + objects.size() - 1) % objects.size() : (index + 1) % objects.size();
                ui::UiCommand select;
                select.kind = ui::UiCommandKind::select_body;
                select.body = objects[index];
                apply_command(select);
                return;
            }
            case A::toggle_developer_overlay:
                return;
            default:
                apply_command(app::command_for_action(action));
            }
        }

        // The per-event body of Application::process_events.
        void dispatch(ui::UiEvent input, std::optional<app::AppAction> bound = std::nullopt)
        {
            const auto model = session.build_model();
            if (input.kind == ui::UiEventKind::focus_lost)
                session.set_window_backgrounded(true);
            else if (input.kind == ui::UiEventKind::focus_gained)
                session.set_window_backgrounded(false);
            input.logical_pixel_scale = display_scale * model.ui_scale;
            app::InputContext context;
            context.focus = interface.focus_owner();
            context.interface_escape = interface.escape_target();
            context.interface_modal = !interface.view_state().sheets().empty() || !interface.view_state().transients().empty() || model.confirmation.has_value();
            context.pointer_over_surface = interface.surface_at(input.pointer_px);
            context.pointer_captured_by_interface = interface.pointer_captured();
            context.gesture_active = session.interaction_active() || session.shape_editor().has_pointer_capture();
            context.draw_active = model.shape_editor_active;
            context.draft_node_selected = model.shape_node_selected;
            context.draft_nodes = model.shape_node_count;
            context.select_tool = session.interaction_mode() == app::InteractionMode::select;
            context.has_selection = !model.selected_bodies.empty();
            context.keyboard_mode = context.focus == ui::FocusOwner::keyboard_control;
            context.open_list = context.focus == ui::FocusOwner::transient;
            context.present = interface.view_state().present().mode;
            context.present_locked = interface.view_state().present().lock;
            context.pick_surface_armed = interface.view_state().pick_surface_armed();
            if (input.kind == ui::UiEventKind::pointer_down && !context.gesture_active && interface.dismiss_menus_at(input.pointer_px))
            {
                apply_pending();
                return;
            }
            if (context.pick_surface_armed && input.kind == ui::UiEventKind::pointer_down && input.button == ui::PointerButton::primary && !context.pointer_over_surface)
            {
                (void)session.pick_surface_at(input.pointer_px);
                interface.view_state().set_pick_surface_armed(false);
                apply_pending();
                return;
            }
            const auto decision = app::route_event(input, bound, context);
            bool consumed = false;
            if (decision.to_interface)
                consumed = interface.handle_event(input);
            else if (decision.to_scene && input.kind == ui::UiEventKind::pointer_down)
                interface.release_focus();
            if (decision.to_scene)
                (void)session.handle_scene_event(input, consumed);
            switch (decision.escape)
            {
            case app::EscapeStep::cancel_gesture:
                session.cancel_gesture();
                interface.view_state().set_pick_surface_armed(false);
                break;
            case app::EscapeStep::draw_deselect_node:
                session.deselect_draft_node();
                break;
            case app::EscapeStep::draw_confirm_discard:
                interface.view_state().set_section_open("draw.discard_prompt", true);
                break;
            case app::EscapeStep::draw_discard:
            {
                ui::UiCommand discard;
                discard.kind = ui::UiCommandKind::cancel_shape_outline;
                apply_command(discard);
                break;
            }
            case app::EscapeStep::tool_to_select:
                handle_action(app::AppAction::select_mode);
                break;
            case app::EscapeStep::clear_selection:
            {
                ui::UiCommand clear;
                clear.kind = ui::UiCommandKind::clear_selection;
                apply_command(clear);
                break;
            }
            case app::EscapeStep::close_transient:
                interface.view_state().set_section_open("draw.discard_prompt", false);
                interface.request(ui::ViewRequest::close_top_surface);
                break;
            case app::EscapeStep::close_sheet:
                interface.request(ui::ViewRequest::close_top_surface);
                break;
            case app::EscapeStep::leave_present:
                interface.request(ui::ViewRequest::toggle_present);
                break;
            default:
                break;
            }
            if (decision.action)
                handle_action(*decision.action);
            poll_platform_state();
            apply_pending();
        }

        void key(ui::UiKey value, ui::KeyModifiers modifiers = {}, bool repeat = false)
        {
            ui::UiEvent press;
            press.kind = ui::UiEventKind::key_down;
            press.key = value;
            press.modifiers = modifiers;
            press.repeat = repeat;
            press.pointer_px = pointer_px_;
            const auto action = repeat ? app::AppAction::none : app::action_for_key(value, modifiers);
            dispatch(press, action == app::AppAction::none ? std::nullopt : std::optional<app::AppAction> { action });
            ui::UiEvent release = press;
            release.kind = ui::UiEventKind::key_up;
            release.repeat = false;
            dispatch(release);
        }

        void text(std::string value)
        {
            ui::UiEvent typed;
            typed.kind = ui::UiEventKind::text_input;
            typed.text = std::move(value);
            typed.pointer_px = pointer_px_;
            dispatch(typed);
        }

        void pointer(ui::UiEventKind kind, math::Vec2 point, ui::PointerButton button = ui::PointerButton::primary, ui::KeyModifiers modifiers = {})
        {
            ui::UiEvent event;
            event.kind = kind;
            event.pointer_px = point;
            event.pointer_delta_px = point - pointer_px_;
            event.button = button;
            event.modifiers = modifiers;
            event.timestamp_s = wall_time_s;
            pointer_px_ = point;
            dispatch(event);
        }

        void wheel(math::Vec2 point, double delta)
        {
            ui::UiEvent event;
            event.kind = ui::UiEventKind::wheel;
            event.pointer_px = point;
            event.wheel_delta = delta;
            pointer_px_ = point;
            dispatch(event);
        }

        void focus_window(bool focused)
        {
            ui::UiEvent event;
            event.kind = focused ? ui::UiEventKind::focus_gained : ui::UiEventKind::focus_lost;
            event.pointer_px = pointer_px_;
            dispatch(event);
        }

        void click(math::Vec2 point, ui::PointerButton button = ui::PointerButton::primary, ui::KeyModifiers modifiers = {})
        {
            pointer(ui::UiEventKind::pointer_move, point, button, modifiers);
            pointer(ui::UiEventKind::pointer_down, point, button, modifiers);
            pointer(ui::UiEventKind::pointer_up, point, button, modifiers);
        }

        [[nodiscard]] math::Vec2 stage_point(double x, double y) const
        {
            const auto stage = interface.layout().stage;
            return { stage.minimum.x + stage.width() * x, stage.minimum.y + stage.height() * y };
        }

        // The laid-out box of a displayed element with a non-empty area.
        [[nodiscard]] std::optional<ui::Rect> visible_bounds(std::string_view id) const
        {
            const auto& backend = document();
            if (!backend.element_visible(id))
                return std::nullopt;
            const auto bounds = backend.element_bounds(id);
            if (!bounds || bounds->width() <= 0.0 || bounds->height() <= 0.0)
                return std::nullopt;
            return bounds;
        }

        // Presses and releases the primary button at the centre of a displayed element.
        bool click_element(std::string_view id)
        {
            const auto bounds = visible_bounds(id);
            if (!bounds)
                return false;
            click((bounds->minimum + bounds->maximum) * 0.5);
            return true;
        }

        // Displayed controls, rails and toast buttons whose centre lies inside the viewport.
        [[nodiscard]] std::vector<std::pair<std::string, math::Vec2>> visible_controls() const
        {
            std::vector<std::pair<std::string, math::Vec2>> result;
            auto ids = document().control_ids();
            for (const auto* extra : { "rail-guide", "rail-inspector", "region-scrim", "measure-grip" })
                ids.emplace_back(extra);
            for (const auto& notification : session.build_model().notifications)
            {
                const auto prefix = "toast-" + std::to_string(notification.serial);
                ids.push_back(prefix + "--dismiss");
                ids.push_back(prefix + "--action");
            }
            const auto size = viewport();
            for (const auto& id : ids)
                if (const auto bounds = visible_bounds(id))
                {
                    const auto centre = (bounds->minimum + bounds->maximum) * 0.5;
                    if (centre.x >= 0.0 && centre.y >= 0.0 && centre.x < size.width && centre.y < size.height)
                        result.emplace_back(id, centre);
                }
            return result;
        }

        // Returns to the plain workspace through the paths a learner has: Discard, Present off,
        // Escape-equivalent closes, Cancel on a confirmation and an application focus change.
        void settle()
        {
            // A frame first lets the interface take up requests still queued in the session, such
            // as a context menu asked for by the last press.
            frame();
            // The pointer leaves every object, so no hover card lingers over the interface.
            ui::UiEvent away;
            away.kind = ui::UiEventKind::pointer_move;
            away.pointer_px = { -1.0e4, -1.0e4 };
            (void)session.handle_scene_event(away);
            if (session.build_model().shape_editor_active)
            {
                ui::UiCommand discard;
                discard.kind = ui::UiCommandKind::cancel_shape_outline;
                apply_command(discard);
            }
            if (interface.view_state().present().mode)
                interface.request(ui::ViewRequest::toggle_present);
            interface.view_state().set_pick_surface_armed(false);
            interface.view_state().set_section_open("draw.discard_prompt", false);
            for (int attempt = 0; attempt < 16 && (!interface.view_state().sheets().empty() || !interface.view_state().transients().empty()); ++attempt)
                interface.request(ui::ViewRequest::close_top_surface);
            if (const auto model = session.build_model(); model.confirmation)
                apply_command(model.confirmation->cancel);
            focus_window(false);
            focus_window(true);
            frames_for(3);
        }

        // The interface still answers a learner: Play is laid out and displayed, and a pointer
        // press on it toggles the simulation exactly once.
        void expect_usable(const std::string& context)
        {
            settle();
            RIGIDBODIES_EXPECT(interface.backend_name() == "rmlui", "the document backend is still running " + context);
            const auto& backend = document();
            const auto play = backend.element_for_key("legacy", "transport.play");
            RIGIDBODIES_EXPECT(play.has_value(), "the Play control exists " + context);
            const auto bounds = visible_bounds(*play);
            RIGIDBODIES_EXPECT(bounds.has_value(), "the Play control is displayed with a layout box " + context);
            const auto before = session.build_model();
            const auto centre = (bounds->minimum + bounds->maximum) * 0.5;
            const auto hit = backend.element_at(centre);
            const auto commands_before = commands;
            click(centre);
            const auto after = session.build_model();
            std::ostringstream state;
            state << context << " (Play at " << centre.x << "," << centre.y << " in " << viewport().width << "x" << viewport().height << " at " << interface.scale()
                  << "x; hits " << hit << ", over surface " << interface.surface_at(centre) << ", commands " << commands - commands_before << ", run state " << static_cast<int>(before.run_state) << "->" << static_cast<int>(after.run_state)
                  << ", drawing " << after.shape_editor_active << ", confirmation " << after.confirmation.has_value() << ", focus " << static_cast<int>(interface.focus_owner()) << ")";
            RIGIDBODIES_EXPECT(after.paused != before.paused, "pressing Play toggles the simulation exactly once " + state.str());
            frame();
            RIGIDBODIES_EXPECT(backend.control_count() > 10, "the document still exposes its keyboard controls " + context);
        }

        ui::UiContext interface;
        app::SimulationSession session;
        render::DrawList scene_list, interface_list;
        float display_scale { 1.0f };
        double wall_time_s { 1.0 };
        std::string saved_setup, saved_shape, clipboard;
        std::size_t frames { 0 }, meshes { 0 }, commands { 0 };
        // The window around the interface, as Application reports it each frame, and the window
        // commands the interface sent, which Application would carry out on the window.
        ui::WindowFrameState window_frame;
        std::vector<ui::UiCommandKind> window_requests;

    private:
        // What Application reads after every event to drive the platform text input and cursor.
        std::optional<app::SetupDeparture> save_continuation_;
        void poll_platform_state()
        {
            if (interface.wants_text_input())
            {
                const auto area = interface.text_input_area();
                RIGIDBODIES_EXPECT(std::isfinite(area.minimum.x) && std::isfinite(area.maximum.y), "the text input area is finite");
            }
            (void)interface.cursor();
            (void)session.scene_cursor();
        }

        SDL_Surface* surface_ { nullptr };
        render::RenderDevicePtr software_;
        std::unique_ptr<ViewportDevice> device_;
        math::Vec2 pointer_px_ { 800.0, 450.0 };
    };
}
