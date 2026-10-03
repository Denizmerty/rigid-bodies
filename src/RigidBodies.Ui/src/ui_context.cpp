#include <rigidbodies/ui/ui_context.hpp>

#include <rigidbodies/core/log.hpp>
#include <rigidbodies/ui/overlay_backend.hpp>
#if defined(RIGIDBODIES_HAS_RMLUI)
#include <rigidbodies/ui/document_backend.hpp>
#endif

#include <algorithm>
#include <array>
#include <charconv>
#include <utility>

namespace rigidbodies::ui
{
    namespace
    {
        // Menus and popovers that close when another opens, when an item is chosen, or when the
        // pointer presses elsewhere.
        constexpr std::array<const char*, 5> menu_transients { "main_menu", "add_menu", "context_menu", "show", "draw_options" };
    }

    std::string_view to_string(UiBackendKind kind)
    {
        switch (kind)
        {
        case UiBackendKind::overlay:
            return "overlay";
        case UiBackendKind::document:
            return "document";
        }
        return "overlay";
    }

    UiBackendKind backend_kind_from_name(std::string_view name)
    {
        if (name == "rmlui" || name == "document")
        {
            return UiBackendKind::document;
        }
        return UiBackendKind::overlay;
    }

    UiBackendPtr create_ui_backend(UiBackendKind kind)
    {
        switch (kind)
        {
        case UiBackendKind::overlay:
            return std::make_unique<OverlayBackend>();
        case UiBackendKind::document:
        {
#if defined(RIGIDBODIES_HAS_RMLUI)
            return std::make_unique<DocumentBackend>();
#else
            return nullptr;
#endif
        }
        }
        return nullptr;
    }

    UiContext::UiContext()
    {
        view_state_.set_surface_open("hints.enabled", true);
    }

    UiContext::~UiContext()
    {
        shutdown();
    }

    bool UiContext::initialize(UiBackendKind requested, const std::filesystem::path& asset_root)
    {
        backend_ = create_ui_backend(requested);
        if (!backend_ || !backend_->initialize(asset_root))
        {
            if (requested != UiBackendKind::overlay)
            {
                core::log_warning("the {} interface backend is not available in this build; using the overlay instead", to_string(requested));
            }
            backend_ = create_ui_backend(UiBackendKind::overlay);
            if (!backend_ || !backend_->initialize(asset_root))
            {
                core::log_error("no interface backend could be started");
                backend_.reset();
                return false;
            }
        }

        if (panels_.empty())
        {
            panels_ = create_default_panels();
        }

        backend_->set_clipboard_hooks(clipboard_reader_, clipboard_writer_);

        core::log_info("interface ready on the {} backend with {} panels", backend_->name(), panels_.size());
        return true;
    }

    void UiContext::shutdown()
    {
        if (backend_)
        {
            backend_->shutdown();
            backend_.reset();
        }
        commands_.clear();
    }

    std::string_view UiContext::backend_name() const
    {
        return backend_ ? backend_->name() : std::string_view { "none" };
    }

    const UiBackend* UiContext::backend() const
    {
        return backend_.get();
    }

    const Theme& UiContext::theme() const
    {
        return theme_;
    }

    void UiContext::set_theme(const Theme& value)
    {
        theme_ = value;
    }

    float UiContext::scale() const
    {
        return scale_;
    }

    void UiContext::set_scale(float value)
    {
        // A scale outside this range makes the interface either unreadable or large enough to
        // leave no room for the scene.
        scale_ = std::clamp(value, 0.5f, 4.0f);
    }

    void UiContext::set_window_frame(const WindowFrameState& value)
    {
        window_frame_ = value;
    }

    bool UiContext::draws_window_controls() const
    {
        return backend_ && backend_->draws_window_controls();
    }

    WindowPart UiContext::window_part(const Vec2& point) const
    {
        if (!layout_.find(RegionId::title_bar))
            return WindowPart::client;
        if (backend_)
            if (const auto part = backend_->window_part(point))
                return *part;
        // Without the backend's own answer the controls share their region in order, and the rest
        // of the strip is title bar.
        if (const auto* controls = layout_.find(RegionId::window_controls); controls && controls->bounds.contains(point))
        {
            const auto button = 46.0 * static_cast<double>(scale_);
            const auto from_right = controls->bounds.maximum.x - point.x;
            if (from_right < button)
                return WindowPart::close;
            if (from_right < 2.0 * button)
                return WindowPart::maximize;
            if (from_right < 3.0 * button)
                return WindowPart::minimize;
            return WindowPart::title_bar;
        }
        const auto* strip = layout_.find(RegionId::title_bar);
        return strip->bounds.contains(point) ? WindowPart::title_bar : WindowPart::client;
    }

    void UiContext::add_panel(std::unique_ptr<Panel> panel)
    {
        if (panel)
        {
            panels_.push_back(std::move(panel));
        }
    }

    Panel* UiContext::find_panel(std::string_view id)
    {
        for (const auto& panel : panels_)
        {
            if (panel->id() == id)
            {
                return panel.get();
            }
        }
        return nullptr;
    }

    const std::vector<std::unique_ptr<Panel>>& UiContext::panels() const
    {
        return panels_;
    }

    bool UiContext::handle_event(const UiEvent& event)
    {
        if (!backend_)
        {
            return false;
        }

        if (event.kind == UiEventKind::viewport_resized)
        {
            last_viewport_ = { event.viewport_width, event.viewport_height };
        }

        const auto consumed = backend_->handle_event(event, commands_);
        if (event.kind == UiEventKind::pointer_leave)
        {
            // A toast under the pointer stops being held open once the pointer has gone.
            if (hovered_notification_)
                view_state_.toast_presenter().set_hovered(*hovered_notification_, false, wall_time_s_);
            hovered_notification_.reset();
            pointer_over_interface_ = false;
        }
        if (event.kind == UiEventKind::pointer_move)
        {
            const auto hovered = backend_->hovered_notification();
            if (hovered != hovered_notification_)
            {
                if (hovered_notification_)
                    view_state_.toast_presenter().set_hovered(*hovered_notification_, false, wall_time_s_);
                if (hovered)
                    view_state_.toast_presenter().set_hovered(*hovered, true, wall_time_s_);
                hovered_notification_ = hovered;
            }
            pointer_over_interface_ = surface_at(event.pointer_px) || hovered.has_value();
        }
        return consumed;
    }

    void UiContext::release_focus()
    {
        if (backend_)
            backend_->release_focus(commands_);
    }

    void UiContext::build(const UiModel& model, const render::RenderDevice& device, DrawList& list, double wall_time_s)
    {
        if (!backend_)
        {
            return;
        }

        last_viewport_ = device.drawable_size();
        wall_time_s_ = wall_time_s;
        view_state_.set_experiment_context(model.scenario_id);
        if (model.last_setup_file)
            view_state_.merge_setup_file(*model.last_setup_file);
        if (model.context_menu_request && model.context_menu_request->serial > context_menu_serial_)
        {
            context_menu_serial_ = model.context_menu_request->serial;
            view_state_.set_value("context.kind", "stage");
            // The keyboard opens its menu at once; if Escape closed it before this frame, the
            // request it came with must not open it again.
            if (!context_menu_requested_)
            {
                close_menus();
                view_state_.open_transient("context_menu");
            }
        }
        context_menu_requested_ = false;
        const auto has_guide = model.scenario_content && !model.scenario_content->guide.empty();
        view_state_.set_surface_open("guide.available", has_guide);
        if (has_guide && !view_state_.visited(model.scenario_id))
        {
            view_state_.set_surface_open("guide.expanded", true);
            view_state_.mark_visited(model.scenario_id);
        }
        // Where only one side surface can dock, the most recent intent owns the dock: opening an
        // experiment gives it to the Guide, so its questions come before the first run; selecting
        // an object, or asking for the Inspector or World, gives it to the Inspector; asking for
        // the Guide gives it back. A Guide the learner collapsed stays collapsed.
        if (model.scenario_id != arranged_experiment_)
        {
            arranged_experiment_ = model.scenario_id;
            if (has_guide)
                view_state_.set_value("side.last_used", "inspector");
        }
        if (model.selection.is_valid() && !(model.selection == inspected_selection_))
        {
            view_state_.set_surface_open("inspector.open", true);
            // Selecting an object shows that object, even after World was brought forward.
            view_state_.set_active_tab("inspector.target", "selection");
            // The Guide yields only to an Inspector that can dock in its place; a window too
            // narrow for that keeps the Guide rather than trading it for a rail.
            if (!inspector_as_sheet())
                view_state_.set_value("side.last_used", "guide");
        }
        inspected_selection_ = model.selection;

        UiFrameContext context;
        context.model = &model;
        context.device = &device;
        context.theme = &theme_;
        context.scale = scale_;
        context.viewport = last_viewport_;
        context.view = &view_state_;
        LayoutInput layout_input;
        layout_input.viewport = last_viewport_;
        layout_input.scale = scale_;
        layout_input.previous_mode = previous_layout_mode_;
        layout_input.overlay_backend = backend_->name() == "overlay";
        layout_input.present = view_state_.present().mode;
        layout_input.draw_bar = (model.draft.active || model.shape_editor_active) && !view_state_.present().mode;
        if (!layout_input.draw_bar && view_state_.transient_open("draw_options"))
            view_state_.close_transient("draw_options");
        const auto has_steps = model.scenario_content && !model.scenario_content->guide.steps.empty();
        layout_input.present_caption = view_state_.present().mode && has_steps;
        layout_input.present_caption_height = layout_input.present_caption ? caption_height_logical_ : 0.0;
        layout_input.guide_available = view_state_.surface_open("guide.available", false);
        layout_input.guide_expanded = view_state_.surface_open("guide.expanded", false);
        layout_input.inspector_open = view_state_.surface_open("inspector.open", true);
        layout_input.narrow_guide_sheet = view_state_.surface_open("guide.sheet", false);
        layout_input.narrow_inspector_sheet = view_state_.surface_open("inspector.sheet", false);
        layout_input.measure_open = view_state_.surface_open("measure.open", false);
        layout_input.measure_height_logical = view_state_.number("measure.height", 0.0);
        layout_input.least_recently_used = view_state_.value("side.last_used", "guide") == "inspector" ? SideSurface::inspector : SideSurface::guide;
        layout_input.window_controls = window_frame_.controls && backend_->draws_window_controls();
        context.window_frame = window_frame_;
        context.window_frame.controls = layout_input.window_controls;
        layout_ = compute_layout(layout_input);
        layout_input_ = layout_input;
        previous_layout_mode_ = layout_.mode;
        context.layout = &layout_;
        context.wall_time_s = wall_time_s;
        context.toasts = view_state_.toast_presenter().update(model.notifications, wall_time_s, view_state_.present().mode);
        if (model.reveal_request && model.reveal_request->serial != reveal_request_serial_)
        {
            reveal_request_serial_ = model.reveal_request->serial;
            view_state_.set_surface_open("measure.open", true);
            reveal(model.reveal_request->key, model.reveal_request->instance);
        }
        const auto next_hint = view_state_.next_hint();
        if (next_hint == "play" && !model.paused)
            view_state_.dismiss_hint("play");
        else if (next_hint == "inspect" && !model.selected_bodies.empty())
            view_state_.dismiss_hint("inspect");
        else if (next_hint == "library" && view_state_.sheet_open("library"))
            view_state_.dismiss_hint("library");
        context.panels.reserve(panels_.size());
        // Sheets stack: only the one on top is shown, and closing it reveals the one beneath.
        const auto top_sheet = [&](std::string_view id)
        {
            return !view_state_.sheets().empty() && view_state_.sheets().back() == id;
        };
        for (const auto& panel : panels_)
        {
            bool surface_visible = true;
            if (backend_->name() != "overlay")
            {
                if (panel->id() == "library")
                    surface_visible = top_sheet("library");
                else if (panel->id() == "main_menu")
                    surface_visible = view_state_.transient_open("main_menu");
                else if (panel->id() == "preferences")
                    surface_visible = top_sheet("preferences");
                else if (panel->id() == "save_details")
                    surface_visible = top_sheet("save_details");
                else if (panel->id() == "shortcuts")
                    surface_visible = top_sheet("shortcuts");
                else if (panel->id() == "about")
                    surface_visible = top_sheet("about");
                else if (panel->id() == "present")
                    surface_visible = view_state_.present().mode;
                else if (panel->id() == "present_caption")
                    surface_visible = layout_.find(RegionId::present_caption) != nullptr;
                else if (panel->id() == "command_search")
                    surface_visible = top_sheet("command_search");
                else if (panel->id() == "context_menu")
                    surface_visible = view_state_.transient_open("context_menu");
                else if (panel->id() == "add_menu")
                    surface_visible = view_state_.transient_open("add_menu");
                else if (panel->id() == "hover_card")
                    surface_visible = model.hover && !model.hover->card_lines.empty() && view_state_.sheets().empty() && view_state_.transients().empty();
                else if (panel->id() == "hints")
                    surface_visible = !view_state_.present().mode && view_state_.sheets().empty() && view_state_.transients().empty() && !view_state_.next_hint().empty();
                else if (panel->id() == "performance")
                    surface_visible = view_state_.surface_open("performance.overlay", false);
                else if (panel->id() == "visualization")
                    surface_visible = view_state_.transient_open("show");
                else if (panel->id() == "measure")
                    surface_visible = view_state_.surface_open("measure.open", false);
                else if (panel->id() == "guide")
                    surface_visible = has_guide;
                else if (panel->id() == "draw_bar")
                    surface_visible = model.draft.active || model.shape_editor_active;
                else if (panel->id() == "draw_options")
                    surface_visible = (model.draft.active || model.shape_editor_active) && view_state_.transient_open("draw_options");
                else if (panel->id() == "banner")
                    surface_visible = model.banner.has_value();
                else if (panel->id() == "confirmation")
                    surface_visible = model.confirmation.has_value();
                if (view_state_.present().mode && (panel->id() == "command_bar" || panel->id() == "status_line" || panel->id() == "guide" || panel->id() == "inspector" || panel->id() == "measure" || panel->id() == "draw_bar" || panel->id() == "library"))
                    surface_visible = false;
            }
            if (panel->is_visible() && surface_visible)
            {
                context.panels.push_back(panel.get());
            }
        }

        backend_->build(context, list);
        caption_height_logical_ = backend_->measured_height(RegionId::present_caption) / std::max(0.01, static_cast<double>(scale_));
        if (pending_reveal_)
        {
            backend_->reveal(pending_reveal_->first, pending_reveal_->second);
            pending_reveal_.reset();
        }
        if (pending_focus_)
        {
            backend_->focus_field(*pending_focus_);
            pending_focus_.reset();
        }
    }

    std::vector<UiCommand> UiContext::take_commands()
    {
        auto taken = std::move(commands_);
        commands_.clear();
        taken.erase(std::remove_if(taken.begin(), taken.end(), [&](const UiCommand& command)
                        {
                            if (command.detail == "keep_drawing")
                            {
                                view_state_.set_section_open("draw.discard_prompt", false);
                                return true;
                            }
                            if (command.detail.rfind("reveal:", 0) == 0)
                            {
                                reveal(std::string_view(command.detail).substr(7), command.id);
                                return command.kind == UiCommandKind::none;
                            }
                            if (command.detail.rfind("present-reveal:", 0) == 0)
                            {
                                view_state_.present().mode = false;
                                reveal(std::string_view(command.detail).substr(15), command.kind == UiCommandKind::select_connection ? std::string_view(command.id) : std::string_view {});
                                return command.kind == UiCommandKind::none;
                            }
                            if (command.detail.rfind("search-reveal:", 0) == 0)
                            {
                                const auto key = std::string_view(command.detail).substr(14);
                                view_state_.remember_search(key);
                                view_state_.close_sheet("command_search");
                                reveal(key, command.id);
                                return true;
                            }
                            if (command.detail.rfind("context-ui:", 0) == 0)
                            {
                                const auto payload = std::string_view(command.detail).substr(11);
                                const auto separator = payload.find('\x1f');
                                view_state_.set_value("context.kind", separator == std::string_view::npos ? payload : payload.substr(0, separator));
                                view_state_.set_value("context.key", command.id);
                                view_state_.set_value("context.value", separator == std::string_view::npos ? std::string_view {} : payload.substr(separator + 1));
                                close_menus();
                                view_state_.open_transient("context_menu");
                                return true;
                            }
                            if (command.detail.rfind("dismiss-hint:", 0) == 0)
                            {
                                view_state_.dismiss_hint(std::string_view(command.detail).substr(13));
                                return true;
                            }
                            if (command.detail == "reset-hints")
                            {
                                view_state_.reset_hints();
                                return true;
                            }
                            if (command.detail.rfind("projector:", 0) == 0)
                            {
                                view_state_.dismiss_hint("projector_theme");
                                if (command.detail == "projector:keep")
                                    return true;
                            }
                            if (command.detail.rfind("search-run:", 0) == 0)
                            {
                                view_state_.remember_search(std::string_view(command.detail).substr(11));
                                view_state_.close_sheet("command_search");
                            }
                            if (command.detail.rfind("present:", 0) == 0)
                            {
                                const auto name = std::string_view(command.detail).substr(8);
                                if (name == "lock")
                                    view_state_.present().lock = !view_state_.present().lock;
                                else if (name == "spotlight")
                                    view_state_.present().spotlight = !view_state_.present().spotlight;
                                else if (name == "prev" || name == "next")
                                {
                                    auto value = view_state_.number("present.guide_step", 0.0);
                                    view_state_.set_number("present.guide_step", std::max(0.0, value + (name == "next" ? 1.0 : -1.0)));
                                    view_state_.set_value("present.guide_step", std::to_string(static_cast<int>(view_state_.number("present.guide_step", 0.0))));
                                }
                                return true;
                            }
                            if (command.kind == UiCommandKind::none && command.detail.rfind("dismiss_toast:", 0) == 0)
                            {
                                const auto serial_text = std::string_view(command.detail).substr(14);
                                std::uint64_t serial {};
                                const auto [end, error] = std::from_chars(serial_text.data(), serial_text.data() + serial_text.size(), serial);
                                if (error == std::errc {} && end == serial_text.data() + serial_text.size())
                                    view_state_.toast_presenter().dismiss(serial);
                                return true;
                            }
                            if (command.kind == UiCommandKind::none && command.detail == "remove-setup")
                            {
                                view_state_.remove_setup_file(command.id);
                                return true;
                            }
                            if (command.kind == UiCommandKind::none && command.detail.rfind("view:", 0) == 0)
                            {
                                if (command.id.rfind("search:", 0) == 0)
                                {
                                    view_state_.remember_search(std::string_view(command.id).substr(7));
                                    view_state_.close_sheet("command_search");
                                }
                                const auto name = std::string_view(command.detail).substr(5);
                                if (view_state_.present().mode && view_state_.present().lock && (name == "library" || name == "add"))
                                    return true;
                                if (name == "library")
                                    request(ViewRequest::open_library);
                                else if (name == "show")
                                    request(ViewRequest::toggle_show);
                                else if (name == "draw_options")
                                    open_menu("draw_options");
                                else if (name == "measure")
                                    request(ViewRequest::toggle_measure);
                                else if (name == "world")
                                    request(ViewRequest::open_world);
                                else if (name == "guide")
                                    request(ViewRequest::toggle_guide);
                                else if (name == "inspector")
                                    request(ViewRequest::toggle_inspector_pin);
                                else if (name == "menu")
                                    request(ViewRequest::open_main_menu);
                                else if (name == "preferences")
                                    request(ViewRequest::open_preferences);
                                else if (name == "save_details")
                                    request(ViewRequest::open_save_details);
                                else if (name == "shortcuts")
                                    request(ViewRequest::open_shortcuts);
                                else if (name == "add")
                                    request(ViewRequest::open_add_menu);
                                else if (name == "about")
                                    request(ViewRequest::open_about);
                                else if (name == "present")
                                    request(ViewRequest::toggle_present);
                                else if (name == "search")
                                    request(ViewRequest::open_command_search);
                                else if (name == "performance")
                                    request(ViewRequest::toggle_performance_overlay);
                                else if (name == "pick_surface")
                                    request(ViewRequest::arm_pick_surface);
                                else if (name == "close")
                                    request(ViewRequest::close_top_surface);
                                return true;
                            }
                            if (view_state_.present().mode && view_state_.present().lock)
                            {
                                const auto locked = command.kind == UiCommandKind::delete_selected_body || command.kind == UiCommandKind::start_new_shape || command.kind == UiCommandKind::add_object || command.kind == UiCommandKind::import_shape || command.kind == UiCommandKind::open_arrangement || command.kind == UiCommandKind::save_arrangement;
                                if (locked)
                                    return true;
                            }
                            if (command.kind == UiCommandKind::none && command.detail.rfind("state:", 0) == 0)
                            {
                                const auto assignment = std::string_view(command.detail).substr(6);
                                const auto separator = assignment.find('=');
                                if (separator != std::string_view::npos)
                                    view_state_.set_value(assignment.substr(0, separator), assignment.substr(separator + 1));
                                return true;
                            }
                            if (command.kind == UiCommandKind::none && command.detail == "library-show")
                            {
                                view_state_.set_value("library.selected", command.id);
                                reveal("library.cards.card", command.id);
                                return true;
                            }
                            if (command.kind == UiCommandKind::none && command.detail.rfind("state-id:", 0) == 0)
                            {
                                view_state_.set_value(std::string_view(command.detail).substr(9), command.id);
                                return true;
                            }
                            if (command.kind == UiCommandKind::none && command.detail.rfind("show-run-in-graph:", 0) == 0)
                            {
                                view_state_.set_value("measure.graph.compare_with", std::string_view(command.detail).substr(18));
                                view_state_.set_active_tab("measure.header.tabs", "graph");
                                view_state_.set_surface_open("measure.open", true);
                                return true;
                            }
                            if (command.kind == UiCommandKind::load_scenario)
                            {
                                view_state_.mark_visited(command.id);
                                view_state_.set_value("library.last_experiment", command.id);
                                view_state_.close_sheet("library");
                            }
                            if (command.kind == UiCommandKind::toggle_pause)
                                view_state_.dismiss_hint("play");
                            if (command.kind == UiCommandKind::select_body || command.kind == UiCommandKind::select_bodies)
                                view_state_.dismiss_hint("inspect");
                            if (command.kind == UiCommandKind::cancel_shape_outline)
                                view_state_.set_section_open("draw.discard_prompt", false);
                            return false;
                        }),
            taken.end());
        return taken;
    }

    bool UiContext::surface_at(const Vec2& point) const
    {
        if (!view_state_.sheets().empty())
            return true; // The sheet's scrim makes the stage inert, including outside the card.
        if (backend_)
            if (const auto covered = backend_->covers(point))
                return *covered;
        if (!view_state_.transients().empty())
            if (const auto* transient = layout_.find(view_state_.transients().back() == "show" ? RegionId::show_popover : RegionId::modal); transient && transient->bounds.contains(point))
                return true;
        for (const auto& region : layout_.regions)
            if (region.id != RegionId::stage && region.id != RegionId::library_sheet && region.id != RegionId::show_popover && region.id != RegionId::modal && region.id != RegionId::performance_overlay && region.bounds.contains(point))
                return true;
        return false;
    }

    bool UiContext::menu_open() const
    {
        for (const auto* id : menu_transients)
            if (view_state_.transient_open(id))
                return true;
        return false;
    }

    bool UiContext::dismiss_menus_at(const Vec2& point)
    {
        // The bare title bar counts as outside: a press there closes the menu, as on a native one.
        if (!menu_open() || (surface_at(point) && !(backend_ && backend_->bare_title_bar(point))))
            return false;
        close_menus();
        return true;
    }

    void UiContext::close_menus()
    {
        for (const auto* id : menu_transients)
            view_state_.close_transient(id);
    }

    void UiContext::open_menu(std::string_view id)
    {
        const auto already_open = view_state_.transient_open(id);
        close_menus();
        if (!already_open)
            view_state_.open_transient(id);
    }

    const LayoutResult& UiContext::layout() const
    {
        return layout_;
    }

    bool UiContext::inspector_as_sheet() const
    {
        if (layout_.mode == LayoutMode::narrow)
            return true;
        auto input = layout_input_;
        input.inspector_open = true;
        input.least_recently_used = SideSurface::guide;
        input.narrow_inspector_sheet = false;
        return compute_layout(input).find(RegionId::inspector) == nullptr;
    }

    FocusOwner UiContext::focus_owner() const
    {
        return backend_ ? backend_->focus_owner() : FocusOwner::scene;
    }

    EscapeTarget UiContext::escape_target() const
    {
        if (view_state_.section_open("draw.discard_prompt", false))
            return EscapeTarget::transient;
        if (!view_state_.transients().empty())
            return EscapeTarget::transient;
        if (!view_state_.sheets().empty())
            return EscapeTarget::sheet;
        return backend_ ? backend_->escape_target() : EscapeTarget::none;
    }

    bool UiContext::wants_text_input() const
    {
        return backend_ && backend_->wants_text_input();
    }

    Rect UiContext::text_input_area() const
    {
        return backend_ ? backend_->text_input_area() : Rect {};
    }

    std::optional<Rect> UiContext::hover_card_bounds() const
    {
        return backend_ ? backend_->hover_card_bounds() : std::nullopt;
    }

    CursorShape UiContext::cursor() const
    {
        return backend_ ? backend_->cursor() : CursorShape::arrow;
    }

    bool UiContext::pointer_captured() const
    {
        return backend_ && backend_->pointer_captured();
    }

    void UiContext::set_clipboard_hooks(ClipboardReader reader, ClipboardWriter writer)
    {
        clipboard_reader_ = std::move(reader);
        clipboard_writer_ = std::move(writer);
        if (backend_)
            backend_->set_clipboard_hooks(clipboard_reader_, clipboard_writer_);
    }

    ViewState& UiContext::view_state()
    {
        return view_state_;
    }

    const ViewState& UiContext::view_state() const
    {
        return view_state_;
    }

    void UiContext::request(ViewRequest request, std::string_view key)
    {
        switch (request)
        {
        case ViewRequest::open_shortcuts:
            view_state_.open_sheet("shortcuts");
            break;
        case ViewRequest::open_library:
            view_state_.open_sheet("library");
            view_state_.dismiss_hint("library");
            break;
        case ViewRequest::toggle_show:
            open_menu("show");
            break;
        case ViewRequest::toggle_measure:
            view_state_.toggle_surface("measure.open", false);
            view_state_.set_value("side.last_used", "inspector");
            break;
        case ViewRequest::open_world:
            view_state_.set_surface_open("inspector.open", true);
            if (inspector_as_sheet())
            {
                view_state_.set_surface_open("inspector.sheet", true);
                view_state_.set_surface_open("guide.sheet", false);
            }
            view_state_.set_value("side.last_used", "guide");
            reveal(key.empty() ? "world.gravity.enabled" : key);
            break;
        case ViewRequest::toggle_guide:
            view_state_.set_surface_open("guide.available", true);
            if (layout_.mode == LayoutMode::narrow)
            {
                // Narrow windows show one sheet at a time and never one the learner did not ask for.
                const auto opening = !layout_.find(RegionId::guide_panel);
                view_state_.set_surface_open("guide.sheet", opening);
                if (opening)
                    view_state_.set_surface_open("inspector.sheet", false);
            }
            // A guide that is open but yielded its dock to the inspector is brought forward
            // rather than closed, so the request always changes what the learner sees.
            else if (!(view_state_.surface_open("guide.expanded", false) && !layout_.find(RegionId::guide_panel)))
                view_state_.toggle_surface("guide.expanded", false);
            view_state_.set_value("side.last_used", "inspector");
            break;
        case ViewRequest::toggle_inspector_pin:
            if (inspector_as_sheet())
            {
                const auto opening = !layout_.find(RegionId::inspector);
                view_state_.set_surface_open("inspector.open", true);
                view_state_.set_surface_open("inspector.sheet", opening);
                if (opening)
                    view_state_.set_surface_open("guide.sheet", false);
            }
            else if (!(view_state_.surface_open("inspector.open", true) && !layout_.find(RegionId::inspector)))
                view_state_.toggle_surface("inspector.open", true);
            view_state_.set_value("side.last_used", "guide");
            break;
        case ViewRequest::open_main_menu:
            open_menu("main_menu");
            break;
        case ViewRequest::open_preferences:
            view_state_.open_sheet("preferences");
            break;
        case ViewRequest::open_save_details:
            view_state_.open_sheet("save_details");
            break;
        case ViewRequest::open_add_menu:
            open_menu("add_menu");
            break;
        case ViewRequest::open_about:
            view_state_.open_sheet("about");
            break;
        case ViewRequest::toggle_present:
            view_state_.present().mode = !view_state_.present().mode;
            view_state_.close_sheet("command_search");
            break;
        case ViewRequest::open_command_search:
            view_state_.open_sheet("command_search");
            // A palette is for typing: its field takes the keys at once, so letters reach the
            // search instead of the stage's single-key shortcuts.
            pending_focus_ = "search.field.query";
            break;
        case ViewRequest::open_context_menu:
            close_menus();
            view_state_.open_transient("context_menu");
            context_menu_requested_ = true;
            break;
        case ViewRequest::toggle_performance_overlay:
            view_state_.toggle_surface("performance.overlay", false);
            break;
        case ViewRequest::arm_pick_surface:
            view_state_.set_pick_surface_armed(true);
            view_state_.close_transient("show");
            break;
        case ViewRequest::close_top_surface:
            if (!view_state_.transients().empty())
                view_state_.close_transient(view_state_.transients().back());
            else if (!view_state_.sheets().empty())
                view_state_.close_sheet(view_state_.sheets().back());
            break;
        }
        if (request == ViewRequest::open_shortcuts && !key.empty())
        {
            const auto section_key = key.empty() ? std::string_view { "tools.reference" } : key;
            view_state_.set_section_open(section_key, true);
        }
    }

    void UiContext::reveal(std::string_view key, std::string_view instance)
    {
        if (key.empty())
            return;
        // The Inspector builds only its current target and tab, so a revealed setting first
        // brings its page forward; the row is focused once the next build has created it.
        if (key.rfind("world.", 0) == 0)
            view_state_.set_active_tab("inspector.target", "world");
        else if (key.rfind("object.", 0) == 0)
        {
            const auto page = key.rfind("object.properties.", 0) == 0 ? std::string_view { "properties" }
                : key.rfind("object.motion.", 0) == 0                 ? std::string_view { "motion" }
                : key.rfind("object.forces.", 0) == 0                 ? std::string_view { "forces" }
                : key.rfind("object.shape.", 0) == 0                  ? std::string_view { "shape" }
                                                                      : std::string_view {};
            if (!page.empty())
            {
                view_state_.set_active_tab("inspector.target", "selection");
                view_state_.set_active_tab("inspector.object", page);
            }
        }
        pending_reveal_ = { std::string(key), std::string(instance) };
        if (key.rfind("world.", 0) == 0 || key.rfind("object.", 0) == 0 || key.rfind("joint.", 0) == 0 || key.rfind("spring.", 0) == 0)
        {
            view_state_.set_surface_open("inspector.open", true);
            // Revealing a setting is an explicit request: the Inspector takes the dock from the
            // Guide, and a window too narrow to dock it opens the sheet.
            view_state_.set_value("side.last_used", "guide");
            if (inspector_as_sheet())
            {
                view_state_.set_surface_open("inspector.sheet", true);
                view_state_.set_surface_open("guide.sheet", false);
            }
        }
        else if (key.rfind("measure.", 0) == 0)
            view_state_.set_surface_open("measure.open", true);
        else if (key.rfind("show.", 0) == 0 || key == "prefs.units.system" || key == "camera.scale.height")
        {
            close_menus();
            view_state_.open_transient("show");
        }
        else if (key.rfind("prefs.", 0) == 0)
            view_state_.open_sheet("preferences");
        const auto first_dot = key.find('.');
        if (first_dot != std::string_view::npos)
            view_state_.set_section_open(key.substr(0, key.find('.', first_dot + 1)), true);
        if (backend_)
            backend_->reveal(key, instance);
    }

} // namespace rigidbodies::ui
