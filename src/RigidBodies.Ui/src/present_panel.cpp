#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

#include <rigidbodies/core/text_format.hpp>

#include <algorithm>

namespace rigidbodies::ui
{
    namespace
    {
        UiCommand action(UiCommandKind kind, std::string_view detail = {})
        {
            UiCommand c;
            c.kind = kind;
            c.detail = detail;
            return c;
        }

        physics::BodyId annotated_body(const UiModel& model, std::string_view document_id)
        {
            const auto found = std::find_if(model.objects.begin(), model.objects.end(), [&](const auto& value)
                {
                    return value.document_id == document_id;
                });
            return found == model.objects.end() ? physics::BodyId {} : found->id;
        }

        std::size_t current_step(const PanelBuilder& builder, std::size_t count)
        {
            std::size_t index {};
            try
            {
                index = static_cast<std::size_t>(std::stoul(std::string(builder.view_value("present.guide_step", "0"))));
            }
            catch (...)
            {
            }
            return count == 0 ? 0 : std::min(index, count - 1);
        }
    }

    std::string_view PresentPanel::id() const
    {
        return "present";
    }
    std::string_view PresentPanel::title() const
    {
        return "Present";
    }
    RegionId PresentPanel::region() const
    {
        return RegionId::present_strip;
    }

    // Every control stays in the strip at any width: Present mode has no menu to fold into, so
    // labels shed first and only the title and clock may leave.
    void PresentPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!builder.view_present())
            return;
        builder.begin_group("identity");
        builder.title(model.scenario_title.empty() ? "Rigid Bodies" : model.scenario_title);
        builder.present_last(presentation().shrink_at(1).overflow_at(3));
        builder.end_group();
        builder.spacer(0.0);

        builder.begin_group("transport");
        builder.action_row("present.transport.reset", "Back to start", action(UiCommandKind::reset_scenario), model.elapsed_time_s <= 0.0 ? "Already at the starting setup." : "");
        builder.present_last(presentation(icons::back_to_start, "R").icon_label_only());
        const auto running = model.run_state == RunState::running;
        builder.action_row("present.transport.play", running ? "Pause" : "Play", action(UiCommandKind::toggle_pause));
        builder.present_last(presentation(running ? icons::pause : icons::play, "Space").primary().hide_label_at(5));
        builder.action_row("present.transport.step", "Step", action(UiCommandKind::single_step));
        builder.present_last(presentation(icons::step, ".").icon_label_only());
        // A relativity experiment's clock is the lab's, in nanoseconds of lab time; the strip shows
        // no names, so the reading carries its own.
        builder.readout("present.time", model.relativity ? "Lab time" : "Time", model.relativity ? "Lab time " + now_text(model) : core::substitute("{} s", core::fixed(model.elapsed_time_s, 2)), RowTone::normal, true);
        builder.present_last(presentation(icons::timer).overflow_at(4));
        builder.end_group();
        builder.spacer(0.0);

        // A one-time suggestion for the presenter, kept off the audience-facing caption.
        if (model.theme_id != "workbench_projector" && !builder.view_hint_dismissed("projector_theme"))
        {
            builder.begin_group("projector");
            auto use = action(UiCommandKind::set_theme, "projector:switch");
            use.id = "workbench_projector";
            builder.action_row("present.theme.switch", "Use projector theme", use);
            builder.present_last(presentation(icons::display).hide_label_at(1).overflow_at(3));
            builder.action_row("present.theme.keep", "Keep current theme", action(UiCommandKind::none, "projector:keep"));
            builder.present_last(presentation(icons::close).icon_label_only().quiet().overflow_at(3));
            builder.end_group();
        }
        builder.begin_group("presenter");
        const auto spotlight = builder.view_present_spotlight();
        builder.action_row("present.transport.spotlight", "Spotlight", action(UiCommandKind::none, "present:spotlight"));
        builder.present_last(presentation(icons::spotlight).hide_label_at(2));
        builder.select_last(spotlight);
        const auto locked = builder.view_present_locked();
        // The pressed button names the state it holds; the released one, the action it offers.
        builder.action_row("present.transport.lock", locked ? "Locked" : "Lock", action(UiCommandKind::none, "present:lock"));
        builder.present_last(presentation(locked ? icons::lock : icons::unlock).hide_label_at(2));
        builder.select_last(locked);
        builder.action_row("present.transport.exit", "Exit", action(UiCommandKind::none, "view:present"));
        // Exit keeps its label for as long as the strip has room: next to the window's own close
        // button a bare cross would read as one more way to close the window.
        builder.present_last(presentation(icons::close, "Esc").hide_label_at(6));
        builder.end_group();
    }

    std::string_view PresentCaptionPanel::id() const
    {
        return "present_caption";
    }
    std::string_view PresentCaptionPanel::title() const
    {
        return "Lesson step";
    }
    RegionId PresentCaptionPanel::region() const
    {
        return RegionId::present_caption;
    }

    void PresentCaptionPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!builder.view_present())
            return;
        if (model.scenario_content && !model.scenario_content->guide.steps.empty())
        {
            const auto& guide = model.scenario_content->guide;
            const auto& steps = guide.steps;
            const auto index = current_step(builder, steps.size());
            const auto& step = steps[index];
            // The step's own control is offered on the card, so an edit the caption asks for works
            // in Present. A step that reads Measure or edits shape structure needs the workbench;
            // its button leaves Present and opens the place the Guide would reveal.
            std::string label;
            for (const auto& variable : guide.variables)
                if (variable.control == step.control && variable.body == step.body && variable.instance == step.instance)
                    label = variable.label;
            const auto inline_control = !step.control.empty() && guide_control_fits_row(step.control);
            const auto reveal_key = !step.open.empty() ? step.open : !inline_control ? step.control
                                                                                     : std::string {};
            builder.begin_group("caption-header");
            builder.label(core::substitute("Step {} of {}", index + 1, steps.size()));
            const auto body = annotated_body(model, step.body);
            if (body.is_valid() || !step.instance.empty() || !reveal_key.empty())
            {
                UiCommand show;
                show.kind = !step.instance.empty() ? UiCommandKind::select_connection : body.is_valid() ? UiCommandKind::select_body
                                                                                                        : UiCommandKind::none;
                show.body = body;
                show.id = step.instance;
                if (!step.instance.empty())
                    show.detail = model.world && model.world->spring_by_key(step.instance).is_valid() ? "spring" : "joint";
                if (!reveal_key.empty())
                {
                    // Leaving Present reveals the setting where it lives; a connection is then
                    // reached through the Inspector rather than selected here.
                    if (show.kind == UiCommandKind::select_connection)
                        show.kind = UiCommandKind::none;
                    show.detail = "present-reveal:" + reveal_key;
                    builder.action_row("present.strip.show_me", "Exit and show me", show);
                    builder.present_last(presentation(icons::frame_subject));
                }
                else
                {
                    builder.action_row("present.strip.show_me", "Show me", show);
                    builder.present_last(presentation(icons::frame_subject));
                }
            }
            builder.action_row("present.strip.prev", "Previous step", action(UiCommandKind::none, "present:prev"), index == 0 ? "This is the first step." : "");
            builder.present_last(presentation(icons::previous_step, "Left").quiet());
            builder.action_row("present.strip.next", "Next step", action(UiCommandKind::none, "present:next"), index + 1 >= steps.size() ? "This is the last step." : "");
            builder.present_last(presentation(icons::next_step, "Right").primary().icon_trailing());
            builder.end_group();
            builder.paragraph(step.text);
            if (inline_control)
            {
                builder.begin_group("caption-control");
                guide_control_row(model, builder, step.control, step.body, step.instance, label, "present");
                builder.end_group();
            }
        }
    }
}
