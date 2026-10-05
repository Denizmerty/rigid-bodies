#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

namespace rigidbodies::ui
{
    std::string_view HintsPanel::id() const
    {
        return "hints";
    }
    std::string_view HintsPanel::title() const
    {
        return "Hint";
    }
    RegionId HintsPanel::region() const
    {
        return RegionId::toasts;
    }

    void HintsPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        const auto hint = builder.view_next_hint();
        if (!builder.view_surface_open("hints.enabled", false) || hint.empty() || builder.view_present())
            return;
        builder.title("Quick start");
        // A prediction is asked only before the first run, so the hint sends the learner to it
        // first; where the Guide is folded away (a narrow window's ribbon) the hint opens it.
        const auto predict_first = hint == "play" && prediction_pending(model);
        const auto guide_shown = builder.view_region_present(RegionId::guide_panel);
        const auto text = hint == "play" ? (!predict_first ? "Press Play to start the experiment." : guide_shown ? "Answer the prediction in the Guide, then press Play."
                                                                                                                 : "Open the Guide to answer its prediction, then press Play.")
            : hint == "inspect"          ? (model.relativity ? "Drag Probe speed, or press Shift+Up, to send the probe closer to c." : "Select an object, then drag its arrow tip to set velocity.")
                                         : "Open the Library to try another experiment.";
        builder.paragraph(text);
        if (predict_first && !guide_shown)
        {
            UiCommand open_guide;
            open_guide.detail = "view:guide";
            builder.action_row("hints.open_guide", "Open Guide", open_guide);
            builder.present_last(presentation(icons::guide, "G").primary());
        }
        UiCommand dismiss;
        dismiss.detail = "dismiss-hint:" + std::string(hint);
        builder.action_row("hints.dismiss", "Got it", dismiss);
    }
}
