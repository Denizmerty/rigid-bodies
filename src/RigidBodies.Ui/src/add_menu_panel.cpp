#include <rigidbodies/ui/icons.hpp>
#include <rigidbodies/ui/panels.hpp>

namespace rigidbodies::ui
{
    std::string_view AddMenuPanel::id() const
    {
        return "add_menu";
    }
    std::string_view AddMenuPanel::title() const
    {
        return "Add";
    }
    RegionId AddMenuPanel::region() const
    {
        return RegionId::show_popover;
    }
    void AddMenuPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!builder.view_transient_open("add_menu"))
            return;
        builder.heading("Add object");
        const auto locked = builder.view_present() && builder.view_present_locked();
        // The menu stays open across a switch to a relativity experiment, whose probe is its only object.
        const auto reason = model.relativity ? "Objects cannot be added to a relativity experiment." : locked ? "Locked in Present mode. Use the lock button to unlock."
                                                                                                              : "";
        struct Item
        {
            const char* id;
            const char* label;
            std::string_view icon;
        };
        for (const auto& item : { Item { "ball", "Ball", icons::ball }, Item { "box", "Box", icons::box }, Item { "plank", "Plank", icons::plank } })
        {
            UiCommand add;
            add.kind = UiCommandKind::add_object;
            add.id = item.id;
            builder.action_row("add." + std::string(item.id), item.label, add, reason);
            builder.present_last(presentation(item.icon));
        }
        UiCommand draw;
        draw.kind = UiCommandKind::start_new_shape;
        builder.action_row("Draw shape", draw, model.shape_editor_active && !model.relativity ? "A shape is already being drawn." : reason);
        builder.present_last(presentation(icons::draw, "D"));
    }
}
