#include <rigidbodies/ui/panels.hpp>

namespace rigidbodies::ui
{
    std::string_view BannerPanel::id() const
    {
        return "banner";
    }
    std::string_view BannerPanel::title() const
    {
        return "Notice";
    }
    RegionId BannerPanel::region() const
    {
        return RegionId::banner;
    }
    void BannerPanel::build(const UiModel& model, PanelBuilder& builder)
    {
        if (!model.banner)
            return;
        builder.notice("stage.banner", Severity::info, model.banner->text, model.banner->actions.empty() ? std::optional<NotificationAction> {} : model.banner->actions.front(), model.banner->actions.size() < 2 ? std::optional<NotificationAction> {} : model.banner->actions[1]);
    }
}
