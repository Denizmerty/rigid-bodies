#pragma once

#include <rigidbodies/ui/panel.hpp>

#include <memory>
#include <vector>

namespace rigidbodies::ui
{
    class CommandBarPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class StatusLinePanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class LibraryPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class MainMenuPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class PreferencesPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class SaveDetailsPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class ShortcutsPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class AboutPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class ConfirmationPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    // Transport controls: pause, single step, speed, and the reset that returns a scenario to its
    // starting arrangement.
    class SimulationControlsPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    // The catalogue of starting arrangements, each with the sentence describing what it shows.
    class ScenarioPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    class InspectorPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    class DrawBarPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };
    class DrawOptionsPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    class MeasurePanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;

    private:
        PlotData plot_;
        std::vector<std::vector<float>> plot_values_;
        std::vector<std::string> option_ids_, option_labels_;
        std::vector<std::string> tab_labels_;
        std::vector<OptionSpec> tab_options_, graph_quantity_options_, graph_compare_options_, graph_scope_options_, run_options_, object_options_;
    };

    // The impacts the Collisions tab lists under its Show filter ("all" or "moving"); the Measure
    // toolbar badge counts the same set.
    [[nodiscard]] std::size_t listed_impact_count(const UiModel& model, std::string_view filter);

    // One toggle per visualisation layer.
    class VisualizationPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    class GuidePanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    class BannerPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    class PerformancePanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    class PresentPanel final : public Panel
    {
    public:
        std::string_view id() const override;
        std::string_view title() const override;
        RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };
    class PresentCaptionPanel final : public Panel
    {
    public:
        std::string_view id() const override;
        std::string_view title() const override;
        RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };
    class HoverCardPanel final : public Panel
    {
    public:
        std::string_view id() const override;
        std::string_view title() const override;
        RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };
    class ContextMenuPanel final : public Panel
    {
    public:
        std::string_view id() const override;
        std::string_view title() const override;
        RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };
    class CommandSearchPanel final : public Panel
    {
    public:
        std::string_view id() const override;
        std::string_view title() const override;
        RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };
    class HintsPanel final : public Panel
    {
    public:
        std::string_view id() const override;
        std::string_view title() const override;
        RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };
    class AddMenuPanel final : public Panel
    {
    public:
        std::string_view id() const override;
        std::string_view title() const override;
        RegionId region() const override;
        void build(const UiModel&, PanelBuilder&) override;
    };

    class InteractionPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    class RenderingPanel final : public Panel
    {
    public:
        [[nodiscard]] std::string_view id() const override;
        [[nodiscard]] std::string_view title() const override;
        [[nodiscard]] RegionId region() const override;
        void build(const UiModel& model, PanelBuilder& builder) override;
    };

    // The gravity preset the world's gravity matches ("earth", "moon", "mars"), "custom" for any
    // other strength or a tilted direction, or empty while gravity is off.
    [[nodiscard]] std::string_view gravity_preset_id(const UiModel& model);

    // Whether an object counts as moving rather than resting, the one test behind every place the
    // interface names its state (Inspector, hover card, object list).
    [[nodiscard]] bool body_moving(const physics::RigidBody& body);

    // The control a guide variable or lesson step names, showing the current value of its object,
    // world setting or connection, so the Guide and the Present caption offer the same edit.
    // Returns false when the control cannot be shown as a single row (it edits structure or
    // lives on a surface of its own). `surface` keeps the copy's element apart from the
    // Inspector's row for the same setting.
    bool guide_control_row(const UiModel& model, PanelBuilder& builder, std::string_view control, std::string_view body_document_id, std::string_view instance, std::string_view label, std::string_view surface);
    [[nodiscard]] bool guide_control_fits_row(std::string_view control);
    // Whether the Guide is still waiting for this experiment's prediction, which it asks for only
    // before the first run.
    [[nodiscard]] bool prediction_pending(const UiModel& model);

    // The arrangement the application opens with.
    [[nodiscard]] std::vector<std::unique_ptr<Panel>> create_default_panels();

} // namespace rigidbodies::ui
