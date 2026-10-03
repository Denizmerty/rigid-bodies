#pragma once

#include <rigidbodies/ui/ui_backend.hpp>

namespace rigidbodies::ui
{

    // Presents the panels as docked columns drawn through the project renderer.
    //
    // This backend provides the interface without pulling in a document
    // library, and it remains useful afterwards as a dependency-free fallback and as the surface
    // the layout rules are settled against.
    class OverlayBackend final : public UiBackend
    {
    public:
        [[nodiscard]] std::string_view name() const override;
        [[nodiscard]] bool initialize(const std::filesystem::path& asset_root) override;
        void shutdown() override;

        bool handle_event(const UiEvent& event, std::vector<UiCommand>& commands) override;
        void build(const UiFrameContext& context, DrawList& list) override;
        [[nodiscard]] std::vector<UiCommand> emitted_commands() const override;

        // Width of each docked column before the display scale is applied, in pixels.
        [[nodiscard]] double column_width() const;
        void set_column_width(double value);

    private:
        void draw_panel_frame(const Rect& bounds, const Theme& theme, DrawList& list) const;

        double column_width_ { 270.0 };
        LayoutInput last_input_ {};
        double scroll_offset_ { 0.0 };
        double content_height_ { 0.0 };
        std::optional<std::size_t> active_number_;
        std::string edit_buffer_;
        std::vector<PanelRow> rows_;

        // Regions recorded while the last frame was drawn, tested against the next press. The
        // interface is rebuilt every frame, so a region is never older than one frame.
        std::vector<Hotspot> hotspots_;
        std::vector<std::size_t> hotspot_rows_;
        std::vector<Rect> panel_regions_;
        Vec2 pointer_px_ { -1.0, -1.0 };
    };

} // namespace rigidbodies::ui
