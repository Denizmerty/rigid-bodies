#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace Rml
{
    class Element;
}

namespace rigidbodies::ui
{
    enum class RowTone
    {
        normal,
        positive,
        info,
        warning,
        danger,
        muted
    };

    enum class SeriesStyle : std::uint8_t
    {
        solid,
        dashed,
        dotted
    };

    struct AxisSpec
    {
        std::string label, unit;
        double minimum {}, maximum {};
    };

    // Colour follows `slot`, so one quantity keeps its colour across runs; `style` and `source`
    // tell the runs apart. Dashed and dotted series are drawn as secondary comparisons.
    struct PlotSeries
    {
        std::string label;
        const float* values {};
        const float* times {};
        std::size_t count {};
        SeriesStyle style { SeriesStyle::solid };
        std::uint8_t slot {};
        std::string source;
        std::string unit;
    };

    enum class PlotMarkerKind : std::uint8_t
    {
        intervention,
        impact
    };

    struct PlotMarker
    {
        double time_s {};
        std::string label;
        PlotMarkerKind kind { PlotMarkerKind::intervention };
    };

    // The y range is the data extent to cover; the plot widens it to whole tick steps where that
    // leaves little of the axis blank, and otherwise to a little headroom. The x
    // range is shown exactly. `cursor_t` places a cursor until the learner hovers or pins one.
    // `note` is a short muted remark beside the legend, such as why a chosen series has no data.
    // `subject` names what the values belong to (the whole scene or one object) after the title.
    struct PlotData
    {
        std::vector<PlotSeries> series;
        std::vector<PlotMarker> markers;
        AxisSpec x, y;
        std::optional<double> cursor_t;
        std::string empty_title, empty_detail;
        std::string note;
        std::string subject;
    };

    struct PlotTicks
    {
        double minimum {}, maximum {}, step {};
        int decimals {};
    };

    // Ranges whose largest magnitude is below this are numerical noise around zero.
    inline constexpr double plot_zero_floor = 1.0e-6;

#if defined(RIGIDBODIES_HAS_RMLUI)
    // Ticks on a 1-2-5 sequence with an interval count close to `target_count`, preferring fewer.
    // `extend` widens the range outward to whole steps (value axes), or only by a little headroom
    // where whole steps would leave over a fifth of the axis blank, and weighs how much of the
    // axis the data fills; otherwise the range is kept. Ticks fall inside the range.
    [[nodiscard]] PlotTicks plot_ticks(double minimum, double maximum, int target_count, bool extend);
    // Fixed decimals with a true minus sign and no negative zero.
    [[nodiscard]] std::string format_plot_number(double value, int decimals);
    // Three significant figures, independent of the axis, for read-outs.
    [[nodiscard]] std::string format_plot_value(double value);
    // One sentence naming what the plot shows, for assistive technology.
    [[nodiscard]] std::string plot_summary(const PlotData& data);

    struct PlotDiagnostics
    {
        std::size_t draw_calls {};
        std::size_t series_draw_index {};
        std::size_t series_drawn {};
        bool empty_state {};
        bool pinned {};
        std::optional<double> cursor_t;
        std::array<std::uint8_t, 4> primary_colour {};
        PlotTicks y_ticks;
        // Content-box coordinates of the data area and of the read-out box (empty when hidden).
        std::array<float, 4> frame {};
        std::array<float, 4> tooltip {};
        std::size_t marker_labels {};
        float canvas_height {};
    };

    void register_plot_element();
    void set_plot_element_data(Rml::Element* element, const PlotData* data);
    [[nodiscard]] std::size_t plot_render_count();
    void reset_plot_render_count();
    // State of the most recently rendered plot, for tests.
    [[nodiscard]] PlotDiagnostics plot_diagnostics();
#endif
}
