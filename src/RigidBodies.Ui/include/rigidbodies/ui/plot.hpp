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

    // Value axes only. A log10 axis keeps the series values raw, treats values at or below zero as
    // gaps, uses its range as given and ticks at whole decades.
    enum class AxisScale : std::uint8_t
    {
        linear,
        log10
    };

    // How the read-out names an x position. `speed_fraction` reads x as v/c, and
    // `rapidity_speed_fraction` reads x as a rapidity and names its v/c, so a read-out near c
    // keeps its nines and never reads as c.
    enum class AxisReadout : std::uint8_t
    {
        number,
        speed_fraction,
        rapidity_speed_fraction
    };

    struct AxisTick
    {
        double value {};
        std::string label;
    };

    // Later members default to a linear time axis, so `{ "Time", "s", 0.0, 10.0 }` still reads as
    // one. Non-empty `ticks` replace the 1-2-5 ticks (a value axis then keeps its range as given);
    // `symbol` names x in the read-out ("t" when empty).
    struct AxisSpec
    {
        std::string label, unit;
        double minimum {}, maximum {};
        AxisScale scale { AxisScale::linear };
        std::vector<AxisTick> ticks;
        std::string symbol;
        AxisReadout readout { AxisReadout::number };
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

    // A limit is a value the curves approach but never reach, such as c on a speed axis: a dashed
    // line across the plot labelled at its top, never counted as a change.
    enum class PlotMarkerKind : std::uint8_t
    {
        intervention,
        impact,
        limit
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
    // `operating_x` marks where the model is now on every series; unlike the cursor it stays
    // drawn while the learner hovers or pins, and a value off the chart becomes a chevron.
    struct PlotData
    {
        std::vector<PlotSeries> series;
        std::vector<PlotMarker> markers;
        AxisSpec x, y;
        std::optional<double> cursor_t;
        std::string empty_title, empty_detail;
        std::string note;
        std::string subject;
        std::optional<double> operating_x;
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
    // A log axis label for 10^exponent: "0.0001" to "1000" in full, "10⁴" and beyond with a
    // superscript power.
    [[nodiscard]] std::string format_plot_decade(int exponent);
    // One sentence naming what the plot shows, for assistive technology.
    [[nodiscard]] std::string plot_summary(const PlotData& data);
    // A hash of everything the plot draws from; the element rebuilds its geometry when it changes.
    [[nodiscard]] std::uint64_t plot_data_signature(const PlotData& data);

    struct PlotDiagnostics
    {
        std::size_t draw_calls {};
        std::size_t series_draw_index {};
        std::size_t series_drawn {};
        bool empty_state {};
        bool pinned {};
        std::optional<double> cursor_t;
        std::array<std::uint8_t, 4> primary_colour {};
        // On a log axis the range as drawn, with a zero step.
        PlotTicks y_ticks;
        // Content-box coordinates of the data area and of the read-out box (empty when hidden),
        // and the window position of the content box.
        std::array<float, 4> frame {};
        std::array<float, 4> tooltip {};
        std::array<float, 2> origin {};
        std::size_t marker_labels {};
        float canvas_height {};
        bool log_y {};
        std::size_t explicit_x_ticks {};
        // The read-out's first line, such as "t = 1.250 s" or "v/c = 0.99999", and the number
        // of rows below it.
        std::string cursor_header;
        std::size_t cursor_rows {};
        // Content-box x of the operating point and of the first limit, before pixel snapping.
        std::optional<float> operating_px;
        std::size_t operating_points {}, off_scale_points {};
        std::optional<float> limit_px;
        // Layout passes in the last rebuild: text that brings new glyphs into a face makes its
        // earlier text stale, and a second pass lays it out again.
        std::size_t layout_passes {};
    };

    void register_plot_element();
    void set_plot_element_data(Rml::Element* element, const PlotData* data);
    [[nodiscard]] std::size_t plot_render_count();
    void reset_plot_render_count();
    // State of the most recently rendered plot, for tests.
    [[nodiscard]] PlotDiagnostics plot_diagnostics();
#endif
}
