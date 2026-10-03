#include <rigidbodies/ui/plot.hpp>

#include <rigidbodies/ui/icons.hpp>

#include <RmlUi/Core.h>
#include <RmlUi/Core/ElementUtilities.h>
#include <RmlUi/Core/FontEngineInterface.h>
#include <RmlUi/Core/TextShapingContext.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib> // PLOTDBG
#include <cstring>
#include <functional>
#include <iterator>
#include <limits>
#include <string_view>
#include <utility>
#include <vector>

namespace rigidbodies::ui
{
    namespace
    {
        using Rml::Vector2f;
        using Colour = Rml::ColourbPremultiplied;

        std::size_t render_count {};
        PlotDiagnostics last_diagnostics;

        // Every colour, font and size the plot paints with comes from these hidden child elements,
        // so themes restyle the canvas through ordinary RCSS rules such as `.plot-series-1`.
        enum class Part : std::size_t
        {
            caption,
            key,
            tick,
            grid,
            baseline,
            series_1,
            series_2,
            series_3,
            neutral,
            marker,
            impact,
            cursor,
            cursor_pinned,
            tooltip,
            tooltip_muted,
            shadow,
            empty,
            empty_detail,
            icon,
            count
        };
        constexpr std::array<const char*, static_cast<std::size_t>(Part::count)> part_classes {
            "plot-caption", "plot-key", "plot-tick", "plot-grid", "plot-baseline", "plot-series-1", "plot-series-2", "plot-series-3", "plot-neutral", "plot-marker", "plot-impact", "plot-cursor", "plot-cursor-pinned", "plot-tooltip", "plot-tooltip-muted", "plot-shadow", "plot-empty", "plot-empty-detail", "plot-icon"
        };
        constexpr std::size_t palette_size = 3;
        constexpr float feather = 1.0f;
        constexpr const char* ellipsis = "\xE2\x80\xA6";

        const Colour clear(0, 0);

        float length(Vector2f value)
        {
            return std::sqrt(value.x * value.x + value.y * value.y);
        }
        float dot(Vector2f a, Vector2f b)
        {
            return a.x * b.x + a.y * b.y;
        }
        Vector2f perpendicular(Vector2f value)
        {
            return { -value.y, value.x };
        }

        void push(Rml::Mesh& mesh, Vector2f position, Colour colour)
        {
            mesh.vertices.push_back({ position, colour, {} });
        }
        void quad(Rml::Mesh& mesh, int a, int b, int c, int d)
        {
            for (const auto index : { a, b, c, a, c, d })
                mesh.indices.push_back(index);
        }

        void fill_rect(Rml::Mesh& mesh, float x0, float y0, float x1, float y1, Colour colour)
        {
            if (x1 <= x0 || y1 <= y0 || colour.alpha == 0)
                return;
            const auto base = static_cast<int>(mesh.vertices.size());
            push(mesh, { x0, y0 }, colour);
            push(mesh, { x1, y0 }, colour);
            push(mesh, { x1, y1 }, colour);
            push(mesh, { x0, y1 }, colour);
            quad(mesh, base, base + 1, base + 2, base + 3);
        }

        // Pixel-aligned hairlines stay crisp at every density; data lines use feathered strokes.
        void horizontal_hairline(Rml::Mesh& mesh, float x0, float x1, float y, float width, Colour colour)
        {
            const auto top = std::floor(y - width * 0.5f + 0.5f);
            fill_rect(mesh, std::floor(x0), top, std::ceil(x1), top + width, colour);
        }
        // A hairline whose opacity ramps linearly from `from` at x0 to `to` at x1.
        void horizontal_ramp(Rml::Mesh& mesh, float x0, float x1, float y, float width, Colour from, Colour to)
        {
            if (x1 - x0 < 0.5f)
                return;
            const auto top = std::floor(y - width * 0.5f + 0.5f);
            const auto base = static_cast<int>(mesh.vertices.size());
            push(mesh, { x0, top }, from);
            push(mesh, { x1, top }, to);
            push(mesh, { x1, top + width }, to);
            push(mesh, { x0, top + width }, from);
            quad(mesh, base, base + 1, base + 2, base + 3);
        }
        void vertical_hairline(Rml::Mesh& mesh, float x, float y0, float y1, float width, Colour colour, float dash = 0.0f, float gap = 0.0f)
        {
            const auto left = std::floor(x - width * 0.5f + 0.5f);
            if (dash <= 0.0f || gap <= 0.0f)
            {
                fill_rect(mesh, left, std::floor(y0), left + width, std::ceil(y1), colour);
                return;
            }
            for (auto y = std::floor(y0); y < y1; y += dash + gap)
                fill_rect(mesh, left, y, left + width, std::min(y + dash, y1), colour);
        }

        // A triangle strip with a one-pixel alpha ramp on both sides and both ends. Gentle joins
        // are mitred along the bisector; sharper ones are bevelled with one row of vertices per
        // segment, so the stroke keeps its full width through the spikes of impact data.
        void stroke_run(Rml::Mesh& mesh, const Vector2f* points, std::size_t count, Colour colour, float width, bool closed)
        {
            if (count < 2)
                return;
            const auto half = std::max(0.0f, width - feather) * 0.5f;
            const auto outer = half + feather;
            const auto edge = [&](std::size_t index)
            {
                const auto delta = points[(index + 1) % count] - points[index];
                const auto distance = length(delta);
                return distance > 1.0e-4f ? delta / distance : Vector2f { 1.0f, 0.0f };
            };
            const auto base = static_cast<int>(mesh.vertices.size());
            int rows = 0;
            const auto add_row = [&](Vector2f point, Vector2f normal, float scale)
            {
                push(mesh, point + normal * (outer * scale), clear);
                push(mesh, point + normal * (half * scale), colour);
                push(mesh, point - normal * (half * scale), colour);
                push(mesh, point - normal * (outer * scale), clear);
                ++rows;
            };
            for (std::size_t index = 0; index < count; ++index)
            {
                if (!closed && (index == 0 || index + 1 == count))
                {
                    add_row(points[index], perpendicular(edge(index == 0 ? 0 : count - 2)), 1.0f);
                    continue;
                }
                const auto before = perpendicular(edge((index + count - 1) % count));
                const auto after = perpendicular(edge(index));
                const auto sum = before + after;
                const auto sum_length = length(sum);
                if (sum_length >= 1.0e-3f)
                {
                    const auto normal = sum / sum_length;
                    const auto cosine = dot(normal, after);
                    if (cosine >= 0.5f)
                    {
                        add_row(points[index], normal, 1.0f / cosine);
                        continue;
                    }
                }
                add_row(points[index], before, 1.0f);
                add_row(points[index], after, 1.0f);
            }
            // Consecutive rows are joined by three bands; between the two rows of a bevel the
            // bands fill the wedge on the outside of the turn.
            const auto connections = closed ? rows : rows - 1;
            for (int connection = 0; connection < connections; ++connection)
            {
                const auto a = base + connection * 4;
                const auto b = base + ((connection + 1) % rows) * 4;
                for (int band = 0; band < 3; ++band)
                    quad(mesh, a + band, a + band + 1, b + band + 1, b + band);
            }
            if (closed)
                return;
            const auto cap = [&](int row, Vector2f direction)
            {
                const auto start = static_cast<int>(mesh.vertices.size());
                for (int corner = 0; corner < 4; ++corner)
                    push(mesh, mesh.vertices[static_cast<std::size_t>(row + corner)].position + direction * feather, clear);
                for (int band = 0; band < 3; ++band)
                    quad(mesh, row + band, row + band + 1, start + band + 1, start + band);
            };
            cap(base, edge(0) * -1.0f);
            cap(base + (rows - 1) * 4, edge(count - 2));
        }

        void append_point(std::vector<Vector2f>& points, Vector2f point)
        {
            if (points.empty() || length(point - points.back()) > 1.0e-3f)
                points.push_back(point);
        }

        void stroke_polyline(Rml::Mesh& mesh, const std::vector<Vector2f>& points, Colour colour, float width, float dash = 0.0f, float gap = 0.0f)
        {
            if (points.size() < 2)
                return;
            if (dash <= 0.0f || gap <= 0.0f)
            {
                stroke_run(mesh, points.data(), points.size(), colour, width, false);
                return;
            }
            std::vector<Vector2f> piece { points.front() };
            auto drawing = true;
            auto travelled = 0.0f;
            for (std::size_t index = 1; index < points.size(); ++index)
            {
                auto from = points[index - 1];
                const auto to = points[index];
                auto remaining = length(to - from);
                if (remaining <= 1.0e-5f)
                    continue;
                const auto direction = (to - from) / remaining;
                while (remaining > 0.0f)
                {
                    const auto limit = (drawing ? dash : gap) - travelled;
                    if (remaining < limit)
                    {
                        travelled += remaining;
                        if (drawing)
                            append_point(piece, to);
                        break;
                    }
                    from = from + direction * limit;
                    remaining -= limit;
                    travelled = 0.0f;
                    if (drawing)
                    {
                        append_point(piece, from);
                        stroke_run(mesh, piece.data(), piece.size(), colour, width, false);
                        piece.clear();
                    }
                    else
                        piece.assign(1, from);
                    drawing = !drawing;
                }
            }
            if (drawing && piece.size() >= 2)
                stroke_run(mesh, piece.data(), piece.size(), colour, width, false);
        }

        // Convex fill whose edge fades over `soften` pixels centred on the outline.
        void fill_convex(Rml::Mesh& mesh, const std::vector<Vector2f>& outline, Colour colour, float soften = feather)
        {
            const auto count = outline.size();
            if (count < 3 || colour.alpha == 0)
                return;
            Vector2f centre;
            for (const auto& point : outline)
                centre += point;
            centre /= static_cast<float>(count);
            const auto outward = [&](std::size_t index)
            {
                const auto a = outline[index], b = outline[(index + 1) % count];
                auto normal = perpendicular(b - a);
                const auto normal_length = length(normal);
                normal = normal_length > 1.0e-5f ? normal / normal_length : Vector2f { 0.0f, 0.0f };
                return dot(normal, (a + b) * 0.5f - centre) < 0.0f ? normal * -1.0f : normal;
            };
            const auto base = static_cast<int>(mesh.vertices.size());
            push(mesh, centre, colour);
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto sum = outward((index + count - 1) % count) + outward(index);
                const auto sum_length = length(sum);
                const auto normal = sum_length > 1.0e-5f ? sum / sum_length : Vector2f { 0.0f, 0.0f };
                const auto scale = sum_length > 1.0e-5f ? 1.0f / std::max(dot(normal, outward(index)), 0.5f) : 1.0f;
                push(mesh, outline[index] - normal * (soften * 0.5f * scale), colour);
                push(mesh, outline[index] + normal * (soften * 0.5f * scale), clear);
            }
            for (std::size_t index = 0; index < count; ++index)
            {
                const auto a = base + 1 + static_cast<int>(index) * 2;
                const auto b = base + 1 + static_cast<int>((index + 1) % count) * 2;
                mesh.indices.push_back(base);
                mesh.indices.push_back(a);
                mesh.indices.push_back(b);
                quad(mesh, a, a + 1, b + 1, b);
            }
        }

        std::vector<Vector2f> rounded_rect(float x0, float y0, float x1, float y1, float radius)
        {
            radius = std::max(0.0f, std::min(radius, std::min(x1 - x0, y1 - y0) * 0.5f));
            const auto steps = std::clamp(static_cast<int>(radius * 0.75f), 2, 8);
            const std::array<Vector2f, 4> centres { Vector2f { x1 - radius, y0 + radius }, Vector2f { x1 - radius, y1 - radius }, Vector2f { x0 + radius, y1 - radius }, Vector2f { x0 + radius, y0 + radius } };
            std::vector<Vector2f> outline;
            outline.reserve(static_cast<std::size_t>(steps + 1) * 4);
            constexpr auto quarter = 1.5707963267948966f;
            for (std::size_t corner = 0; corner < 4; ++corner)
                for (int step = 0; step <= steps; ++step)
                {
                    const auto angle = -quarter + quarter * (static_cast<float>(corner) + static_cast<float>(step) / static_cast<float>(steps));
                    outline.push_back(centres[corner] + Vector2f { std::cos(angle), std::sin(angle) } * radius);
                }
            return outline;
        }

        std::vector<Vector2f> circle(Vector2f centre, float radius)
        {
            const auto steps = std::clamp(static_cast<int>(radius * 2.5f), 12, 40);
            std::vector<Vector2f> outline;
            outline.reserve(static_cast<std::size_t>(steps));
            for (int step = 0; step < steps; ++step)
            {
                const auto angle = 6.283185307179586f * static_cast<float>(step) / static_cast<float>(steps);
                outline.push_back(centre + Vector2f { std::cos(angle), std::sin(angle) } * radius);
            }
            return outline;
        }

        struct Font
        {
            Rml::FontFaceHandle face {};
            Rml::Colourb colour;
            const Rml::String* language {};
            Rml::Style::Direction direction { Rml::Style::Direction::Auto };
            float letter_spacing {};
            float size {}, ascent {}, descent {};

            // Inter's cap height; centring on it puts digits optically on their grid line.
            [[nodiscard]] float cap() const
            {
                return size * 0.72f;
            }
            [[nodiscard]] float line() const
            {
                return std::ceil(size * 1.4f);
            }
        };

        struct TextLayer
        {
            struct Run
            {
                Rml::FontFaceHandle face {};
                Rml::TexturedMeshList meshes;
            };
            std::vector<Run> runs;
        };

        float text_width(const Font& font, const std::string& text)
        {
            if (!font.face || !font.language || text.empty())
                return 0.0f;
            const Rml::TextShapingContext context { *font.language, font.direction, font.letter_spacing };
            return static_cast<float>(Rml::GetFontEngineInterface()->GetStringWidth(font.face, text, context));
        }

        void draw_text(Rml::RenderManager& render_manager, TextLayer& layer, const Font& font, const std::string& text, Vector2f baseline, Colour colour)
        {
            if (!font.face || !font.language || text.empty())
                return;
            auto run = std::find_if(layer.runs.begin(), layer.runs.end(), [&](const TextLayer::Run& value)
                {
                    return value.face == font.face;
                });
            if (run == layer.runs.end())
            {
                layer.runs.push_back({ font.face, {} });
                run = std::prev(layer.runs.end());
            }
            const Rml::TextShapingContext context { *font.language, font.direction, font.letter_spacing };
            Rml::GetFontEngineInterface()->GenerateString(render_manager, font.face, {}, text, { std::round(baseline.x), std::round(baseline.y) }, colour, 1.0f, context, run->meshes);
        }

        std::string truncate(const Font& font, std::string text, float maximum_width)
        {
            if (text_width(font, text) <= maximum_width)
                return text;
            while (!text.empty())
            {
                // Drop one whole UTF-8 code point: its continuation bytes, then its lead byte.
                while (!text.empty() && (static_cast<unsigned char>(text.back()) & 0xC0u) == 0x80u)
                    text.pop_back();
                if (!text.empty())
                    text.pop_back();
                while (!text.empty() && text.back() == ' ')
                    text.pop_back();
                if (text_width(font, text + ellipsis) <= maximum_width)
                    return text + ellipsis;
            }
            return {};
        }

        std::uint64_t mix(std::uint64_t hash, std::uint64_t value)
        {
            return hash ^ (value + 0x9e3779b97f4a7c15ull + (hash << 6) + (hash >> 2));
        }
        std::uint64_t mix_double(std::uint64_t hash, double value)
        {
            std::uint64_t bits {};
            std::memcpy(&bits, &value, sizeof bits);
            return mix(hash, bits);
        }
        std::uint64_t mix_text(std::uint64_t hash, const std::string& value)
        {
            return mix(hash, static_cast<std::uint64_t>(std::hash<std::string> {}(value)));
        }
        std::uint64_t mix_floats(std::uint64_t hash, const float* values, std::size_t count)
        {
            if (!values)
                return mix(hash, 0);
            for (std::size_t index = 0; index < count; ++index)
            {
                std::uint32_t bits {};
                std::memcpy(&bits, values + index, sizeof bits);
                hash = mix(hash, bits);
            }
            return hash;
        }

        std::uint64_t data_signature(const PlotData& data)
        {
            std::uint64_t hash = 0x51ed270b27f3a5c1ull;
            for (const auto* axis : { &data.x, &data.y })
            {
                hash = mix_text(mix_text(hash, axis->label), axis->unit);
                hash = mix_double(mix_double(hash, axis->minimum), axis->maximum);
            }
            hash = mix_text(mix_text(mix_text(mix_text(hash, data.empty_title), data.empty_detail), data.note), data.subject);
            hash = mix_double(mix(hash, data.cursor_t.has_value()), data.cursor_t.value_or(0.0));
            for (const auto& series : data.series)
            {
                hash = mix_text(mix_text(mix_text(hash, series.label), series.source), series.unit);
                hash = mix(mix(mix(hash, static_cast<std::uint64_t>(series.style)), series.slot), series.count);
                hash = mix_floats(mix_floats(hash, series.times, series.count), series.values, series.count);
            }
            for (const auto& marker : data.markers)
                hash = mix(mix_text(mix_double(hash, marker.time_s), marker.label), static_cast<std::uint64_t>(marker.kind));
            return hash;
        }

        int step_decimals(double step)
        {
            if (!(step > 0.0) || !std::isfinite(step))
                return 0;
            return std::clamp(static_cast<int>(-std::floor(std::log10(step) + 1.0e-9)), 0, 9);
        }

        std::vector<double> tick_values(const PlotTicks& ticks)
        {
            std::vector<double> values;
            if (!(ticks.step > 0.0))
                return values;
            // A range may end between steps; its ticks are the whole steps inside it.
            const auto first = std::ceil(ticks.minimum / ticks.step - 1.0e-6) * ticks.step;
            const auto count = static_cast<int>(std::floor((ticks.maximum - first) / ticks.step + 1.0e-6));
            for (int index = 0; index <= std::min(count, 64); ++index)
                values.push_back(first + ticks.step * index);
            return values;
        }

        // Large axes read better with an SI multiplier than with long digit runs.
        std::string tick_label(double value, const PlotTicks& ticks)
        {
            const auto magnitude = std::max(std::abs(ticks.minimum), std::abs(ticks.maximum));
            if (magnitude >= 1.0e7)
                return format_plot_number(value / 1.0e6, step_decimals(ticks.step / 1.0e6)) + "M";
            if (magnitude >= 1.0e4)
                return format_plot_number(value / 1.0e3, step_decimals(ticks.step / 1.0e3)) + "k";
            return format_plot_number(value, ticks.decimals);
        }

        bool finite_sample(const PlotSeries& series, std::size_t index)
        {
            return std::isfinite(series.times[index]) && std::isfinite(series.values[index]);
        }

        bool drawable(const PlotSeries& series)
        {
            if (!series.times || !series.values || series.count < 2)
                return false;
            std::size_t finite = 0;
            for (std::size_t index = 0; index < series.count && finite < 2; ++index)
                finite += finite_sample(series, index) ? 1u : 0u;
            return finite >= 2;
        }

        bool samples_in_window(const PlotSeries& series, double x0, double x1)
        {
            if (!series.times || !series.values)
                return false;
            std::size_t inside = 0;
            for (std::size_t index = 0; index < series.count && inside < 2; ++index)
                inside += finite_sample(series, index) && series.times[index] >= x0 - 1.0e-9 && series.times[index] <= x1 + 1.0e-9 ? 1u : 0u;
            return inside >= 2;
        }

        std::size_t primary_samples(const PlotData& data)
        {
            for (const auto& series : data.series)
                if (series.style == SeriesStyle::solid)
                    return series.count;
            return 0;
        }

        std::string axis_caption(const AxisSpec& axis)
        {
            if (axis.unit.empty())
                return axis.label;
            return axis.label.empty() ? axis.unit : axis.label + " (" + axis.unit + ")";
        }

        std::optional<double> value_at(const PlotSeries& series, double time_s)
        {
            if (!series.times || !series.values || series.count == 0)
                return {};
            const auto* end = series.times + series.count;
            if (time_s < series.times[0] - 1.0e-9 || time_s > static_cast<double>(*(end - 1)) + 1.0e-9)
                return {};
            const auto* found = std::lower_bound(series.times, end, time_s, [](float sample, double value)
                {
                    return static_cast<double>(sample) < value;
                });
            const auto after = static_cast<std::size_t>(std::min<std::ptrdiff_t>(found - series.times, static_cast<std::ptrdiff_t>(series.count - 1)));
            const auto before = after > 0 ? after - 1 : 0;
            const double t0 = series.times[before], t1 = series.times[after];
            const double v0 = series.values[before], v1 = series.values[after];
            if (std::isfinite(v0) && std::isfinite(v1))
            {
                const auto fraction = t1 > t0 ? std::clamp((time_s - t0) / (t1 - t0), 0.0, 1.0) : 1.0;
                return v0 + (v1 - v0) * fraction;
            }
            const auto nearest = std::abs(time_s - t0) <= std::abs(t1 - time_s) ? v0 : v1;
            if (std::isfinite(nearest))
                return nearest;
            return {};
        }

        // Keeps the first, lowest, highest and last sample of every pixel column, so a dense
        // record collapses to a few vertices per column without losing impact spikes.
        std::vector<Vector2f> decimate(const std::vector<Vector2f>& points, std::size_t budget)
        {
            if (points.size() <= budget)
            {
                std::vector<Vector2f> kept;
                kept.reserve(points.size());
                for (const auto& point : points)
                    if (kept.empty() || length(point - kept.back()) > 0.35f)
                        kept.push_back(point);
                if (kept.size() == 1 && points.size() > 1)
                    kept.push_back(points.back());
                return kept;
            }
            std::vector<Vector2f> kept;
            kept.reserve(budget * 2);
            std::size_t start = 0;
            while (start < points.size())
            {
                const auto column = std::floor(points[start].x);
                auto end = start, lowest = start, highest = start;
                while (end < points.size() && std::floor(points[end].x) == column)
                {
                    if (points[end].y > points[lowest].y)
                        lowest = end;
                    if (points[end].y < points[highest].y)
                        highest = end;
                    ++end;
                }
                std::array<std::size_t, 4> order { start, lowest, highest, end - 1 };
                std::sort(order.begin(), order.end());
                for (std::size_t index = 0; index < order.size(); ++index)
                    if (index == 0 || order[index] != order[index - 1])
                        append_point(kept, points[order[index]]);
                start = end;
            }
            return kept;
        }

        struct Compiled
        {
            Rml::Geometry geometry;
            Rml::Texture texture;
            bool clip_to_plot {};
        };

        struct LegendItem
        {
            std::string text;
            Colour colour;
            SeriesStyle style { SeriesStyle::solid };
            float width {};
            bool note {};
        };

        struct TooltipRow
        {
            std::string label;
            std::vector<std::string> values;
            Colour colour;
            SeriesStyle style { SeriesStyle::solid };
            bool marker {};
            std::uint8_t slot {};
        };

        class PlotElement final : public Rml::Element
        {
        public:
            explicit PlotElement(const Rml::String& tag) : Element(tag), listener_(*this)
            {
                for (const auto id : pointer_events)
                    AddEventListener(id, &listener_);
            }
            ~PlotElement() override
            {
                for (const auto id : pointer_events)
                    RemoveEventListener(id, &listener_);
            }
            PlotElement(const PlotElement&) = delete;
            PlotElement& operator=(const PlotElement&) = delete;

            void set_data(const PlotData* value)
            {
                const auto signature = value ? data_signature(*value) : 0;
                if (value != data_ || signature != data_signature_)
                {
                    data_ = value;
                    data_signature_ = signature;
                    dirty_ = true;
                    // A record that starts over (a reset or a new run) makes a pinned time stale.
                    const auto samples = value ? primary_samples(*value) : 0;
                    if (samples < primary_samples_)
                        pinned_t_.reset();
                    primary_samples_ = samples;
                }
            }

        protected:
            void OnUpdate() override
            {
                if (parts_.empty())
                {
                    if (auto* document = GetOwnerDocument())
                        for (const auto* name : part_classes)
                        {
                            auto part = document->CreateElement("plotpart");
                            part->SetClass(name, true);
                            part->SetProperty("display", "none");
                            parts_.push_back(AppendChild(std::move(part)));
                        }
                }
                fit_to_container();
            }

            void OnResize() override
            {
                dirty_ = true;
            }

            void OnRender() override
            {
                ++render_count;
                auto* render_manager = GetRenderManager();
                if (!render_manager)
                    return;
                const auto size = GetBox().GetSize(Rml::BoxArea::Content);
                if (size.x <= 2.0f || size.y <= 2.0f)
                    return;
                const auto absolute = GetAbsoluteOffset(Rml::BoxArea::Content);
                const Vector2f origin { std::round(absolute.x), std::round(absolute.y) };
                const auto styles = style_signature();
                // Geometry is local to the content box, so scrolling only matters to a hover
                // cursor that follows the pointer.
                if (size != size_ || (origin != origin_ && hover_visible_) || styles != style_signature_ || fonts_changed())
                    dirty_ = true;
                origin_ = origin;
                if (dirty_)
                {
                    size_ = size;
                    style_signature_ = styles;
                    rebuild(*render_manager);
                }

                const auto previous_state = render_manager->GetState();
                Rml::ElementUtilities::SetClippingRegion(this, true);
                const auto canvas_clip = render_manager->GetScissorRegion();
                auto plot_clip = Rml::Rectanglei::FromCorners(
                    { static_cast<int>(std::floor(origin.x + frame_.left)), static_cast<int>(std::floor(origin.y + frame_.top - 2.0f)) },
                    { static_cast<int>(std::ceil(origin.x + frame_.right)), static_cast<int>(std::ceil(origin.y + frame_.bottom + 2.0f)) });
                if (canvas_clip.Valid())
                    plot_clip = plot_clip.Intersect(canvas_clip);
                auto clipped = false;
                for (const auto& item : compiled_)
                {
                    if (item.clip_to_plot != clipped)
                    {
                        clipped = item.clip_to_plot;
                        if (clipped)
                            render_manager->SetScissorRegion(plot_clip);
                        else if (canvas_clip.Valid())
                            render_manager->SetScissorRegion(canvas_clip);
                        else
                            render_manager->DisableScissorRegion();
                    }
                    if (!clipped || plot_clip.Valid())
                        item.geometry.Render(origin, item.texture);
                }
                render_manager->SetState(previous_state);
                last_diagnostics = diagnostics_;
            }

        private:
            static constexpr std::array<Rml::EventId, 3> pointer_events { Rml::EventId::Mousemove, Rml::EventId::Mouseout, Rml::EventId::Click };

            class PointerListener final : public Rml::EventListener
            {
            public:
                explicit PointerListener(PlotElement& owner) : owner_(owner)
                {
                }
                void ProcessEvent(Rml::Event& event) override
                {
                    owner_.on_pointer(event);
                }

            private:
                PlotElement& owner_;
            };

            struct Frame
            {
                float left {}, right {}, top {}, bottom {};
                double x0 {}, x1 {};
                bool valid {};
            };

            // With a max-height in RCSS the canvas grows to fill the visible height of its
            // scrolling region below its top, so dragging the drawer taller gives a taller graph.
            void fit_to_container()
            {
                const auto& computed = GetComputedValues();
                const auto maximum = computed.max_height();
                const auto enabled = maximum.type == Rml::Style::LengthPercentage::Length && maximum.value < 1.0e6f;
                Rml::Element* container = nullptr;
                if (enabled)
                    for (auto* ancestor = GetParentNode(); ancestor && !container; ancestor = ancestor->GetParentNode())
                    {
                        const auto overflow = ancestor->GetComputedValues().overflow_y();
                        if (overflow == Rml::Style::Overflow::Auto || overflow == Rml::Style::Overflow::Scroll)
                            container = ancestor;
                    }
                if (!container || container->GetClientHeight() <= 0.0f)
                {
                    if (fitted_ && !enabled)
                    {
                        RemoveProperty(Rml::PropertyId::Height);
                        fitted_ = false;
                    }
                    return;
                }
                auto reserve = GetBox().GetEdge(Rml::BoxArea::Margin, Rml::BoxEdge::Bottom) + container->GetBox().GetEdge(Rml::BoxArea::Padding, Rml::BoxEdge::Bottom);
                if (auto* parent = GetParentNode(); parent && parent != container)
                    reserve += parent->GetBox().GetEdge(Rml::BoxArea::Padding, Rml::BoxEdge::Bottom);
                const auto top = GetAbsoluteOffset(Rml::BoxArea::Border).y - container->GetAbsoluteOffset(Rml::BoxArea::Padding).y + container->GetScrollTop();
                const auto minimum = computed.min_height().type == Rml::Style::LengthPercentage::Length ? computed.min_height().value : 0.0f;
                const auto wanted = std::floor(std::clamp(container->GetClientHeight() - top - reserve, minimum, std::max(minimum, maximum.value)));
                if (std::abs(wanted - GetBox().GetSize(Rml::BoxArea::Border).y) >= 1.0f)
                {
                    SetProperty(Rml::PropertyId::Height, Rml::Property(wanted, Rml::Unit::PX));
                    fitted_ = true;
                }
            }

            [[nodiscard]] bool pin_in_window() const
            {
                return pinned_t_ && frame_.valid && *pinned_t_ >= frame_.x0 - 1.0e-9 && *pinned_t_ <= frame_.x1 + 1.0e-9;
            }

            void on_pointer(Rml::Event& event)
            {
                const Vector2f mouse { event.GetParameter("mouse_x", 0.0f), event.GetParameter("mouse_y", 0.0f) };
                switch (event.GetId())
                {
                case Rml::EventId::Mousemove:
                    pointer_ = mouse;
                    // A pinned cursor ignores the pointer, so moving over it changes nothing.
                    if (!pin_in_window() && (inside_plot(mouse) || hover_visible_))
                        dirty_ = true;
                    break;
                case Rml::EventId::Mouseout:
                    if (event.GetTargetElement() == this)
                    {
                        pointer_.reset();
                        dirty_ = dirty_ || hover_visible_;
                    }
                    break;
                case Rml::EventId::Click:
                    if (event.GetParameter("button", 0) != 0)
                        break;
                    pointer_ = mouse;
                    if (pin_in_window() || !inside_plot(mouse))
                        pinned_t_.reset();
                    else
                        pinned_t_ = snap_time(time_at(mouse.x - origin_.x));
                    dirty_ = true;
                    break;
                default:
                    break;
                }
            }

            [[nodiscard]] bool inside_plot(Vector2f mouse) const
            {
                const auto local = mouse - origin_;
                return frame_.valid && local.x >= frame_.left && local.x <= frame_.right && local.y >= frame_.top && local.y <= frame_.bottom;
            }
            [[nodiscard]] double time_at(float local_x) const
            {
                const auto fraction = std::clamp((local_x - frame_.left) / std::max(1.0f, frame_.right - frame_.left), 0.0f, 1.0f);
                return frame_.x0 + (frame_.x1 - frame_.x0) * static_cast<double>(fraction);
            }

            // Hovering snaps to recorded samples so the read-out shows measured values, preferring
            // the current run over comparisons.
            [[nodiscard]] double snap_time(double time_s) const
            {
                if (!data_)
                    return time_s;
                const PlotSeries* reference = nullptr;
                for (const auto pass : { true, false })
                    for (const auto& series : data_->series)
                        if (!reference && (series.style == SeriesStyle::solid) == pass && drawable(series) && time_s >= series.times[0] && time_s <= series.times[series.count - 1])
                            reference = &series;
                if (!reference)
                    return time_s;
                const auto* end = reference->times + reference->count;
                const auto* found = std::lower_bound(reference->times, end, time_s, [](float sample, double value)
                    {
                        return static_cast<double>(sample) < value;
                    });
                if (found == end)
                    return static_cast<double>(*(end - 1));
                if (found != reference->times && std::abs(static_cast<double>(*(found - 1)) - time_s) < std::abs(static_cast<double>(*found) - time_s))
                    --found;
                return static_cast<double>(*found);
            }

            [[nodiscard]] Rml::Element* part(Part value) const
            {
                const auto index = static_cast<std::size_t>(value);
                return index < parts_.size() ? parts_[index] : nullptr;
            }
            [[nodiscard]] Rml::Colourb colour_of(Part value) const
            {
                if (const auto* element = part(value))
                    return element->GetComputedValues().color();
                return GetComputedValues().color();
            }
            [[nodiscard]] Font font_of(Part value) const
            {
                Font font;
                const auto* element = part(value);
                if (!element)
                    return font;
                const auto& computed = element->GetComputedValues();
                font.face = element->GetFontFaceHandle();
                font.colour = computed.color();
                font.language = &computed.language();
                font.direction = computed.direction();
                font.letter_spacing = computed.letter_spacing();
                if (font.face)
                {
                    const auto& metrics = Rml::GetFontEngineInterface()->GetFontMetrics(font.face);
                    font.size = static_cast<float>(metrics.size);
                    font.ascent = metrics.ascent;
                    font.descent = metrics.descent;
                }
                return font;
            }

            [[nodiscard]] std::uint64_t style_signature() const
            {
                std::uint64_t hash = mix(0, GetComputedValues().background_color().red);
                hash = mix(mix(mix(hash, GetComputedValues().background_color().green), GetComputedValues().background_color().blue), GetComputedValues().background_color().alpha);
                for (const auto* element : parts_)
                {
                    const auto& computed = element->GetComputedValues();
                    for (const auto colour : { computed.color(), computed.background_color(), computed.border_top_color() })
                        hash = mix(hash, (static_cast<std::uint64_t>(colour.red) << 24) | (static_cast<std::uint64_t>(colour.green) << 16) | (static_cast<std::uint64_t>(colour.blue) << 8) | colour.alpha);
                    hash = mix(hash, static_cast<std::uint64_t>(element->GetFontFaceHandle()));
                }
                return mix(hash, parts_.size());
            }

            [[nodiscard]] bool fonts_changed() const
            {
                for (const auto& [face, version] : font_versions_)
                    if (Rml::GetFontEngineInterface()->GetVersion(face) != version)
                        return true;
                return false;
            }

            void rebuild(Rml::RenderManager& render_manager)
            {
                dirty_ = false;
                compiled_.clear();
                diagnostics_ = {};
                frame_ = {};
                hover_visible_ = false;

                const PlotData empty_data;
                const auto& data = data_ ? *data_ : empty_data;
                const auto dp = std::max(0.5f, Rml::ElementUtilities::GetDensityIndependentPixelRatio(this));
                const auto hair = std::max(1.0f, std::floor(dp));
                const auto width = size_.x, height = size_.y;
                diagnostics_.canvas_height = height;

                const auto caption_font = font_of(Part::caption), key_font = font_of(Part::key), tick_font = font_of(Part::tick);
                const auto marker_font = font_of(Part::marker), tooltip_font = font_of(Part::tooltip), muted_font = font_of(Part::tooltip_muted);
                const auto empty_font = font_of(Part::empty), detail_font = font_of(Part::empty_detail);
                const auto colour = [&](Part value, float alpha = 1.0f)
                {
                    return colour_of(value).ToPremultiplied(alpha);
                };
                const auto slot_part = [](std::uint8_t slot)
                {
                    return static_cast<Part>(static_cast<std::size_t>(Part::series_1) + slot % palette_size);
                };
                const auto source_name = [](const std::string& source)
                {
                    return source.empty() ? std::string("This run") : source;
                };
                // Comparisons stay at full colour so they keep their contrast; the dash pattern
                // and a slightly thinner stroke mark them as secondary.
                const auto dash_of = [&](SeriesStyle style)
                {
                    if (style == SeriesStyle::dashed)
                        return std::pair { 5.0f * dp, 3.5f * dp };
                    if (style == SeriesStyle::dotted)
                        return std::pair { 1.75f * dp, 3.0f * dp };
                    return std::pair { 0.0f, 0.0f };
                };
                const auto stroke_width_of = [&](SeriesStyle style)
                {
                    return (style == SeriesStyle::dashed ? 1.75f : 2.0f) * dp;
                };

                Rml::Mesh chrome, guides, lines, overlay, tooltip_mesh;
                TextLayer labels, tooltip_text;
                const auto swatch = [&](Rml::Mesh& mesh, float x, float y, float length_px, Colour stroke_colour, SeriesStyle style)
                {
                    const auto [dash, gap] = dash_of(style);
                    stroke_polyline(mesh, { { x, y }, { x + length_px, y } }, stroke_colour, stroke_width_of(style), dash * 0.8f, gap * 0.8f);
                };

                std::vector<const PlotSeries*> visible;
                for (const auto& series : data.series)
                    if (drawable(series))
                        visible.push_back(&series);

                auto x0 = data.x.minimum, x1 = data.x.maximum;
                if (!(x1 > x0) || !std::isfinite(x0) || !std::isfinite(x1))
                {
                    x0 = std::numeric_limits<double>::infinity();
                    x1 = -x0;
                    for (const auto* series : visible)
                        for (std::size_t index = 0; index < series->count; ++index)
                            if (std::isfinite(series->times[index]))
                            {
                                x0 = std::min(x0, static_cast<double>(series->times[index]));
                                x1 = std::max(x1, static_cast<double>(series->times[index]));
                            }
                    if (!(x1 > x0))
                    {
                        x0 = 0.0;
                        x1 = 1.0;
                    }
                }
                // A record whose samples all lie outside the window (just after Clear graph, say)
                // has nothing to draw there, so it counts as empty rather than as a blank axis.
                visible.erase(std::remove_if(visible.begin(), visible.end(), [&](const PlotSeries* series)
                                  {
                                      return !samples_in_window(*series, x0, x1);
                                  }),
                    visible.end());
                const auto has_data = !visible.empty();

                auto y_low = data.y.minimum, y_high = data.y.maximum;
                if (!(y_high > y_low) || !std::isfinite(y_low) || !std::isfinite(y_high))
                {
                    y_low = std::numeric_limits<double>::infinity();
                    y_high = -y_low;
                    for (const auto* series : visible)
                        for (std::size_t index = 0; index < series->count; ++index)
                            if (finite_sample(*series, index) && series->times[index] >= x0 && series->times[index] <= x1)
                            {
                                y_low = std::min(y_low, static_cast<double>(series->values[index]));
                                y_high = std::max(y_high, static_cast<double>(series->values[index]));
                            }
                    if (!(y_high >= y_low))
                        y_low = y_high = 0.0;
                }

                // Header: the y quantity names the chart; a legend appears only when colour or
                // line style distinguishes more than one thing.
                const auto caption = axis_caption(data.y);
                std::vector<std::uint8_t> slots;
                std::vector<std::pair<std::string, SeriesStyle>> sources;
                for (const auto* series : visible)
                {
                    if (std::find(slots.begin(), slots.end(), static_cast<std::uint8_t>(series->slot % palette_size)) == slots.end())
                        slots.push_back(static_cast<std::uint8_t>(series->slot % palette_size));
                    if (std::none_of(sources.begin(), sources.end(), [&](const auto& source)
                            {
                                return source.first == series->source;
                            }))
                        sources.emplace_back(series->source, series->style);
                }
                const auto multiple_slots = slots.size() > 1, multiple_sources = sources.size() > 1;
                const auto swatch_length = 16.0f * dp, swatch_gap = 6.0f * dp, item_gap = 14.0f * dp;
                std::vector<LegendItem> legend;
                if (multiple_slots)
                    for (const auto slot : slots)
                        for (const auto* series : visible)
                            if (series->slot % palette_size == slot)
                            {
                                legend.push_back({ series->label, colour(slot_part(slot)), SeriesStyle::solid, 0.0f, false });
                                break;
                            }
                if (multiple_sources)
                    for (const auto& [source, style] : sources)
                        legend.push_back({ source_name(source), multiple_slots ? colour(Part::neutral) : colour(slot_part(slots.front())), style, 0.0f, false });
                if (has_data && !data.note.empty())
                    legend.push_back({ data.note, clear, SeriesStyle::solid, 0.0f, true });
                auto legend_width = 0.0f;
                for (auto& item : legend)
                {
                    const auto lead = item.note ? 0.0f : swatch_length + swatch_gap;
                    item.text = truncate(key_font, item.text, std::max(40.0f * dp, width - lead));
                    item.width = lead + text_width(key_font, item.text);
                    legend_width += item.width + (legend_width > 0.0f ? item_gap : 0.0f);
                }
                const auto header_line = std::max(caption_font.line(), key_font.line());
                // The subject follows the quantity in the lighter key style: "Speed (m/s) · Ball".
                const auto subject = data.subject.empty() ? std::string {} : caption.empty() ? data.subject
                                                                                             : " · " + data.subject;
                const auto has_title = !caption.empty() || !subject.empty();
                const auto caption_width = text_width(caption_font, caption) + text_width(key_font, subject);
                auto header_height = 0.0f;
                {
                    auto line_y = 0.0f;
                    if (has_title)
                    {
                        const auto shown_caption = truncate(caption_font, caption, width);
                        const auto shown_width = text_width(caption_font, shown_caption);
                        const auto baseline = line_y + (header_line + caption_font.cap()) * 0.5f;
                        draw_text(render_manager, labels, caption_font, shown_caption, { 0.0f, baseline }, caption_font.colour.ToPremultiplied());
                        if (!subject.empty() && shown_caption == caption && width - shown_width > 24.0f * dp)
                            draw_text(render_manager, labels, key_font, truncate(key_font, subject, width - shown_width), { shown_width, baseline }, key_font.colour.ToPremultiplied());
                        header_height = header_line;
                    }
                    auto x = 0.0f;
                    if (!legend.empty())
                    {
                        if (has_title && caption_width + 20.0f * dp + legend_width <= width)
                            x = width - legend_width;
                        else if (has_title)
                            line_y += header_line;
                        header_height = line_y + header_line;
                    }
                    for (const auto& item : legend)
                    {
                        if (x > 0.0f && x + item.width > width + 0.5f)
                        {
                            x = 0.0f;
                            line_y += header_line;
                            header_height = line_y + header_line;
                        }
                        const auto middle = line_y + header_line * 0.5f;
                        if (item.note)
                            draw_text(render_manager, labels, key_font, item.text, { x, middle + key_font.cap() * 0.5f }, tick_font.colour.ToPremultiplied());
                        else
                        {
                            swatch(chrome, x, std::round(middle), swatch_length, item.colour, item.style);
                            draw_text(render_manager, labels, key_font, item.text, { x + swatch_length + swatch_gap, middle + key_font.cap() * 0.5f }, key_font.colour.ToPremultiplied());
                        }
                        x += item.width + item_gap;
                    }
                }

                // Labelled interventions get a lane of their own above the data, so a label never
                // hides the moment the data reacts to it.
                std::vector<const PlotMarker*> labelled;
                if (has_data)
                    for (const auto& marker : data.markers)
                        if (marker.kind == PlotMarkerKind::intervention && !marker.label.empty() && marker.time_s >= x0 && marker.time_s <= x1)
                            labelled.push_back(&marker);
                std::stable_sort(labelled.begin(), labelled.end(), [](const PlotMarker* a, const PlotMarker* b)
                    {
                        return a->time_s < b->time_s;
                    });
                const auto lane_height = labelled.empty() ? 0.0f : marker_font.line();
                const auto lane_top = header_height > 0.0f ? header_height + 6.0f * dp : 2.0f * dp;
                const auto tick_height = tick_font.ascent + tick_font.descent;
                const auto x_label_gap = 6.0f * dp;
                if (lane_height > 0.0f)
                    frame_.top = lane_top + lane_height + 6.0f * dp;
                else
                    frame_.top = header_height > 0.0f ? header_height + 10.0f * dp : std::ceil(tick_font.cap() * 0.5f) + 2.0f * dp;
                frame_.bottom = std::max(frame_.top + 8.0f * dp, height - tick_height - x_label_gap);
                frame_.x0 = x0;
                frame_.x1 = x1;
                const auto plot_height = frame_.bottom - frame_.top;

                if (!has_data)
                {
                    frame_.left = 0.0f;
                    frame_.right = width;
                    const auto title = data.empty_title.empty() ? std::string("No data recorded yet") : data.empty_title;
                    const auto title_text = truncate(empty_font, title, width - 16.0f * dp);
                    const auto detail_text = truncate(detail_font, data.empty_detail, width - 16.0f * dp);
                    const auto icon_font = font_of(Part::icon);
                    const auto glyph = icons::utf8(icons::measure);
                    const auto text_block = empty_font.line() + (detail_text.empty() ? 0.0f : detail_font.line());
                    const auto icon_block = icon_font.face && text_block + icon_font.line() + 12.0f * dp <= plot_height ? icon_font.line() : 0.0f;
                    const auto block = icon_block + text_block;
                    const auto top = frame_.top + (plot_height - block) * 0.5f;
                    const auto title_width = text_width(empty_font, title_text), detail_width = text_width(detail_font, detail_text);

                    // A faint placeholder grid that fades out evenly on both sides of the message.
                    const auto half_gap = std::max(title_width, detail_width) * 0.5f + 20.0f * dp;
                    const auto fade = std::min(48.0f * dp, width * 0.12f);
                    const auto gap_left = std::max(0.0f, std::floor(width * 0.5f - half_gap)), gap_right = std::min(width, std::ceil(width * 0.5f + half_gap));
                    const auto rule = [&](float y, Colour line_colour)
                    {
                        const auto fade_left = std::max(0.0f, gap_left - fade), fade_right = std::min(width, gap_right + fade);
                        horizontal_hairline(chrome, 0.0f, fade_left, y, hair, line_colour);
                        horizontal_ramp(chrome, fade_left, gap_left, y, hair, line_colour, clear);
                        horizontal_ramp(chrome, gap_right, fade_right, y, hair, clear, line_colour);
                        horizontal_hairline(chrome, fade_right, width, y, hair, line_colour);
                    };
                    for (int line = 0; line < 4; ++line)
                        rule(frame_.top + plot_height * static_cast<float>(line) / 4.0f, colour(Part::grid, 0.7f));
                    horizontal_hairline(chrome, 0.0f, width, frame_.bottom, hair, colour(Part::baseline, 0.6f));
                    if (icon_block > 0.0f)
                        draw_text(render_manager, labels, icon_font, glyph, { (width - text_width(icon_font, glyph)) * 0.5f, top + (icon_block + icon_font.ascent - icon_font.descent) * 0.5f }, icon_font.colour.ToPremultiplied());
                    const auto text_top = top + icon_block;
                    draw_text(render_manager, labels, empty_font, title_text, { (width - title_width) * 0.5f, text_top + (empty_font.line() + empty_font.cap()) * 0.5f }, empty_font.colour.ToPremultiplied());
                    draw_text(render_manager, labels, detail_font, detail_text, { (width - detail_width) * 0.5f, text_top + empty_font.line() + (detail_font.line() + detail_font.cap()) * 0.5f }, detail_font.colour.ToPremultiplied());
                    diagnostics_.empty_state = true;
                    pinned_t_.reset();
                }
                else
                {
                    // Value axis: whole 1-2-5 steps at most about 30 dp apart, and at least three
                    // intervals once the labels have room, so a short plot still resolves its data.
                    const auto y_target = std::clamp(static_cast<int>(std::ceil(plot_height / (30.0f * dp))), plot_height >= 45.0f * dp ? 3 : 2, 8);
                    const auto y_ticks = plot_ticks(y_low, y_high, y_target, true);
                    const auto y_values = tick_values(y_ticks);
                    std::vector<std::string> y_labels;
                    auto gutter = 0.0f;
                    for (const auto value : y_values)
                    {
                        y_labels.push_back(tick_label(value, y_ticks));
                        gutter = std::max(gutter, text_width(tick_font, y_labels.back()));
                    }
                    frame_.left = std::ceil(gutter + 8.0f * dp);
                    frame_.right = std::floor(width - 2.0f * dp);
                    frame_.valid = true;
                    const auto plot_width = std::max(1.0f, frame_.right - frame_.left);
                    const auto y_span = y_ticks.maximum - y_ticks.minimum;
                    // Data that would lie on the outermost line of a non-zero edge gets a few dp
                    // of room beyond it, so neither the line nor the axis hides under the data.
                    const auto touches = [&](double gap)
                    {
                        return gap / y_span * plot_height < 3.0f * dp;
                    };
                    const auto pad_top = y_ticks.maximum != 0.0 && touches(y_ticks.maximum - y_high) ? 6.0f * dp : 0.0f;
                    const auto pad_bottom = y_ticks.minimum != 0.0 && touches(y_low - y_ticks.minimum) ? 6.0f * dp : 0.0f;
                    const auto value_height = std::max(1.0f, plot_height - pad_top - pad_bottom);
                    const auto to_x = [&](double time_s)
                    {
                        return frame_.left + static_cast<float>((time_s - x0) / (x1 - x0)) * plot_width;
                    };
                    const auto to_y = [&](double value)
                    {
                        return frame_.bottom - pad_bottom - static_cast<float>((value - y_ticks.minimum) / y_span) * value_height;
                    };
                    diagnostics_.y_ticks = y_ticks;

                    for (std::size_t index = 0; index < y_values.size(); ++index)
                    {
                        const auto y = to_y(y_values[index]);
                        if (y < frame_.bottom - 0.5f)
                            horizontal_hairline(chrome, frame_.left, frame_.right, y, hair, colour(std::abs(y_values[index]) < y_ticks.step * 1.0e-6 ? Part::baseline : Part::grid));
                        const auto label_width = text_width(tick_font, y_labels[index]);
                        draw_text(render_manager, labels, tick_font, y_labels[index], { frame_.left - 8.0f * dp - label_width, y + tick_font.cap() * 0.5f }, tick_font.colour.ToPremultiplied());
                    }
                    horizontal_hairline(chrome, frame_.left, frame_.right, frame_.bottom, hair, colour(Part::baseline));
                    vertical_hairline(chrome, frame_.left, frame_.top, frame_.bottom + hair, hair, colour(Part::baseline));

                    // Time axis: keep labels at least 16 dp apart, widening the step if needed.
                    auto x_target = std::max(1, static_cast<int>(plot_width / (84.0f * dp)));
                    PlotTicks x_ticks;
                    std::vector<double> x_values;
                    std::vector<std::string> x_labels;
                    const auto x_suffix = data.x.unit.empty() ? std::string {} : " " + data.x.unit;
                    for (int attempt = 0; attempt < 8; ++attempt)
                    {
                        x_ticks = plot_ticks(x0, x1, x_target, false);
                        x_values = tick_values(x_ticks);
                        x_labels.clear();
                        auto widest = 0.0f;
                        for (const auto value : x_values)
                        {
                            x_labels.push_back(format_plot_number(value, x_ticks.decimals) + x_suffix);
                            widest = std::max(widest, text_width(tick_font, x_labels.back()));
                        }
                        const auto spacing = static_cast<float>(x_ticks.step / (x1 - x0)) * plot_width;
                        if (widest + 16.0f * dp <= spacing || x_target <= 1)
                            break;
                        --x_target;
                    }
                    auto previous_right = -1.0e9f;
                    for (std::size_t index = 0; index < x_values.size(); ++index)
                    {
                        const auto x = to_x(x_values[index]);
                        vertical_hairline(chrome, x, frame_.bottom, frame_.bottom + 4.0f * dp, hair, colour(Part::baseline));
                        const auto label_width = text_width(tick_font, x_labels[index]);
                        const auto left = std::clamp(x - label_width * 0.5f, frame_.left - 2.0f * dp, width - label_width);
                        if (left < previous_right + 8.0f * dp)
                            continue;
                        draw_text(render_manager, labels, tick_font, x_labels[index], { left, frame_.bottom + x_label_gap + tick_font.ascent }, tick_font.colour.ToPremultiplied());
                        previous_right = left + label_width;
                    }

                    // Event guides sit under the data. Impacts are faint dotted lines, merged when
                    // they crowd together; interventions are dashed and labelled in their lane.
                    auto impact_x = -1.0e9f;
                    for (const auto& marker : data.markers)
                    {
                        if (marker.kind != PlotMarkerKind::impact || marker.time_s < x0 || marker.time_s > x1)
                            continue;
                        const auto x = to_x(marker.time_s);
                        if (std::abs(x - impact_x) < 6.0f * dp)
                            continue;
                        vertical_hairline(guides, x, frame_.top, frame_.bottom, hair, colour(Part::impact), hair, 3.0f * dp);
                        impact_x = x;
                    }
                    const auto lane_middle = std::round(lane_top + lane_height * 0.5f);
                    const auto flag_radius = 2.5f * dp;
                    for (const auto& marker : data.markers)
                    {
                        if (marker.kind != PlotMarkerKind::intervention || marker.time_s < x0 || marker.time_s > x1)
                            continue;
                        const auto x = to_x(marker.time_s);
                        const auto in_lane = lane_height > 0.0f && !marker.label.empty();
                        vertical_hairline(guides, x, in_lane ? lane_middle + flag_radius : frame_.top, frame_.bottom, hair, colour(Part::marker, 0.8f), 3.0f * dp, 3.0f * dp);
                        if (in_lane)
                            fill_convex(guides, circle({ std::floor(x - hair * 0.5f + 0.5f) + hair * 0.5f, lane_middle }, flag_radius), colour(Part::marker));
                    }
                    // A label sits right of its flag, or left when that side has room for all of
                    // it; when neither does it is shortened to the wider side rather than dropped.
                    auto labels_right = -1.0e9f;
                    for (std::size_t index = 0; index < labelled.size(); ++index)
                    {
                        const auto x = to_x(labelled[index]->time_s);
                        const auto inset = flag_radius + 4.0f * dp;
                        const auto right_limit = index + 1 < labelled.size() ? to_x(labelled[index + 1]->time_s) - inset - 4.0f * dp : width;
                        const auto right_room = right_limit - (x + inset);
                        const auto left_room = (x - inset) - std::max(0.0f, labels_right + 10.0f * dp);
                        const auto full = text_width(marker_font, labelled[index]->label);
                        const auto left_side = full > right_room && (full <= left_room || left_room > right_room);
                        const auto room = left_side ? left_room : right_room;
                        const auto text = room >= 24.0f * dp ? truncate(marker_font, labelled[index]->label, room) : std::string {};
                        if (text.empty())
                        {
                            labels_right = std::max(labels_right, x + flag_radius);
                            continue;
                        }
                        const auto text_w = text_width(marker_font, text);
                        const auto left = left_side ? x - inset - text_w : x + inset;
                        draw_text(render_manager, labels, marker_font, text, { left, lane_middle + marker_font.cap() * 0.5f }, marker_font.colour.ToPremultiplied());
                        labels_right = std::max(labels_right, std::max(left + text_w, x + flag_radius));
                        ++diagnostics_.marker_labels;
                    }

                    // Data: comparisons first so the current run stays on top.
                    std::size_t solid_count = 0;
                    for (const auto* series : visible)
                        solid_count += series->style == SeriesStyle::solid ? 1u : 0u;
                    const auto budget = static_cast<std::size_t>(plot_width * 1.5f) + 8u;
                    for (const auto pass : { false, true })
                        for (const auto* series : visible)
                        {
                            if ((series->style == SeriesStyle::solid) != pass)
                                continue;
                            const auto stroke_colour = colour(slot_part(series->slot));
                            const auto pattern = dash_of(series->style);
                            const auto stroke_width = stroke_width_of(series->style);
                            const auto fill = series->style == SeriesStyle::solid && solid_count == 1 && y_ticks.minimum >= 0.0;
                            std::vector<Vector2f> run;
                            const auto flush = [&]
                            {
                                const auto points = decimate(run, budget);
                                if (fill && points.size() >= 2)
                                {
                                    const auto base_colour = colour_of(slot_part(series->slot));
                                    for (std::size_t index = 1; index < points.size(); ++index)
                                    {
                                        const auto a = points[index - 1], b = points[index];
                                        const auto shade = [&](float y)
                                        {
                                            return base_colour.ToPremultiplied(0.16f * std::clamp((frame_.bottom - y) / plot_height, 0.0f, 1.0f));
                                        };
                                        const auto start = static_cast<int>(lines.vertices.size());
                                        push(lines, a, shade(a.y));
                                        push(lines, b, shade(b.y));
                                        push(lines, { b.x, frame_.bottom }, clear);
                                        push(lines, { a.x, frame_.bottom }, clear);
                                        quad(lines, start, start + 1, start + 2, start + 3);
                                    }
                                }
                                stroke_polyline(lines, points, stroke_colour, stroke_width, pattern.first, pattern.second);
                                run.clear();
                            };
                            const auto first = static_cast<std::size_t>(std::lower_bound(series->times, series->times + series->count, x0, [](float sample, double value)
                                                                            {
                                                                                return static_cast<double>(sample) < value;
                                                                            }) -
                                series->times);
                            for (std::size_t index = first > 0 ? first - 1 : 0; index < series->count; ++index)
                            {
                                if (!finite_sample(*series, index))
                                {
                                    flush();
                                    continue;
                                }
                                run.push_back({ to_x(series->times[index]), to_y(series->values[index]) });
                                if (series->times[index] > x1)
                                    break;
                            }
                            flush();
                            ++diagnostics_.series_drawn;
                            if (series->style == SeriesStyle::solid && diagnostics_.primary_colour == std::array<std::uint8_t, 4> {})
                            {
                                const auto value = colour_of(slot_part(series->slot));
                                diagnostics_.primary_colour = { value.red, value.green, value.blue, value.alpha };
                            }
                        }

                    // Cursor: a pinned time wins over hover, which wins over a time supplied by
                    // the panel. A pin the window has moved past is released.
                    // PLOTDBG begin
                    if (const char* debug_hover = std::getenv("RB_PLOT_HOVER"))
                    {
                        float fx {}, fy {};
                        if (std::sscanf(debug_hover, "%f,%f", &fx, &fy) == 2)
                            pointer_ = origin_ + Vector2f { frame_.left + (frame_.right - frame_.left) * fx, frame_.top + (frame_.bottom - frame_.top) * fy };
                    }
                    if (const char* debug_pin = std::getenv("RB_PLOT_PIN"))
                        pinned_t_ = snap_time(time_at(frame_.left + (frame_.right - frame_.left) * static_cast<float>(std::atof(debug_pin))));
                    // PLOTDBG end
                    if (pinned_t_ && !pin_in_window())
                        pinned_t_.reset();
                    std::optional<double> cursor_t;
                    if (pinned_t_)
                        cursor_t = pinned_t_;
                    else if (pointer_ && inside_plot(*pointer_))
                    {
                        cursor_t = snap_time(time_at(pointer_->x - origin_.x));
                        hover_visible_ = true;
                    }
                    else if (data.cursor_t)
                        cursor_t = data.cursor_t;
                    diagnostics_.pinned = pinned_t_.has_value();
                    if (cursor_t && *cursor_t >= x0 - 1.0e-9 && *cursor_t <= x1 + 1.0e-9)
                    {
                        diagnostics_.cursor_t = cursor_t;
                        const auto x = to_x(*cursor_t);
                        if (pinned_t_)
                        {
                            // The pinned cursor is drawn in the accent colour with a handle on
                            // top, so it reads as placed rather than as a passing hover.
                            const auto accent = colour(Part::cursor_pinned);
                            const auto line_x = std::floor(x - hair * 0.5f + 0.5f) + hair * 0.5f;
                            vertical_hairline(overlay, x, frame_.top, frame_.bottom, hair, accent);
                            fill_convex(overlay, { { line_x - 4.5f * dp, frame_.top - 6.0f * dp }, { line_x + 4.5f * dp, frame_.top - 6.0f * dp }, { line_x, frame_.top } }, accent);
                        }
                        else
                            vertical_hairline(overlay, x, frame_.top, frame_.bottom, hair, colour(Part::cursor));
                        const auto ring = GetComputedValues().background_color().ToPremultiplied();
                        // Several quantities over several runs read as a quantity-by-run table
                        // rather than one row per line.
                        const auto table = multiple_slots && multiple_sources;
                        std::vector<std::pair<std::string, SeriesStyle>> columns;
                        if (table)
                            for (const auto& [source, style] : sources)
                                columns.emplace_back(source_name(source), style);
                        else
                            columns.emplace_back(std::string {}, SeriesStyle::solid);
                        std::vector<TooltipRow> rows;
                        std::vector<float> point_ys;
                        for (const auto* series : visible)
                        {
                            const auto value = value_at(*series, *cursor_t);
                            if (!value)
                                continue;
                            const auto stroke_colour = colour(slot_part(series->slot));
                            const Vector2f point { x, to_y(*value) };
                            if (point.y >= frame_.top - 4.0f * dp && point.y <= frame_.bottom + 4.0f * dp)
                            {
                                fill_convex(overlay, circle(point, 4.5f * dp), ring);
                                fill_convex(overlay, circle(point, 3.0f * dp), stroke_colour);
                                point_ys.push_back(point.y);
                            }
                            const auto& unit = series->unit.empty() ? data.y.unit : series->unit;
                            auto text = format_plot_value(*value) + (unit.empty() ? std::string {} : " " + unit);
                            const auto slot = static_cast<std::uint8_t>(series->slot % palette_size);
                            if (table)
                            {
                                const auto column = static_cast<std::size_t>(std::distance(sources.begin(), std::find_if(sources.begin(), sources.end(), [&](const auto& source)
                                                                                                                {
                                                                                                                    return source.first == series->source;
                                                                                                                })));
                                auto row = std::find_if(rows.begin(), rows.end(), [&](const TooltipRow& value)
                                    {
                                        return value.slot == slot;
                                    });
                                if (row == rows.end())
                                {
                                    rows.push_back({ series->label, std::vector<std::string>(columns.size()), colour(slot_part(slot)), SeriesStyle::solid, false, slot });
                                    row = std::prev(rows.end());
                                }
                                if (column < row->values.size())
                                    row->values[column] = std::move(text);
                                continue;
                            }
                            auto label = series->label;
                            if (multiple_sources)
                                label = source_name(series->source);
                            rows.push_back({ label, { std::move(text) }, stroke_colour, series->style, false, slot });
                        }
                        if (table)
                            std::stable_sort(rows.begin(), rows.end(), [&](const TooltipRow& a, const TooltipRow& b)
                                {
                                    return std::find(slots.begin(), slots.end(), a.slot) < std::find(slots.begin(), slots.end(), b.slot);
                                });
                        std::size_t marker_rows = 0;
                        for (const auto& marker : data.markers)
                            if (marker_rows < 2 && !marker.label.empty() && std::abs(to_x(marker.time_s) - x) <= 4.0f * dp)
                            {
                                rows.push_back({ marker.label, {}, colour(marker.kind == PlotMarkerKind::impact ? Part::impact : Part::marker), SeriesStyle::solid, true, 0 });
                                ++marker_rows;
                            }

                        // Millisecond precision matches the recorder's 25 ms cadence and keeps the
                        // read-out width steady while the cursor moves.
                        const auto header = "t = " + format_plot_number(*cursor_t, 3) + x_suffix;
                        const auto pinned_text = std::string(pinned_t_ ? "Pinned" : "");
                        const auto natural_row = std::ceil(std::max(tooltip_font.size, muted_font.size) * 1.5f);
                        // A read-out taller than most of the plot packs its rows more tightly.
                        const auto compact = pad_rows_height(natural_row, rows.size(), 7.0f * dp) > plot_height * 0.6f;
                        const auto pad_x = (compact ? 8.0f : 10.0f) * dp, pad_y = (compact ? 4.0f : 7.0f) * dp;
                        const auto row_height = compact ? std::ceil(std::max(tooltip_font.size, muted_font.size) * 1.3f) : natural_row;
                        const auto key_length = 12.0f * dp, key_gap = 7.0f * dp, column_gap = (compact ? 12.0f : 16.0f) * dp, column_key = 10.0f * dp;
                        const auto label_limit = std::max(60.0f * dp, std::min(200.0f * dp, width * (table ? 0.3f : 0.45f)));
                        const auto empty_cell = std::string("\xE2\x80\x94");
                        auto label_width = 0.0f;
                        std::vector<float> column_widths(columns.size(), 0.0f);
                        for (std::size_t column = 0; column < columns.size() && table; ++column)
                            column_widths[column] = column_key + 4.0f * dp + text_width(muted_font, columns[column].first);
                        for (auto& row : rows)
                        {
                            row.label = truncate(muted_font, row.label, label_limit);
                            label_width = std::max(label_width, text_width(muted_font, row.label));
                            for (std::size_t column = 0; column < row.values.size() && column < columns.size(); ++column)
                                column_widths[column] = std::max(column_widths[column], text_width(tooltip_font, row.values[column].empty() ? empty_cell : row.values[column]));
                        }
                        auto columns_width = 0.0f;
                        for (const auto value : column_widths)
                            columns_width += value > 0.0f ? column_gap + value : 0.0f;
                        const auto time_width = text_width(tooltip_font, header);
                        const auto pinned_width = pinned_text.empty() ? 0.0f : text_width(muted_font, pinned_text);
                        const auto lead_width = rows.empty() ? 0.0f : key_length + key_gap + label_width;
                        const auto content_width = table ? std::max(time_width + (pinned_width > 0.0f ? 8.0f * dp + pinned_width : 0.0f), lead_width) + columns_width
                                                         : std::max(time_width + (pinned_width > 0.0f ? column_gap + pinned_width : 0.0f), lead_width + columns_width);
                        const auto box_width = std::ceil(pad_x * 2.0f + content_width);
                        const auto box_height = std::ceil(pad_y * 2.0f + row_height * static_cast<float>(rows.size() + 1));

                        // Beside the cursor on the side with room, inside the data area, and at
                        // the top or bottom away from the points being read.
                        const auto offset = 12.0f * dp, inset = 4.0f * dp;
                        const auto room_right = frame_.right - inset - (x + offset), room_left = (x - offset) - (frame_.left + inset);
                        auto box_left = x + offset;
                        if (box_width > room_right)
                            box_left = box_width <= room_left || room_left > room_right ? x - offset - box_width : frame_.right - inset - box_width;
                        box_left = std::round(std::clamp(box_left, std::min(frame_.left + inset, std::max(0.0f, width - box_width)), std::max(0.0f, width - box_width)));
                        auto mean_y = (frame_.top + frame_.bottom) * 0.5f;
                        if (!point_ys.empty())
                        {
                            mean_y = 0.0f;
                            for (const auto y : point_ys)
                                mean_y += y;
                            mean_y /= static_cast<float>(point_ys.size());
                        }
                        // A read-out taller than the data area rises over the header rather than
                        // over the time axis.
                        const auto lower = std::max(0.0f, frame_.bottom - inset - box_height);
                        const auto upper = std::min(frame_.top + inset, lower);
                        const auto box_top = std::round(mean_y < (frame_.top + frame_.bottom) * 0.5f ? lower : upper);
                        diagnostics_.tooltip = { box_left, box_top, box_left + box_width, box_top + box_height };

                        const auto radius = 6.0f * dp;
                        fill_convex(tooltip_mesh, rounded_rect(box_left, box_top + 2.0f * dp, box_left + box_width, box_top + box_height + 2.0f * dp, radius), colour(Part::shadow), 10.0f * dp);
                        const auto* tooltip_part = part(Part::tooltip);
                        const auto background = tooltip_part ? tooltip_part->GetComputedValues().background_color() : Rml::Colourb(32, 32, 32, 255);
                        const auto border = tooltip_part ? tooltip_part->GetComputedValues().border_top_color() : Rml::Colourb(255, 255, 255, 40);
                        const auto outline = rounded_rect(box_left + 0.5f, box_top + 0.5f, box_left + box_width - 0.5f, box_top + box_height - 0.5f, radius);
                        fill_convex(tooltip_mesh, outline, background.ToPremultiplied());
                        stroke_run(tooltip_mesh, outline.data(), outline.size(), border.ToPremultiplied(), 1.0f, true);
                        std::vector<float> column_right(columns.size(), box_left + box_width - pad_x);
                        for (auto column = columns.size(); column-- > 1;)
                            column_right[column - 1] = column_right[column] - column_widths[column] - column_gap;
                        auto row_top = box_top + pad_y;
                        const auto header_middle = row_top + row_height * 0.5f;
                        draw_text(render_manager, tooltip_text, tooltip_font, header, { box_left + pad_x, header_middle + tooltip_font.cap() * 0.5f }, tooltip_font.colour.ToPremultiplied());
                        if (!pinned_text.empty())
                        {
                            const auto pinned_left = table ? box_left + pad_x + time_width + 8.0f * dp : box_left + box_width - pad_x - pinned_width;
                            draw_text(render_manager, tooltip_text, muted_font, pinned_text, { pinned_left, header_middle + muted_font.cap() * 0.5f }, colour(Part::cursor_pinned));
                        }
                        for (std::size_t column = 0; column < columns.size() && table; ++column)
                        {
                            const auto text_left = column_right[column] - text_width(muted_font, columns[column].first);
                            swatch(tooltip_mesh, text_left - 4.0f * dp - column_key, std::round(header_middle), column_key, colour(Part::neutral), columns[column].second);
                            draw_text(render_manager, tooltip_text, muted_font, columns[column].first, { text_left, header_middle + muted_font.cap() * 0.5f }, muted_font.colour.ToPremultiplied());
                        }
                        for (const auto& row : rows)
                        {
                            row_top += row_height;
                            const auto middle = row_top + row_height * 0.5f;
                            if (row.marker)
                                vertical_hairline(tooltip_mesh, box_left + pad_x + key_length * 0.5f, middle - 5.0f * dp, middle + 5.0f * dp, std::max(hair, std::round(1.5f * dp)), row.colour);
                            else
                                swatch(tooltip_mesh, box_left + pad_x, std::round(middle), key_length, row.colour, row.style);
                            draw_text(render_manager, tooltip_text, muted_font, row.label, { box_left + pad_x + key_length + key_gap, middle + muted_font.cap() * 0.5f }, muted_font.colour.ToPremultiplied());
                            for (std::size_t column = 0; column < row.values.size() && column < columns.size(); ++column)
                            {
                                const auto missing = row.values[column].empty();
                                const auto& text = missing ? empty_cell : row.values[column];
                                const auto& font = missing ? muted_font : tooltip_font;
                                draw_text(render_manager, tooltip_text, font, text, { column_right[column] - text_width(font, text), middle + font.cap() * 0.5f }, font.colour.ToPremultiplied());
                            }
                        }
                    }
                }
                diagnostics_.frame = { frame_.left, frame_.top, frame_.right, frame_.bottom };
                const auto add_mesh = [&](Rml::Mesh& mesh, bool clip)
                {
                    if (mesh)
                        compiled_.push_back({ render_manager.MakeGeometry(std::move(mesh)), {}, clip });
                };
                const auto add_text = [&](TextLayer& layer)
                {
                    for (auto& run : layer.runs)
                        for (auto& textured : run.meshes)
                            if (textured.mesh)
                                compiled_.push_back({ render_manager.MakeGeometry(std::move(textured.mesh)), textured.texture, false });
                };
                add_mesh(chrome, false);
                add_mesh(guides, false);
                diagnostics_.series_draw_index = compiled_.size();
                add_mesh(lines, true);
                add_text(labels);
                add_mesh(overlay, false);
                add_mesh(tooltip_mesh, false);
                add_text(tooltip_text);
                diagnostics_.draw_calls = compiled_.size();

                font_versions_.clear();
                for (const auto* layer : { &labels, &tooltip_text })
                    for (const auto& run : layer->runs)
                        if (std::none_of(font_versions_.begin(), font_versions_.end(), [&](const auto& entry)
                                {
                                    return entry.first == run.face;
                                }))
                            font_versions_.emplace_back(run.face, Rml::GetFontEngineInterface()->GetVersion(run.face));
            }

            static float pad_rows_height(float row_height, std::size_t rows, float pad_y)
            {
                return pad_y * 2.0f + row_height * static_cast<float>(rows + 1);
            }

            PointerListener listener_;
            const PlotData* data_ {};
            std::uint64_t data_signature_ {}, style_signature_ {};
            std::size_t primary_samples_ {};
            std::vector<Rml::Element*> parts_;
            std::vector<Compiled> compiled_;
            std::vector<std::pair<Rml::FontFaceHandle, int>> font_versions_;
            Vector2f size_ {}, origin_ {};
            Frame frame_;
            std::optional<Vector2f> pointer_;
            std::optional<double> pinned_t_;
            bool hover_visible_ {};
            bool fitted_ {};
            bool dirty_ { true };
            PlotDiagnostics diagnostics_;
        };

        Rml::ElementInstancerGeneric<PlotElement> plot_instancer;

        // Interval count of `step` over the range, counting whole steps outward or inward.
        double tick_intervals(double minimum, double maximum, double step, bool extend)
        {
            if (extend)
                return std::ceil(maximum / step - 1.0e-9) - std::floor(minimum / step + 1.0e-9);
            return std::floor(maximum / step + 1.0e-9) - std::ceil(minimum / step - 1.0e-9);
        }
    }

    PlotTicks plot_ticks(double minimum, double maximum, int target_count, bool extend)
    {
        if (!std::isfinite(minimum) || !std::isfinite(maximum))
        {
            minimum = 0.0;
            maximum = 1.0;
        }
        if (maximum < minimum)
            std::swap(minimum, maximum);
        const auto magnitude = std::max(std::abs(minimum), std::abs(maximum));
        if (magnitude < plot_zero_floor)
        {
            minimum = 0.0;
            maximum = 1.0;
        }
        else if (maximum - minimum <= magnitude * 1.0e-6)
        {
            minimum -= magnitude * 0.1;
            maximum += magnitude * 0.1;
        }
        // Score each 1-2-5 step by how far its interval count is from the target (too many
        // costs more than too few, so labels never crowd) and, for a widened value axis, by how
        // much of the axis the data leaves empty. Emptiness weighs in quadratically: a sliver of
        // headroom is harmless, but a third of the plot left blank is not.
        const auto target = static_cast<double>(std::max(1, target_count));
        const auto base = std::pow(10.0, std::floor(std::log10((maximum - minimum) / target)));
        auto step = base;
        auto best_cost = std::numeric_limits<double>::infinity();
        for (const auto factor : { 0.5, 1.0, 2.0, 5.0, 10.0, 20.0 })
        {
            const auto candidate = factor * base;
            const auto intervals = tick_intervals(minimum, maximum, candidate, extend);
            if (intervals < 1.0 && extend)
                continue;
            auto cost = (intervals > target ? 1.5 : 1.0) * std::abs(intervals - target) / target + (intervals < 1.0 ? 100.0 : 0.0);
            if (extend)
            {
                const auto empty = 1.0 - (maximum - minimum) / (intervals * candidate);
                cost += 8.0 * empty * empty;
            }
            if (cost < best_cost)
            {
                best_cost = cost;
                step = candidate;
            }
        }
        PlotTicks ticks;
        ticks.step = step;
        if (extend)
        {
            ticks.minimum = std::floor(minimum / step + 1.0e-9) * step;
            ticks.maximum = std::ceil(maximum / step - 1.0e-9) * step;
            // Whole steps that would leave much of a short axis blank give way to the data and a
            // little headroom; the ticks then fall inside the range.
            const auto span = maximum - minimum;
            if (1.0 - span / (ticks.maximum - ticks.minimum) > 0.2)
            {
                const auto headroom = 0.05 * span;
                ticks.minimum = std::max(ticks.minimum, minimum - headroom);
                ticks.maximum = std::min(ticks.maximum, maximum + headroom);
            }
        }
        else
        {
            ticks.minimum = std::ceil(minimum / step - 1.0e-9) * step;
            ticks.maximum = std::floor(maximum / step + 1.0e-9) * step;
        }
        if (ticks.minimum == 0.0)
            ticks.minimum = 0.0;
        ticks.decimals = step_decimals(step);
        return ticks;
    }

    std::string format_plot_number(double value, int decimals)
    {
        if (!std::isfinite(value))
            return "\xE2\x80\x94";
        std::array<char, 64> buffer {};
        std::snprintf(buffer.data(), buffer.size(), "%.*f", std::clamp(decimals, 0, 9), value);
        std::string text = buffer.data();
        if (!text.empty() && text.front() == '-')
        {
            if (text.find_first_not_of("0.", 1) == std::string::npos)
                text.erase(0, 1);
            else
                text.replace(0, 1, "\xE2\x88\x92");
        }
        return text;
    }

    std::string format_plot_value(double value)
    {
        if (!std::isfinite(value))
            return format_plot_number(value, 0);
        if (std::abs(value) < plot_zero_floor)
            return "0";
        const auto exponent = static_cast<int>(std::floor(std::log10(std::abs(value))));
        auto decimals = std::clamp(2 - exponent, 0, 6);
        // Rounding can carry into a new digit (9.996 to 10.00), which needs one decimal fewer.
        const auto scale = std::pow(10.0, decimals);
        if (decimals > 0 && std::round(std::abs(value) * scale) / scale >= std::pow(10.0, exponent + 1))
            --decimals;
        return format_plot_number(value, decimals);
    }

    std::string plot_summary(const PlotData& data)
    {
        std::vector<std::string> names;
        const auto windowed = data.x.maximum > data.x.minimum;
        for (const auto& series : data.series)
        {
            if (!drawable(series) || (windowed && !samples_in_window(series, data.x.minimum, data.x.maximum)))
                continue;
            auto name = series.label + (series.source.empty() || series.source == "This run" ? std::string {} : " (" + series.source + ")");
            if (std::find(names.begin(), names.end(), name) == names.end())
                names.push_back(std::move(name));
        }
        const auto caption = axis_caption(data.y);
        if (names.empty())
            return "Empty graph" + (caption.empty() ? std::string {} : " of " + caption) + (data.empty_title.empty() ? std::string(".") : ". " + data.empty_title + ".");
        std::string text = "Graph of " + (caption.empty() ? std::string("values") : caption) + " against " + (data.x.label.empty() ? std::string("time") : data.x.label);
        text += data.x.unit.empty() ? std::string {} : " (" + data.x.unit + ")";
        if (!data.subject.empty())
            text += " for " + (data.subject.rfind("Whole scene", 0) == 0 ? "the whole scene" + data.subject.substr(11) : data.subject);
        text += ", showing ";
        for (std::size_t index = 0; index < names.size(); ++index)
            text += (index == 0 ? "" : index + 1 == names.size() ? " and "
                                                                 : ", ") +
                names[index];
        std::size_t impacts = 0, interventions = 0;
        for (const auto& marker : data.markers)
            (marker.kind == PlotMarkerKind::impact ? impacts : interventions) += 1;
        text += ".";
        if (interventions > 0)
            text += " " + std::to_string(interventions) + (interventions == 1 ? " change" : " changes") + " marked.";
        if (impacts > 0)
            text += " " + std::to_string(impacts) + (impacts == 1 ? " impact" : " impacts") + " marked.";
        if (!data.note.empty())
            text += " " + data.note;
        return text;
    }

    void register_plot_element()
    {
        Rml::Factory::RegisterElementInstancer("plot", &plot_instancer);
    }

    void set_plot_element_data(Rml::Element* element, const PlotData* data)
    {
        if (auto* plot = dynamic_cast<PlotElement*>(element))
            plot->set_data(data);
    }

    std::size_t plot_render_count()
    {
        return render_count;
    }

    void reset_plot_render_count()
    {
        render_count = 0;
    }

    PlotDiagnostics plot_diagnostics()
    {
        return last_diagnostics;
    }
}
