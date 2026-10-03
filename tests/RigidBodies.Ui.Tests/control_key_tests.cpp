#include <rigidbodies/ui/control_spec.hpp>
#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/physics/scenario.hpp>

#include "test_framework.hpp"

#include <filesystem>
#include <fstream>
#include <set>

namespace
{
    using namespace rigidbodies;

    class MeasurementDevice final : public render::RenderDevice
    {
    public:
        std::string_view backend_name() const override
        {
            return "test";
        }
        render::ViewportSize drawable_size() const override
        {
            return { 1600, 900 };
        }
        float display_scale() const override
        {
            return 1.0f;
        }
        void set_vertical_sync(bool) override
        {
        }
        void begin_frame(const render::Color&) override
        {
        }
        void submit(const render::DrawList&) override
        {
        }
        void end_frame() override
        {
        }
        float measure_text_width(std::string_view text, float scale) const override
        {
            return static_cast<float>(text.size()) * 7.0f * scale;
        }
        float text_line_height(float scale) const override
        {
            return 15.0f * scale;
        }
        const std::string& last_error() const override
        {
            return error_;
        }

    private:
        std::string error_;
    };

    RIGIDBODIES_TEST("stylesheets use semantic row classes")
    {
        for (const auto& entry : std::filesystem::recursive_directory_iterator(RIGIDBODIES_SOURCE_ASSETS "/ui"))
        {
            if (!entry.is_regular_file() || entry.path().extension() != ".rcss")
                continue;
            std::ifstream input(entry.path(), std::ios::binary);
            const std::string text { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
            RIGIDBODIES_EXPECT(text.find("kind-") == std::string::npos, "no stylesheet contains a numeric kind selector");
        }
    }

    RIGIDBODIES_TEST("element ids are reversible stable mappings of host key and instance")
    {
        RIGIDBODIES_EXPECT(ui::element_id("inspector", "object.properties.mass") == "inspector-object-properties-mass", "dots map to hyphens");
        RIGIDBODIES_EXPECT(ui::element_id("guide", "object.properties.mass", "Steel Ball #1") == "guide-object-properties-mass@_teel__all__1", "instance punctuation is normalized without changing the key, after a separator no child part uses");
    }

    RIGIDBODIES_TEST("legacy interactive rows carry unique well formed final keys")
    {
        physics::World world;
        RIGIDBODIES_EXPECT(physics::load_scenario(world, "free_fall"), "fixture scenario loads");
        ui::UiModel model;
        model.world = &world;
        model.scenario_id = "free_fall";
        model.scenario_title = "Free fall";
        for (const auto id : world.body_ids())
            if (const auto* body = world.find_body(id); body && body->type() == physics::BodyType::dynamic_body)
            {
                model.selection = id;
                model.selected_bodies = { id };
                break;
            }

        MeasurementDevice device;
        render::Theme theme;
        render::DrawList list;
        std::vector<ui::Hotspot> hotspots;
        for (auto& panel : ui::create_default_panels())
        {
            std::set<std::string> identities;
            std::vector<ui::PanelRow> rows;
            ui::PanelBuilder builder { theme, device, 1.0f, { {}, { 1000, 100000 } }, list, hotspots };
            builder.record_rows(rows);
            panel->build(model, builder);
            for (const auto& row : rows)
            {
                const auto interactive = row.kind == ui::PanelRowKind::action || (row.kind >= ui::PanelRowKind::number && row.kind != ui::PanelRowKind::notice && row.kind != ui::PanelRowKind::readout && row.kind != ui::PanelRowKind::plot);
                if (!interactive)
                    continue;
                RIGIDBODIES_EXPECT(!row.key.empty() && ui::valid_control_key(row.key), "interactive row has a valid semantic key: " + row.text);
                RIGIDBODIES_EXPECT(identities.insert(row.key + "#" + row.instance).second, "interactive host/key/instance identity is unique: " + row.key);
            }
        }
    }

    RIGIDBODIES_TEST("every registry key has one command template")
    {
        std::set<std::string_view> keys;
        for (const auto& spec : ui::control_specs())
        {
            RIGIDBODIES_EXPECT(keys.insert(spec.key).second, "registry key occurs exactly once");
            const auto view_owned = spec.kind == ui::ControlKind::checklist || spec.kind == ui::ControlKind::select ||
                spec.kind == ui::ControlKind::segmented || spec.kind == ui::ControlKind::checkbox ||
                spec.kind == ui::ControlKind::tabs || spec.kind == ui::ControlKind::section ||
                spec.kind == ui::ControlKind::readout || spec.kind == ui::ControlKind::list;
            RIGIDBODIES_EXPECT(spec.command != ui::UiCommandKind::none || view_owned, "interactive registry entry has a command template or is owned by ViewState");
            RIGIDBODIES_EXPECT(ui::find_control_spec(spec.key) == &spec, "binary lookup returns the canonical entry");
        }
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
