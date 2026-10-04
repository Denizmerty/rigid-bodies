#include <rigidbodies/ui/document_backend.hpp>
#include <rigidbodies/ui/panels.hpp>
#include <rigidbodies/render/sdl_render_device.hpp>
#include <rigidbodies/physics/scenario.hpp>
#include <SDL3/SDL.h>
#include "test_framework.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace
{
    using namespace rigidbodies;
    struct Fixture
    {
        SDL_Surface* surface { nullptr };
        render::RenderDevicePtr device;
        ui::DocumentBackend backend;
        physics::World world;
        ui::UiModel model;
        render::Theme theme;
        render::DrawList list;
        std::vector<std::unique_ptr<ui::Panel>> panels;
        ui::ViewState view;
        ui::LayoutResult layout;
        float current_scale { 1.0f };
        Fixture(int width = 1600, int height = 900)
        {
            surface = SDL_CreateSurface(width, height, SDL_PIXELFORMAT_RGBA32);
            RIGIDBODIES_EXPECT(surface, "window-free image surface is available");
            device = render::SdlRenderDevice::adopt_software_renderer(SDL_CreateSoftwareRenderer(surface));
            RIGIDBODIES_EXPECT(device != nullptr, "window-free software device is available");
            RIGIDBODIES_EXPECT(backend.initialize(RIGIDBODIES_SOURCE_ASSETS), "real RmlUi document and proportional fonts initialize");
            RIGIDBODIES_EXPECT(physics::load_scenario(world, "free_fall"), "reference scenario loads");
            model.world = &world;
            model.scenario_id = "free_fall";
            model.scenario_title = "Free fall";
            for (const auto& description : physics::available_scenarios())
                model.catalogue.push_back({ std::string(description.id), std::string(description.title), std::string(description.summary), std::string(description.collection), std::string(description.level), std::string(description.hook), description.collection_order, description.suggested_order, description.concepts, description.prerequisites, description.tags, description.lab });
            panels = ui::create_default_panels();
        }
        ~Fixture()
        {
            device.reset();
            SDL_DestroySurface(surface);
        }
        void build(float scale = 1.0f)
        {
            current_scale = scale;
            ui::UiFrameContext frame;
            frame.model = &model;
            frame.device = device.get();
            frame.theme = &theme;
            frame.scale = scale;
            frame.viewport = device->drawable_size();
            frame.view = &view;
            ui::LayoutInput input;
            input.viewport = frame.viewport;
            input.scale = scale;
            input.inspector_open = true;
            input.measure_open = view.surface_open("measure.open", false);
            layout = ui::compute_layout(input);
            frame.layout = &layout;
            for (auto& panel : panels)
            {
                auto visible = panel->is_visible();
                if (panels.size() > 1)
                {
                    if (panel->id() == "library")
                        visible = view.sheet_open("library");
                    else if (panel->id() == "main_menu")
                        visible = view.transient_open("main_menu");
                    else if (panel->id() == "preferences")
                        visible = view.sheet_open("preferences");
                    else if (panel->id() == "save_details")
                        visible = view.sheet_open("save_details");
                    else if (panel->id() == "shortcuts")
                        visible = view.sheet_open("shortcuts");
                    else if (panel->id() == "about")
                        visible = view.sheet_open("about");
                    else if (panel->id() == "visualization")
                        visible = view.transient_open("show");
                    else if (panel->id() == "confirmation")
                        visible = model.confirmation.has_value();
                    else if (panel->id() == "draw_bar")
                        visible = model.draft.active;
                    else if (panel->id() == "guide")
                        visible = model.scenario_content && !model.scenario_content->guide.empty();
                    else if (panel->id() == "measure" || panel->id() == "education")
                        visible = view.surface_open("measure.open", false);
                    else if (panel->id() == "banner")
                        visible = model.banner.has_value();
                }
                if (visible)
                    frame.panels.push_back(panel.get());
            }
            backend.build(frame, list);
        }
        std::vector<ui::UiCommand> key(ui::UiKey key, bool shift = false, bool repeat = false)
        {
            ui::UiEvent event;
            event.kind = ui::UiEventKind::key_down;
            event.key = key;
            event.modifiers.shift = shift;
            event.repeat = repeat;
            std::vector<ui::UiCommand> commands;
            backend.handle_event(event, commands);
            return commands;
        }
        // Tabs forward until the control with the given key has keyboard focus.
        bool focus_key(std::string_view control)
        {
            const auto target = ui::element_id("legacy", control);
            for (std::size_t press = 0; press <= backend.control_count(); ++press)
            {
                key(ui::UiKey::tab);
                if (backend.focused_element() == target)
                    return true;
            }
            return false;
        }
        void save(const char* name)
        {
            // RmlUi caps each animation delta at 100 ms, so settle with actual short frames.
            for (int frame = 0; frame < 10; ++frame)
            {
                SDL_Delay(30);
                build(current_scale);
            }
            device->begin_frame(theme.background);
            device->submit(list);
            device->end_frame();
            RIGIDBODIES_EXPECT(SDL_SaveBMP(surface, name), "software document image saved");
        }
    };
    RIGIDBODIES_TEST("a row id reclaimed by another panel never adopts a hidden panel's element")
    {
        // Show and the main menu both offer Frame selection. The panel that is hidden keeps its
        // old rows, so the id must move to the visible panel without reparenting a foreign row.
        Fixture fixture;
        const auto visible_selection = [&]
        {
            const auto id = fixture.backend.element_for_key("legacy", "camera.frame.selection");
            const auto bounds = id ? fixture.backend.element_bounds(*id) : std::nullopt;
            return bounds && bounds->width() > 0.0 && bounds->height() > 0.0;
        };
        for (int round = 0; round < 3; ++round)
        {
            fixture.view.open_transient("show");
            for (int frame = 0; frame < 3; ++frame)
                fixture.build();
            RIGIDBODIES_EXPECT(visible_selection(), "Show presents its Frame selection row");
            fixture.view.close_transient("show");
            fixture.view.open_transient("main_menu");
            for (int frame = 0; frame < 3; ++frame)
                fixture.build();
            RIGIDBODIES_EXPECT(visible_selection(), "the menu reclaims Frame selection without adopting the hidden row");
            fixture.view.close_transient("main_menu");
        }
    }
    RIGIDBODIES_TEST("content file actions and lesson metadata remain accessible in a compact document")
    {
        Fixture fixture(800, 600);
        fixture.panels.clear();
        fixture.panels.push_back(std::make_unique<ui::ScenarioPanel>());
        fixture.model.can_export_shape = true;
        fixture.model.scenario_concepts = { "Editable geometry", "Persistence <and> compatibility" };
        fixture.model.scenario_prerequisites = { "free_fall" };
        fixture.model.scenario_suggested_order = 20;
        fixture.model.notifications.push_back({ 1, ui::Severity::success, "Arrangement opened.", {}, "content", false });
        fixture.build(1.5f);
        const auto text = fixture.backend.document_text();
        RIGIDBODIES_EXPECT(text.find("Editable geometry") != std::string::npos && text.find("Suggested order") != std::string::npos && text.find("Try first") != std::string::npos, "metadata is presented in the scrolling document");
        bool save = false, open = false, import = false, export_shape = false;
        for (std::size_t index = 0; index < fixture.backend.control_count(); ++index)
        {
            fixture.key(ui::UiKey::tab);
            const auto commands = fixture.key(ui::UiKey::enter);
            if (commands.empty())
                continue;
            save |= commands.front().kind == ui::UiCommandKind::save_arrangement;
            open |= commands.front().kind == ui::UiCommandKind::open_arrangement;
            import |= commands.front().kind == ui::UiCommandKind::import_shape;
            export_shape |= commands.front().kind == ui::UiCommandKind::export_shape;
        }
        RIGIDBODIES_EXPECT(save && open && import && export_shape, "all four file operations send the exact command through keyboard activation");
        fixture.save("stage9-content-compact.bmp");
    }
    RIGIDBODIES_TEST("real document generates textured proportional glyph meshes and rounded surfaces")
    {
        Fixture fixture;
        // The Measure drawer's graph and its controls exceed its height, so its scrolling region must clip.
        fixture.view.set_surface_open("measure.open", true);
        fixture.view.set_active_tab("measure.header.tabs", "graph");
        fixture.build();
        std::size_t textured = 0, clipped = 0;
        for (const auto& command : fixture.list.commands())
        {
            RIGIDBODIES_EXPECT(command.kind == render::DrawCommandKind::indexed_mesh, "document never falls back to bitmap debug text");
            RIGIDBODIES_EXPECT(command.mesh && !command.mesh->vertices.empty(), "document geometry has actual vertices");
            for (const auto index : command.mesh->indices)
                RIGIDBODIES_EXPECT(index >= 0 && static_cast<std::size_t>(index) < command.mesh->vertices.size(), "every triangle index is valid");
            if (command.mesh->texture)
                ++textured;
            if (command.mesh->clip)
                ++clipped;
        }
        RIGIDBODIES_EXPECT(textured > 10 && clipped > 10, "glyph atlas and scroll clipping are used");
        fixture.save("stage7-document-dark.bmp");
        fixture.theme = render::theme_by_name("workbench_light");
        fixture.build();
        fixture.save("stage7-document-light.bmp");
        // These points straddle the thin fan triangles that SDL's quad optimization previously
        // dropped, leaving a long wedge through the top of every rounded button.
        Uint8 red, green, blue, alpha, reference_red, reference_green, reference_blue;
        RIGIDBODIES_EXPECT(SDL_ReadSurfacePixel(fixture.surface, 200, 92, &reference_red, &reference_green, &reference_blue, &alpha), "button interior sample is available");
        for (int x = 80; x < 250; ++x)
            for (int y = 85; y <= 89; ++y)
            {
                RIGIDBODIES_EXPECT(SDL_ReadSurfacePixel(fixture.surface, x, y, &red, &green, &blue, &alpha), "rounded fan strip sample is available");
                RIGIDBODIES_EXPECT(red == reference_red && green == reference_green && blue == reference_blue, "rounded button interior has no missing triangle wedges");
            }
    }
    RIGIDBODIES_TEST("material coefficients are listed as figures under the material's name")
    {
        Fixture fixture;
        fixture.world.clear();
        physics::BodyDefinition body;
        body.name = "test_material";
        physics::Collider collider;
        collider.shape = physics::make_circle(0.5);
        collider.material = physics::materials::by_name("expanded_polystyrene");
        collider.material.restitution = 0.30;
        body.colliders.push_back(collider);
        fixture.model.selection = fixture.world.create_body(body);
        fixture.panels.clear();
        fixture.panels.push_back(std::make_unique<ui::InspectorPanel>());
        fixture.build();
        const auto text = fixture.backend.document_text();
        RIGIDBODIES_EXPECT(text.find("Material properties (Foam)") != std::string::npos, "the coefficients are headed by the material they come from");
        RIGIDBODIES_EXPECT(text.find("Bounciness 0.30") != std::string::npos && text.find("Grip at rest") != std::string::npos && text.find("Drag coefficient") != std::string::npos, "each coefficient shows its two-decimal value");
        RIGIDBODIES_EXPECT(text.find("30\xC2\xA0%") == std::string::npos, "no coefficient is drawn as a bar on a hidden percentage scale");
    }
    RIGIDBODIES_TEST("document Guide preserves and displays the complete scenario summary")
    {
        Fixture fixture;
        fixture.panels.clear();
        fixture.panels.push_back(std::make_unique<ui::GuidePanel>());
        fixture.model.scenario_content = ui::ExperimentContent {};
        fixture.model.scenario_content->title = "Guide fixture";
        fixture.model.scenario_content->summary = "This deliberately long experiment summary must survive the backend-neutral row and remain fully available to the scrolling document without a fixed character cut-off near its former boundary.";
        fixture.model.scenario_content->guide.focus = "Observe the motion.";
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.document_text().find(fixture.model.scenario_content->summary) != std::string::npos, "the document contains the complete summary text");
    }
    RIGIDBODIES_TEST("all panels remain present when compact layout reflows and large text scrolls")
    {
        Fixture fixture(800, 600);
        fixture.build(2.0f);
        ui::LayoutInput input;
        input.viewport = { 800, 600 };
        input.scale = 2.0f;
        const auto scene = ui::compute_layout(input).stage;
        RIGIDBODIES_EXPECT(scene.width() > 790 && scene.height() > 300, "compact layout retains the upper scene");
        for (const auto* id : { "command_bar", "status_line", "inspector" })
            RIGIDBODIES_EXPECT(fixture.backend.element_bounds("panel-" + std::string(id)).has_value(), "every visible shell panel remains in the scrolling document");
        RIGIDBODIES_EXPECT(fixture.backend.control_count() > 25, "controls below viewport remain keyboard reachable");
        fixture.save("stage7-document-compact.bmp");
    }
    RIGIDBODIES_TEST("the Measure badge's tooltip and accessible name say what it counts")
    {
        Fixture fixture;
        ui::ImpactItem impact;
        impact.first_name = "Ball";
        impact.second_name = "Floor";
        impact.second_type = physics::BodyType::static_body;
        fixture.model.impacts = { impact, impact, impact };
        fixture.build(1.5f);
        const auto measure = fixture.backend.element_for_key("legacy", "view.measure");
        RIGIDBODIES_EXPECT(measure.has_value(), "the Measure toolbar button is laid out");
        if (!measure)
            return;
        const auto tooltip = fixture.backend.element_attribute(*measure, "data-tooltip");
        const auto name = fixture.backend.element_attribute(*measure, "aria-label");
        RIGIDBODIES_EXPECT(tooltip && tooltip->find("3 impacts recorded") != std::string::npos, "hovering the badge explains the number");
        RIGIDBODIES_EXPECT(name && name->find("3 impacts recorded") != std::string::npos, "assistive technology hears what the number counts");
    }

    RIGIDBODIES_TEST("Runs Compare lays out in the default drawer at the reference viewport")
    {
        Fixture fixture(1600, 900);
        std::vector<ui::RunRecord> runs(2);
        std::vector<ui::PinnedValue> pinned {
            { "speed:max", "speed", fixture.world.body_ids().front(), ui::RunAggregator::maximum, 0.0 },
            { "mechanical:end", "mechanical", {}, ui::RunAggregator::at_end, 0.0 }
        };
        runs[0].number = 1;
        runs[0].duration_s = 1.0;
        runs[0].starred = true;
        runs[0].pinned_results = { 2.0, 0.0 };
        runs[0].changes_from_original = {
            { "body:ball:mass", "object.properties.mass", {}, "Mass", "1 kg", "2 kg", ui::EditCategory::parameter }
        };
        runs[1].number = 2;
        runs[1].duration_s = 1.5;
        runs[1].changed_during_run = true;
        runs[1].pinned_results = { 3.0, 4.0 };
        runs[1].changes_from_original = {
            { "body:ball:mass", "object.properties.mass", {}, "Mass", "1 kg", "3 kg", ui::EditCategory::parameter },
            { "world:gravity", "world.gravity.strength", {}, "Gravity", "9.81 m/s²", "1.62 m/s²", ui::EditCategory::parameter }
        };
        fixture.model.runs = runs;
        fixture.model.pinned_values = pinned;
        fixture.model.starred_run_count = 1;
        fixture.view.set_experiment_context("free_fall");
        fixture.view.set_surface_open("measure.open", true);
        fixture.view.set_active_tab("measure.header.tabs", "runs");
        fixture.build(1.5f);

        RIGIDBODIES_EXPECT(fixture.layout.logical_width > 1066.0 && fixture.layout.logical_width < 1067.0 && fixture.layout.logical_height == 600.0,
            "the reference 1600 by 900 drawable at 150 percent is the prescribed 1067 by 600 logical viewport");
        const auto* drawer = fixture.layout.find(ui::RegionId::measure_drawer);
        RIGIDBODIES_EXPECT(drawer && std::abs(drawer->bounds.height() - 0.4 * (600.0 - 44.0) * 1.5) < 0.01,
            "Compare uses the default Measure drawer, 40 percent of the height below the command bar");
        const auto compare_a = fixture.backend.element_for_key("legacy", "measure.runs.compare_a");
        const auto compare_b = fixture.backend.element_for_key("legacy", "measure.runs.compare_b");
        const auto show_graph = fixture.backend.element_for_key("legacy", "measure.runs.show_graph");
        RIGIDBODIES_EXPECT(compare_a && compare_b && show_graph, "both run selectors and Show in graph are laid out as document controls");
        for (const auto& id : { *compare_a, *compare_b, *show_graph })
        {
            const auto bounds = fixture.backend.element_bounds(id);
            RIGIDBODIES_EXPECT(bounds && bounds->minimum.x >= drawer->bounds.minimum.x && bounds->maximum.x <= drawer->bounds.maximum.x,
                "every Compare control stays inside the drawer width");
        }
        const auto text = fixture.backend.document_text();
        RIGIDBODIES_EXPECT(text.find("A · B · Δ · Δ %") != std::string::npos && text.find("2 things changed between Run 1 and Run 2") != std::string::npos,
            "the comparison columns and exact amber multi-change note remain present in the compact drawer");

        fixture.view.set_value("measure.runs.confirm_clear", "true");
        fixture.build(1.5f);
        RIGIDBODIES_EXPECT(fixture.backend.document_text().find("Clear 1 run? Starred runs are kept.") != std::string::npos,
            "Clear runs names the exact number removed and states that starred runs are kept");
    }
    RIGIDBODIES_TEST("Shortcuts sheet exposes the categorized keyboard reference")
    {
        Fixture fixture;
        fixture.model.keyboard_reference = { { "Space", "Play / pause" }, { "Ctrl+A", "Select all free objects" } };
        fixture.panels.clear();
        fixture.panels.push_back(std::make_unique<ui::ShortcutsPanel>());
        fixture.build(1.5f);
        RIGIDBODIES_EXPECT(fixture.backend.document_text().find("Keyboard shortcuts") != std::string::npos && fixture.backend.document_text().find("Play / pause") != std::string::npos, "the sheet exposes its title and reference rows");
    }
    RIGIDBODIES_TEST("keyboard focus activates exact commands and survives live readout updates")
    {
        Fixture fixture;
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.focus_key("transport.play"), "Tab reaches the Play control");
        const auto focused = fixture.backend.focused_element();
        RIGIDBODIES_EXPECT(!focused.empty(), "Tab gives a control visible keyboard focus");
        fixture.key(ui::UiKey::tab, false, true);
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == focused, "a repeated Tab press does not move focus again");
        const auto space = fixture.key(ui::UiKey::space);
        RIGIDBODIES_EXPECT(space.size() == 1 && space.front().kind == ui::UiCommandKind::toggle_pause, "Space activates the focused control");
        RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::space, false, true).empty(), "holding Space never repeatedly activates the focused control");
        RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::enter, false, true).empty(), "a repeated Enter press does not activate the focused control");
        auto commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().kind == ui::UiCommandKind::toggle_pause, "Enter activates the focused play or pause control");
        fixture.model.elapsed_time_s = 17.25;
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == focused, "live text refresh preserves focus identity");
        fixture.key(ui::UiKey::tab, true);
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() != focused, "reverse traversal reaches the last control");
    }
    RIGIDBODIES_TEST("a pointer click never leaves Space bound to a control")
    {
        Fixture fixture;
        fixture.build(1.5f);
        const auto control_count = fixture.backend.control_count();
        RIGIDBODIES_EXPECT(control_count > 30, "the routing check covers every control in every current panel");
        for (std::size_t control_index = 0; control_index < control_count; ++control_index)
        {
            for (std::size_t tab_index = 0; tab_index <= control_index; ++tab_index)
                fixture.key(ui::UiKey::tab);
            const auto focused = fixture.backend.focused_element();
            const auto rect = fixture.backend.element_bounds(focused);
            RIGIDBODIES_EXPECT(rect.has_value(), "each keyboard-reachable control reports bounds for pointer activation");

            ui::UiEvent event;
            event.kind = ui::UiEventKind::pointer_down;
            event.pointer_px = (rect->minimum + rect->maximum) * 0.5;
            std::vector<ui::UiCommand> commands;
            RIGIDBODIES_EXPECT(fixture.backend.handle_event(event, commands), "control press is consumed by the interface");
            event.kind = ui::UiEventKind::pointer_up;
            RIGIDBODIES_EXPECT(fixture.backend.handle_event(event, commands), "control release is consumed by the interface");
            RIGIDBODIES_EXPECT(commands.size() <= 1, "pointer activation emits no duplicate interface command");
            event.kind = ui::UiEventKind::key_down;
            event.key = ui::UiKey::space;
            commands.clear();
            const auto consumed_space = fixture.backend.handle_event(event, commands);
            RIGIDBODIES_EXPECT(commands.empty(), "Space never activates a control after pointer interaction");
            if (fixture.backend.focused_element().empty())
                RIGIDBODIES_EXPECT(!consumed_space, "Space returns to the scene after pointer command activation");
        }
    }
    RIGIDBODIES_TEST("document pointer activation and scene routing agree with actual layout")
    {
        Fixture fixture;
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.focus_key("transport.play"), "Tab reaches the Play control");
        const auto rect = fixture.backend.element_bounds(fixture.backend.focused_element());
        RIGIDBODIES_EXPECT(rect.has_value(), "focused control reports laid-out bounds");
        ui::UiEvent event;
        event.kind = ui::UiEventKind::pointer_down;
        event.pointer_px = (rect->minimum + rect->maximum) * 0.5;
        std::vector<ui::UiCommand> commands;
        RIGIDBODIES_EXPECT(fixture.backend.handle_event(event, commands), "press on button stays in interface");
        event.kind = ui::UiEventKind::pointer_up;
        RIGIDBODIES_EXPECT(fixture.backend.handle_event(event, commands), "release on button stays in interface");
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().kind == ui::UiCommandKind::toggle_pause, "native DOM click sends command once");
        RIGIDBODIES_EXPECT(fixture.backend.focused_element().empty(), "pointer activation returns focus to the scene");
        fixture.key(ui::UiKey::tab);
        RIGIDBODIES_EXPECT(!fixture.backend.focused_element().empty(), "Tab restores keyboard-visible focus before a scene press");
        event.kind = ui::UiEventKind::pointer_down;
        event.pointer_px = (fixture.layout.stage.minimum + fixture.layout.stage.maximum) * 0.5;
        RIGIDBODIES_EXPECT(!fixture.backend.handle_event(event, commands), "uncovered scene accepts manipulation");
        RIGIDBODIES_EXPECT(fixture.backend.focused_element().empty(), "a primary press in the scene returns focus to the scene");
    }
    RIGIDBODIES_TEST("multiple document contexts and missing asset fallback have independent lifetimes")
    {
        ui::DocumentBackend missing;
        RIGIDBODIES_EXPECT(!missing.initialize("nonexistent-stage7-assets"), "missing assets fail cleanly for overlay fallback");
        Fixture first;
        first.build();
        {
            Fixture second;
            second.build();
        }
        first.build();
        RIGIDBODIES_EXPECT(!first.list.is_empty(), "closing another context preserves shared renderer and glyph atlas");
    }
    RIGIDBODIES_TEST("interface stylesheets and documents are plain UTF-8 without a byte-order mark")
    {
        // RmlUi does not skip a byte-order mark, so one silently discards the first rule block.
        const std::filesystem::path source { RIGIDBODIES_SOURCE_ASSETS };
        for (const auto* path : { "ui/playground.rml", "ui/playground.rcss", "ui/geometry.rcss", "ui/components.rcss", "ui/themes.rcss" })
        {
            std::ifstream file(source / path, std::ios::binary);
            char head[3] {};
            file.read(head, 3);
            RIGIDBODIES_EXPECT(!(head[0] == '\xEF' && head[1] == '\xBB' && head[2] == '\xBF'), std::string(path) + " has no byte-order mark");
        }
    }
    RIGIDBODIES_TEST("incomplete document assets and missing dock nodes fail cleanly")
    {
        const auto directory = std::filesystem::current_path() / "stage7-incomplete-document-assets";
        std::filesystem::remove_all(directory);
        for (const auto* folder : { "fonts", "ui", "licenses" })
            std::filesystem::create_directories(directory / folder);
        const std::filesystem::path source { RIGIDBODIES_SOURCE_ASSETS };
        const auto required = { "fonts/Inter-Regular.ttf", "fonts/Inter-Medium.ttf", "fonts/Inter-SemiBold.ttf", "fonts/Phosphor-Regular-subset.ttf", "fonts/OFL-Inter.txt", "licenses/Phosphor-MIT.txt", "ui/playground.rml", "ui/playground.rcss", "ui/geometry.rcss", "ui/components.rcss", "ui/themes.rcss" };
        const auto copy = [&](const char* path)
        {
            std::filesystem::copy_file(source / path, directory / path, std::filesystem::copy_options::overwrite_existing);
        };
        for (const auto* path : required)
            copy(path);
        {
            ui::DocumentBackend complete;
            RIGIDBODIES_EXPECT(complete.initialize(directory), "the complete shipped asset set starts the document backend");
        }
        for (const auto* path : required)
        {
            std::filesystem::remove(directory / path);
            ui::DocumentBackend backend;
            RIGIDBODIES_EXPECT(!backend.initialize(directory), std::string("a missing ") + path + " is reported instead of producing an unstyled document");
            copy(path);
        }
        {
            std::ofstream malformed(directory / "ui/playground.rml");
            malformed << "<rml><head><title>Missing docks</title></head><body/></rml>";
        }
        ui::DocumentBackend backend;
        RIGIDBODIES_EXPECT(!backend.initialize(directory), "missing dock nodes are rejected before layout can dereference them");
        std::filesystem::remove_all(directory);
    }
    RIGIDBODIES_TEST("compact keyboard traversal scrolls hidden controls into view and focus loss cancels clicks")
    {
        Fixture fixture(800, 600);
        fixture.build(2.0f);
        for (std::size_t i = 0; i < fixture.backend.control_count(); ++i)
            fixture.key(ui::UiKey::tab);
        fixture.build(2.0f);
        auto rect = fixture.backend.element_bounds(fixture.backend.focused_element());
        // Scroll offsets clamp to whole pixels while row heights are fractional, so a focused
        // control may sit within one device pixel of the viewport edge after scrolling.
        RIGIDBODIES_EXPECT(rect && rect->minimum.y >= -1.0 && rect->maximum.y <= 601, "last control scrolls fully into the compact viewport");
        ui::UiEvent event;
        event.kind = ui::UiEventKind::pointer_down;
        event.pointer_px = (rect->minimum + rect->maximum) * 0.5;
        std::vector<ui::UiCommand> commands;
        fixture.backend.handle_event(event, commands);
        event.kind = ui::UiEventKind::focus_lost;
        fixture.backend.handle_event(event, commands);
        event.kind = ui::UiEventKind::pointer_up;
        fixture.backend.handle_event(event, commands);
        RIGIDBODIES_EXPECT(commands.empty(), "losing application focus cancels the pressed control instead of activating on return");
    }
    double luminance(render::Color color)
    {
        const auto channel = [](double value)
        {
            return value <= 0.04045 ? value / 12.92 : std::pow((value + 0.055) / 1.055, 2.4);
        };
        return 0.2126 * channel(color.red) + 0.7152 * channel(color.green) + 0.0722 * channel(color.blue);
    }
    class ContrastComponentsPanel final : public ui::Panel
    {
    public:
        std::string_view id() const override
        {
            return "contrast_components";
        }
        std::string_view title() const override
        {
            return "Components";
        }
        ui::RegionId region() const override
        {
            return ui::RegionId::inspector;
        }
        void build(const ui::UiModel&, ui::PanelBuilder& builder) override
        {
            ui::UiCommand command;
            command.kind = ui::UiCommandKind::set_selected_mass;
            builder.number_row(*ui::find_control_spec("object.properties.mass"), 2.0, command);
            command.kind = ui::UiCommandKind::set_integrator;
            builder.select_row(*ui::find_control_spec("world.advanced.integration_method"), "velocity_verlet", command);
            command.kind = ui::UiCommandKind::set_theme;
            builder.segmented_row(*ui::find_control_spec("prefs.appearance.theme"), "workbench_dark", command);
            command.kind = ui::UiCommandKind::set_gravity_enabled;
            builder.switch_row(*ui::find_control_spec("world.gravity.enabled"), true, command);
            builder.notice("test.notice.contrast", ui::Severity::info, "Informational notice");
        }
    };
    RIGIDBODIES_TEST("computed document colors retain accessible contrast in dark light hover and selected states")
    {
        Fixture fixture;
        fixture.view.open_sheet("library");
        for (const auto* theme : { "workbench_dark", "workbench_light", "workbench_projector" })
        {
            fixture.theme = render::theme_by_name(theme);
            fixture.model.theme_id = theme;
            fixture.build();
            SDL_Delay(260);
            fixture.build();
            fixture.key(ui::UiKey::tab);
            // Inactive controls are exempt from text contrast (WCAG 1.4.3); measure the first
            // control a learner can use.
            for (int skipped = 0; skipped < 8 && fixture.backend.element_has_class(fixture.backend.focused_element(), "is-disabled"); ++skipped)
                fixture.key(ui::UiKey::tab);
            const auto id = fixture.backend.focused_element();
            const auto focus_ring = fixture.backend.element_color(id, "border-top-color");
            const auto focus_surface = fixture.backend.effective_background(id);
            RIGIDBODIES_EXPECT(focus_ring && focus_surface, "keyboard focus exposes a computed focus ring");
            const auto ring_luminance = luminance(*focus_ring), surface_luminance = luminance(*focus_surface);
            RIGIDBODIES_EXPECT((std::max(ring_luminance, surface_luminance) + 0.05) / (std::min(ring_luminance, surface_luminance) + 0.05) >= 3.0, "keyboard focus ring exceeds 3:1 contrast in both themes");
            const auto check = [&](const std::string& element, const std::string& background)
            {
                const auto foreground = fixture.backend.element_color(element, "color");
                const auto surface = fixture.backend.effective_background(background);
                auto message = std::string { "contrast uses actual computed document properties: " };
                message.append(element).append(" / ").append(background);
                RIGIDBODIES_EXPECT(foreground && surface, message);
                const auto a = luminance(*foreground), b = luminance(*surface);
                RIGIDBODIES_EXPECT((std::max(a, b) + 0.05) / (std::min(a, b) + 0.05) >= 4.5, std::string("document text exceeds 4.5:1 contrast in ") + theme + ": " + element + " on " + background + " (" + std::to_string(foreground->red) + "," + std::to_string(foreground->green) + "," + std::to_string(foreground->blue) + " on " + std::to_string(surface->red) + "," + std::to_string(surface->green) + "," + std::to_string(surface->blue) + ")");
            };
            check(id, id);
            const auto scenario = fixture.backend.element_for_key("legacy", "library.cards.card", "free_fall");
            const auto play = fixture.backend.element_for_key("legacy", "transport.play");
            const auto elapsed = fixture.backend.element_for_key("legacy", "bar.time.readout");
            RIGIDBODIES_EXPECT(scenario && play && elapsed, "keyed controls and read-outs are available for contrast checks");
            check(*scenario, *scenario);
            // The open experiment's card is selected; its description and metadata sit on the tint.
            check(*scenario + "--secondary", *scenario);
            check(*scenario + "--badges", *scenario);
            check(*play, *play);
            check(*elapsed, "panel-status_line");
            const auto rect = fixture.backend.element_bounds(id);
            ui::UiEvent event;
            event.kind = ui::UiEventKind::pointer_move;
            event.pointer_px = (rect->minimum + rect->maximum) * 0.5;
            std::vector<ui::UiCommand> commands;
            fixture.backend.handle_event(event, commands);
            SDL_Delay(180);
            fixture.build();
            check(id, id);
        }
    }
    RIGIDBODIES_TEST("new components retain text contrast in dark and light at 75 and 200 percent")
    {
        Fixture fixture;
        fixture.panels.clear();
        fixture.panels.push_back(std::make_unique<ContrastComponentsPanel>());
        for (const auto* theme : { "workbench_dark", "workbench_light", "workbench_projector" })
            for (const auto scale : { 0.75f, 2.0f })
            {
                fixture.theme = render::theme_by_name(theme);
                fixture.model.theme_id = theme;
                fixture.build(scale);
                // Hover backgrounds ease for a moment after a theme switch; read settled colours.
                SDL_Delay(200);
                fixture.build(scale);
                const std::string ids[] {
                    ui::element_id("legacy", "object.properties.mass") + "--field",
                    ui::element_id("legacy", "world.advanced.integration_method") + "--field",
                    ui::element_id("legacy", "prefs.appearance.theme") + "--option_workbench_dark",
                    ui::element_id("legacy", "world.gravity.enabled"),
                    ui::element_id("legacy", "test.notice.contrast")
                };
                for (const auto& id : ids)
                {
                    const auto foreground = fixture.backend.element_color(id, "color");
                    const auto background = fixture.backend.effective_background(id);
                    RIGIDBODIES_EXPECT(foreground && background, "component exposes computed colors at each scale");
                    const auto a = luminance(*foreground), b = luminance(*background);
                    const auto ratio = (std::max(a, b) + 0.05) / (std::min(a, b) + 0.05);
                    RIGIDBODIES_EXPECT(ratio >= 4.5, "component text exceeds 4.5:1: " + id + " in " + theme + " at " + std::to_string(scale) + "x has " + std::to_string(ratio) + ":1");
                }
                // The chosen segment stands out from its track by its fill or its outline at
                // 3:1, the contrast WCAG asks of a control's state (1.4.11).
                const auto segments = ui::element_id("legacy", "prefs.appearance.theme") + "--options";
                const auto chosen = ui::element_id("legacy", "prefs.appearance.theme") + "--option_workbench_dark";
                const auto track = fixture.backend.effective_background(segments);
                const auto fill = fixture.backend.effective_background(chosen);
                const auto outline = fixture.backend.element_color(chosen, "border-top-color");
                RIGIDBODIES_EXPECT(track && fill && outline, "the segmented control exposes its computed colours");
                if (track && fill && outline)
                {
                    const auto contrast = [](render::Color a, render::Color b)
                    {
                        const auto x = luminance(a), y = luminance(b);
                        return (std::max(x, y) + 0.05) / (std::min(x, y) + 0.05);
                    };
                    auto drawn_outline = *outline;
                    drawn_outline.red = outline->alpha * outline->red + (1.0f - outline->alpha) * track->red;
                    drawn_outline.green = outline->alpha * outline->green + (1.0f - outline->alpha) * track->green;
                    drawn_outline.blue = outline->alpha * outline->blue + (1.0f - outline->alpha) * track->blue;
                    const auto ratio = std::max(contrast(*fill, *track), contrast(drawn_outline, *track));
                    RIGIDBODIES_EXPECT(ratio >= 3.0, std::string("the selected segment stands out 3:1 from its track in ") + theme + ": " + std::to_string(ratio) + ":1");
                }
            }
    }
    class MixedValuesPanel final : public ui::Panel
    {
    public:
        std::string_view id() const override
        {
            return "mixed_values";
        }
        std::string_view title() const override
        {
            return "Several objects";
        }
        ui::RegionId region() const override
        {
            return ui::RegionId::inspector;
        }
        void build(const ui::UiModel&, ui::PanelBuilder& builder) override
        {
            ui::UiCommand command;
            command.kind = ui::UiCommandKind::set_selected_mass;
            builder.mixed_number_row(*ui::find_control_spec("object.properties.mass"), command);
            command.kind = ui::UiCommandKind::set_selected_material;
            builder.mixed_select_row(*ui::find_control_spec("object.properties.material"), command);
            builder.select_row(*ui::find_control_spec("object.properties.material"), "grippy_rubber", command, "Grippy rubber");
        }
    };
    RIGIDBODIES_TEST("a mixed field is empty, reads Mixed, and applies only a value that is entered")
    {
        Fixture fixture;
        fixture.panels.clear();
        fixture.panels.push_back(std::make_unique<MixedValuesPanel>());
        fixture.build();
        fixture.build();
        const auto mass = ui::element_id("legacy", "object.properties.mass");
        RIGIDBODIES_EXPECT(fixture.backend.element_value(mass + "--field") == std::string {}, "the mass field shows no invented value");
        RIGIDBODIES_EXPECT(fixture.backend.element_has_class(mass, "is-mixed") && !fixture.backend.element_visible(mass + "--slider"), "the row reads Mixed and offers no slider position");
        const auto material = ui::element_id("legacy", "object.properties.material");
        RIGIDBODIES_EXPECT(fixture.backend.element_value(material + "--field") == std::string { "--mixed" }, "the material select shows Mixed rather than its first option");
        RIGIDBODIES_EXPECT(fixture.backend.element_value(ui::element_id("legacy", "object.properties.material", "2") + "--field") == std::string { "grippy_rubber" }, "an authored material is shown as itself");
        fixture.backend.focus_field("object.properties.mass");
        RIGIDBODIES_EXPECT(fixture.key(ui::UiKey::enter).empty(), "Enter on the empty field changes nothing");
        ui::UiEvent text;
        text.kind = ui::UiEventKind::text_input;
        text.text = "5";
        std::vector<ui::UiCommand> ignored;
        fixture.backend.handle_event(text, ignored);
        const auto commands = fixture.key(ui::UiKey::enter);
        RIGIDBODIES_EXPECT(commands.size() == 1 && commands.front().kind == ui::UiCommandKind::set_selected_mass && commands.front().value == 5.0, "an entered value applies to the whole selection");
    }
    RIGIDBODIES_TEST("command search opens with its field focused and a hint inside it")
    {
        Fixture fixture;
        fixture.view.open_sheet("command_search");
        fixture.panels.clear();
        fixture.panels.push_back(std::make_unique<ui::CommandSearchPanel>());
        fixture.build();
        fixture.backend.focus_field("search.field.query");
        fixture.build();
        const auto row = ui::element_id("legacy", "search.field.query");
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == row + "--field" && fixture.backend.wants_text_input(), "typed keys reach the search field");
        RIGIDBODIES_EXPECT(fixture.backend.element_visible(row + "--placeholder"), "the empty field explains what to type");
        RIGIDBODIES_EXPECT(fixture.backend.document_text().find("Suggestions") != std::string::npos, "suggested settings are listed before anything is typed");
    }
    RIGIDBODIES_TEST("live rows never flash and discrete changes flash non-live rows once for 0.6 seconds")
    {
        Fixture fixture;
        for (const auto id : fixture.world.body_ids())
            if (const auto* body = fixture.world.find_body(id); body && body->type() == physics::BodyType::dynamic_body)
            {
                fixture.model.selection = id;
                fixture.model.selected_bodies = { id };
                break;
            }
        RIGIDBODIES_EXPECT(fixture.model.selection.is_valid(), "the reference scenario has a dynamic object for the mass flash check");
        fixture.build();
        fixture.key(ui::UiKey::tab);
        const auto focus = fixture.backend.focused_element();
        fixture.model.elapsed_time_s = 1.75;
        ++fixture.model.change_serial;
        fixture.build();
        const auto elapsed_row = ui::element_id("legacy", "bar.time.readout");
        RIGIDBODIES_EXPECT(fixture.backend.element_has_class(elapsed_row, "is-live") && !fixture.backend.element_has_class(elapsed_row, "is-flash"), "Elapsed remains live and never carries the flash class even during a discrete change");
        RIGIDBODIES_EXPECT(fixture.backend.document_text().find("1.75") != std::string::npos, "animated presentation shows the current value");

        auto* body = fixture.world.find_body(fixture.model.selection);
        body->override_mass(body->mass_properties().mass_kg * 2.0);
        ++fixture.model.change_serial;
        fixture.build();
        const auto mass_row = ui::element_id("legacy", "object.properties.mass");
        RIGIDBODIES_EXPECT(fixture.backend.element_has_class(mass_row, "is-flash"), "a discrete mass change flashes the Mass row once");
        SDL_Delay(400);
        fixture.build();
        RIGIDBODIES_EXPECT(fixture.backend.element_has_class(mass_row, "is-flash"), "the Mass row remains flashed well inside the 0.6 second interval");
        SDL_Delay(250);
        fixture.build();
        RIGIDBODIES_EXPECT(!fixture.backend.element_has_class(mass_row, "is-flash"), "the Mass row settles after 0.6 seconds plus one frame");

        body->override_mass(body->mass_properties().mass_kg * 1.5);
        fixture.build();
        RIGIDBODIES_EXPECT(!fixture.backend.element_has_class(mass_row, "is-flash"), "a value refresh without a new change serial never re-arms the flash");
        RIGIDBODIES_EXPECT(fixture.backend.focused_element() == focus, "discrete and live refreshes do not steal keyboard focus");
    }
    RIGIDBODIES_TEST("software mesh pixels preserve translucent geometry and glyph texture transparency")
    {
        Fixture fixture(64, 64);
        auto pixels = std::make_shared<render::TexturePixels>();
        pixels->width = 2;
        pixels->height = 1;
        pixels->rgba = { 0, 0, 0, 0, 128, 0, 0, 128 };
        auto mesh = std::make_shared<render::IndexedMesh>();
        mesh->texture = pixels;
        mesh->vertices = { { { 0, 0 }, { 0, 0 }, {} }, { { 64, 0 }, { 1, 0 }, {} }, { { 64, 64 }, { 1, 1 }, {} }, { { 0, 64 }, { 0, 1 }, {} } };
        mesh->indices = { 0, 1, 2, 0, 2, 3 };
        render::DrawList list;
        list.add_indexed_mesh(mesh);
        fixture.device->begin_frame({ 1, 1, 1, 1 });
        fixture.device->submit(list);
        fixture.device->end_frame();
        Uint8 red, green, blue, alpha;
        RIGIDBODIES_EXPECT(SDL_ReadSurfacePixel(fixture.surface, 8, 16, &red, &green, &blue, &alpha), "transparent pixel can be measured");
        RIGIDBODIES_EXPECT(red > 250 && green > 250 && blue > 250, "transparent atlas background preserves the underlying panel");
        RIGIDBODIES_EXPECT(SDL_ReadSurfacePixel(fixture.surface, 56, 16, &red, &green, &blue, &alpha), "blended pixel can be measured");
        RIGIDBODIES_EXPECT(red > 250 && green >= 125 && green <= 129 && blue >= 125 && blue <= 129, "premultiplied half-red composites without a dark fringe");
    }
}
int main()
{
    return rigidbodies::testing::run_all();
}
