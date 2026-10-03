#include <rigidbodies/render/scene_renderer.hpp>
#include <rigidbodies/render/sdl_render_device.hpp>
#include <rigidbodies/ui/ui_context.hpp>

#include <SDL3/SDL.h>

#include "test_framework.hpp"

#include <array>
#include <filesystem>
#include <set>

namespace
{
    using namespace rigidbodies;

    struct Gallery
    {
        SDL_Surface* surface {};
        render::RenderDevicePtr device;
        ui::UiContext interface;
        physics::World world;
        render::SceneRenderer scene;
        render::SceneRenderSettings settings;
        render::Camera2D camera;
        ui::UiModel model;
        render::DrawList scene_list, interface_list;
        std::array<physics::BodyId, 4> examples;

        Gallery()
        {
            surface = SDL_CreateSurface(1600, 900, SDL_PIXELFORMAT_RGBA32);
            RIGIDBODIES_EXPECT(surface, "gallery surface created without a desktop window");
            device = render::SdlRenderDevice::adopt_software_renderer(SDL_CreateSoftwareRenderer(surface));
            RIGIDBODIES_EXPECT(device != nullptr, "software presentation device created");
            RIGIDBODIES_EXPECT(device->load_font(std::filesystem::path(RIGIDBODIES_SOURCE_ASSETS) / "fonts/Inter-Medium.ttf"), "bundled proportional scene font loaded");
            RIGIDBODIES_EXPECT(interface.initialize(ui::UiBackendKind::document, RIGIDBODIES_SOURCE_ASSETS), "viewer interface initialized");
            physics::WorldSettings environment;
            environment.gravity_m_s2 = {};
            world.set_settings(environment);
            physics::BodyDefinition floor;
            floor.name = "Support surface";
            floor.type = physics::BodyType::static_body;
            floor.position_m = { 0.0, -0.60 };
            physics::Collider floor_shape;
            floor_shape.shape = physics::make_box(4.3, 0.24);
            floor_shape.material = physics::materials::steel();
            floor.colliders.push_back(floor_shape);
            world.create_body(floor);
            const std::array<physics::Material, 4> materials { physics::materials::steel(), physics::materials::oak_wood(), physics::materials::rubber(), physics::materials::expanded_polystyrene() };
            const std::array<math::Vec2, 4> positions { math::Vec2 { -1.55, 0.90 }, math::Vec2 { -0.45, -0.155 }, math::Vec2 { 0.55, 0.21 }, math::Vec2 { 1.45, -0.10 } };
            const std::array<const char*, 4> names { "Steel", "Oak", "Rubber", "Foam" };
            for (std::size_t i = 0; i < materials.size(); ++i)
            {
                physics::BodyDefinition body;
                body.name = names[i];
                body.position_m = positions[i];
                body.linear_velocity_m_s = i == 0 ? math::Vec2 { 2.0, -1.0 } : i == 2 ? math::Vec2 { 0, -4.0 }
                                                                                      : math::Vec2 {};
                body.orientation_rad = i == 3 ? 0.10 : 0.0;
                physics::Collider shape;
                shape.shape = i % 2 == 0 ? physics::make_circle(0.34) : physics::make_box(0.62, 0.65);
                shape.material = materials[i];
                body.colliders.push_back(shape);
                examples[i] = world.create_body(body);
            }
            settings.layers = render::LayerMask::none();
            for (const auto layer : { render::VisualizationLayer::grid, render::VisualizationLayer::bodies, render::VisualizationLayer::outlines, render::VisualizationLayer::labels, render::VisualizationLayer::selection })
                settings.layers.set(layer, true);
            settings.transitions = false;
            settings.selection = examples[2];
            for (int step = 0; step < 120; ++step)
            {
                world.step(1.0 / 120.0);
                scene.record_visual_sample(world, 1.0 / 120.0, settings);
                if (step >= 8 && scene.effect_statistics().impact_bursts > 0)
                    break;
            }
            for (int step = 0; step < 3; ++step)
            {
                world.step(1.0 / 120.0);
                scene.record_visual_sample(world, 1.0 / 120.0, settings);
            }
            RIGIDBODIES_EXPECT(scene.effect_statistics().impact_bursts > 0 && scene.effect_statistics().motion_bodies > 0, "gallery contains a real collision and sampled motion");
            model.world = &world;
            model.scenario_title = "Material laboratory";
            model.scenario_summary = "A common light reveals steel, oak, rubber, and foam. Impact and motion cues reflect the simulated state.";
            model.selection = examples[2];
            model.selected_bodies = { examples[2] };
            model.paused = true;
            model.layers = settings.layers;
            model.visual_settings = settings;
            model.elapsed_time_s = world.statistics().elapsed_time_s;
            model.render_backend = "Software image rendering";
            model.interface_backend = std::string(interface.backend_name());
            camera.set_viewport({ 1600, 900 });
        }

        ~Gallery()
        {
            interface.shutdown();
            device.reset();
            SDL_DestroySurface(surface);
        }

        void draw(bool light, bool with_interface, const char* filename)
        {
            settings.theme = render::theme_by_name(light ? "workbench_light" : "workbench_dark");
            model.theme_id = light ? "workbench_light" : "workbench_dark";
            model.visual_settings = settings;
            interface.set_theme(settings.theme);
            camera.set_view_height(with_interface ? 4.2 : 3.5);
            camera.set_center({ 0.0, 0.3 });
            if (with_interface)
            {
                const auto region = interface.layout().stage;
                camera.set_focus_rect({ region.minimum.x + 24.0, region.minimum.y + 24.0, region.width() - 48.0, region.height() - 68.0 });
                math::Aabb subject;
                subject.expand({ -2.0, -1.3 });
                subject.expand({ 2.0, 2.2 });
                camera.frame_bounds(subject, 0.08);
                for (int frame = 0; frame < 10; ++frame)
                {
                    interface.build(model, *device, interface_list);
                    SDL_Delay(30);
                }
            }
            const auto checkpoint = world.snapshot();
            scene.render(world, camera, settings, scene_list);
            const auto heading = with_interface ? math::Vec2 { 356, 42 } : math::Vec2 { 82, 46 };
            scene_list.set_layer(30);
            scene_list.add_text(heading, "Material, light, and motion", settings.theme.panel_title, 1.6f);
            scene_list.add_text(heading + math::Vec2 { 0, 35 }, "One physical material record. A readable visual difference.", settings.theme.panel_text, 1.0f);
            scene_list.add_text({ heading.x, with_interface ? 562.0 : 825.0 }, "Steel  /  Oak  /  Rubber  /  Foam", settings.theme.panel_text, 1.0f);
            device->begin_frame(settings.theme.background);
            device->submit(scene_list);
            if (with_interface)
                device->submit(interface_list);
            device->end_frame();
            RIGIDBODIES_EXPECT(SDL_SavePNG(surface, filename), "rendered presentation PNG saved for visual inspection");

            std::set<std::uint32_t> colors;
            for (int y = 110; y < 550; y += 3)
                for (int x = 360; x < 1240; x += 3)
                {
                    Uint8 red, green, blue, alpha;
                    RIGIDBODIES_EXPECT(SDL_ReadSurfacePixel(surface, x, y, &red, &green, &blue, &alpha), "scene pixels readable");
                    RIGIDBODIES_EXPECT(alpha == 255, "opaque presentation composites correctly");
                    colors.insert((static_cast<std::uint32_t>(red) << 16u) | (static_cast<std::uint32_t>(green) << 8u) | blue);
                }
            RIGIDBODIES_EXPECT(colors.size() > 500, "scene contains shaded gradients and antialiased edges, not blank flat paint");
            physics::World expected;
            expected.restore(checkpoint);
            RIGIDBODIES_EXPECT(world.statistics().step_index == expected.statistics().step_index, "drawing cannot advance the world clock");
            for (const auto id : world.body_ids())
            {
                const auto* actual = world.find_body(id);
                const auto* before = expected.find_body(id);
                RIGIDBODIES_EXPECT(actual->position_m() == before->position_m() && actual->linear_velocity_m_s() == before->linear_velocity_m_s(), "presentation preserves positions and velocities");
                RIGIDBODIES_EXPECT(actual->orientation_rad() == before->orientation_rad() && actual->angular_velocity_rad_s() == before->angular_velocity_rad_s() && actual->is_awake() == before->is_awake(), "presentation preserves rotation and sleep state");
                RIGIDBODIES_EXPECT(actual->mass_properties().mass_kg == before->mass_properties().mass_kg && actual->colliders().size() == before->colliders().size(), "deformation changes only rendered geometry");
            }
            RIGIDBODIES_EXPECT_NEAR(world.statistics().total_kinetic_energy_j, expected.statistics().total_kinetic_energy_j, 0.0, "drawing consumes no physical energy");
        }
    };

    RIGIDBODIES_TEST("dark and light material galleries render the real scene and viewer interface without mutating physics")
    {
        Gallery gallery;
        gallery.draw(false, true, "stage8-presentation-dark.png");
        gallery.draw(true, true, "stage8-presentation-light.png");
    }

    RIGIDBODIES_TEST("scene closeup retains material form trails and impact cues without the interface")
    {
        Gallery gallery;
        gallery.draw(false, false, "stage8-presentation-scene.png");
    }
}

int main()
{
    return rigidbodies::testing::run_all();
}
