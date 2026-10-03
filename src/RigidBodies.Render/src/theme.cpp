#include <rigidbodies/render/color.hpp>

#include <string_view>

namespace rigidbodies::render
{
    namespace
    {

        Theme workbench_dark()
        {
            return Theme {};
        }

        Theme workbench_light()
        {
            Theme theme;
            theme.background = Color::from_bytes(236, 238, 242);
            theme.stage_highlight = Color::from_bytes(245, 246, 248);
            theme.stage_vignette = Color::from_bytes(30, 40, 60, 14);
            theme.grid_minor = Color::from_bytes(20, 30, 50, 9);
            theme.grid_major = Color::from_bytes(20, 30, 50, 21);
            theme.axis = Color::from_bytes(40, 52, 74, 92);

            theme.body_fill = Color::from_bytes(142, 172, 216, 235);
            theme.body_outline = Color::from_bytes(58, 92, 142);
            theme.static_body_fill = Color::from_bytes(219, 223, 229);
            theme.static_body_outline = Color::from_bytes(176, 183, 194);
            theme.kinematic_body_fill = Color::from_bytes(186, 214, 210, 235);
            theme.kinematic_body_outline = Color::from_bytes(22, 122, 112);
            theme.ground_surface = Color::from_bytes(84, 94, 110);
            theme.ground_hatch = Color::from_bytes(20, 30, 50, 30);

            theme.selection = Color::from_bytes(37, 99, 235);
            theme.center_of_mass = Color::from_bytes(255, 255, 255);
            theme.center_of_mass_ink = Color::from_bytes(21, 24, 29);
            theme.shadow = Color::from_bytes(20, 30, 50, 70);

            theme.velocity = Color::from_bytes(16, 134, 88);
            theme.acceleration = Color::from_bytes(8, 120, 172);
            theme.force = Color::from_bytes(206, 70, 40);
            theme.momentum = Color::from_bytes(118, 76, 196);
            theme.contact = Color::from_bytes(140, 100, 0);
            theme.bounds = Color::from_bytes(120, 130, 146, 190);
            theme.trajectory = Color::from_bytes(52, 112, 186, 190);

            theme.label_plate = Color::from_bytes(255, 255, 255, 230);
            theme.label_border = Color::from_bytes(20, 30, 50, 30);
            theme.label_text = Color::from_bytes(21, 24, 29);
            theme.label_muted = Color::from_bytes(74, 82, 96);

            theme.panel_background = Color::from_bytes(255, 255, 255, 240);
            theme.panel_border = Color::from_bytes(214, 218, 225);
            theme.panel_title = Color::from_bytes(21, 24, 29);
            theme.panel_text = Color::from_bytes(74, 82, 96);
            theme.panel_accent = Color::from_bytes(37, 99, 235);
            theme.panel_muted = Color::from_bytes(92, 101, 115);
            theme.light_stage = true;
            return theme;
        }

        Theme workbench_projector()
        {
            Theme theme = workbench_light();
            theme.background = Color::from_bytes(255, 255, 255);
            theme.stage_highlight = Color::from_bytes(255, 255, 255);
            theme.stage_vignette = Color::from_bytes(0, 0, 0, 0);
            theme.grid_minor = Color::from_bytes(0, 0, 0, 12);
            theme.grid_major = Color::from_bytes(0, 0, 0, 30);
            theme.axis = Color::from_bytes(0, 0, 0, 130);
            theme.body_fill = Color::from_bytes(154, 188, 232);
            theme.body_outline = Color::from_bytes(18, 54, 104);
            theme.static_body_fill = Color::from_bytes(228, 231, 236);
            theme.static_body_outline = Color::from_bytes(70, 76, 86);
            theme.kinematic_body_outline = Color::from_bytes(0, 100, 90);
            theme.ground_surface = Color::from_bytes(16, 20, 26);
            theme.ground_hatch = Color::from_bytes(0, 0, 0, 46);
            theme.selection = Color::from_bytes(0, 62, 168);
            theme.center_of_mass_ink = Color::from_bytes(0, 0, 0);
            theme.shadow = Color::from_bytes(0, 0, 0, 60);
            theme.velocity = Color::from_bytes(0, 112, 70);
            theme.acceleration = Color::from_bytes(0, 94, 150);
            theme.force = Color::from_bytes(186, 44, 12);
            theme.momentum = Color::from_bytes(110, 40, 180);
            theme.contact = Color::from_bytes(122, 98, 0);
            theme.bounds = Color::from_bytes(60, 66, 76);
            theme.trajectory = Color::from_bytes(0, 62, 168, 200);
            theme.label_plate = Color::from_bytes(255, 255, 255, 245);
            theme.label_border = Color::from_bytes(0, 0, 0, 70);
            theme.label_text = Color::from_bytes(0, 0, 0);
            theme.label_muted = Color::from_bytes(45, 51, 60);
            theme.panel_background = Color::from_bytes(255, 255, 255, 250);
            theme.panel_border = Color::from_bytes(65, 76, 92);
            theme.panel_title = Color::from_bytes(0, 0, 0);
            theme.panel_text = Color::from_bytes(34, 38, 44);
            theme.panel_accent = Color::from_bytes(0, 62, 168);
            theme.panel_muted = Color::from_bytes(66, 72, 82);
            theme.stroke_weight = 1.3f;
            return theme;
        }

    } // namespace

    Theme theme_by_name(const char* name)
    {
        const std::string_view requested { name == nullptr ? "" : name };
        if (requested == "workbench_light")
        {
            return workbench_light();
        }
        if (requested == "workbench_projector")
            return workbench_projector();
        return workbench_dark();
    }

} // namespace rigidbodies::render
