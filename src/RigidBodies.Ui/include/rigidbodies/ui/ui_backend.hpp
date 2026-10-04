#pragma once

#include <rigidbodies/ui/panel.hpp>
#include <rigidbodies/ui/layout.hpp>
#include <rigidbodies/ui/ui_event.hpp>
#include <rigidbodies/ui/view_state.hpp>

#include <filesystem>
#include <functional>
#include <memory>
#include <vector>

namespace rigidbodies::ui
{
    using ClipboardReader = std::function<std::string()>;
    using ClipboardWriter = std::function<void(std::string_view)>;

    enum class FocusOwner : std::uint8_t
    {
        scene,
        keyboard_control,
        text_field,
        transient
    };

    enum class EscapeTarget : std::uint8_t
    {
        none,
        text_field,
        control,
        transient,
        sheet
    };

    enum class CursorShape : std::uint8_t
    {
        arrow,
        pointer,
        text,
        crosshair,
        move,
        resize_ns,
        resize_ew,
        not_allowed,
        open_hand,
        grab,
        pen
    };

    using render::ViewportSize;

    // What a point in the window is, when the interface draws the window's own title bar: the
    // interface itself, the title bar the window is dragged by, or one of its window controls.
    enum class WindowPart : std::uint8_t
    {
        client,
        title_bar,
        minimize,
        maximize,
        close
    };

    // The window around the interface, as the application reports it each frame.
    struct WindowFrameState
    {
        // The interface draws the window's title bar controls; the platform draws none.
        bool controls { false };
        bool maximized { false };
        // A fixed-size window greys out its maximize button, as a native one does.
        bool resizable { true };
        // The window has keyboard focus; an inactive window dims its controls, as native ones do.
        bool active { true };
        // Where the platform owns the controls' pointer input (Windows treats them as caption
        // buttons), the interface sees no hover there, so the platform reports it.
        WindowPart hot { WindowPart::client };
        WindowPart pressed { WindowPart::client };
    };

    // What a backend is given to draw one frame of the interface.
    struct UiFrameContext
    {
        const UiModel* model { nullptr };
        const render::RenderDevice* device { nullptr };
        const Theme* theme { nullptr };
        float scale { 1.0f };
        ViewportSize viewport {};
        const ViewState* view { nullptr };
        const LayoutResult* layout { nullptr };
        double wall_time_s { 0.0 };
        std::vector<Notification> toasts;

        // Panels to present, already filtered to the visible ones and ordered as they should
        // appear within their dock.
        std::vector<Panel*> panels;

        WindowFrameState window_frame {};
    };

    // Turns panels into drawing commands and routes input to them.
    //
    // The seam exists because the presentation of this interface is expected to change while what
    // it presents is not. Panels describe their content in terms of rows and commands; how those
    // rows are laid out, styled, and animated belongs to a backend. A document-driven backend can
    // therefore replace the built-in one without a single panel being rewritten.
    class UiBackend
    {
    public:
        UiBackend() = default;
        UiBackend(const UiBackend&) = delete;
        UiBackend(UiBackend&&) = delete;
        UiBackend& operator=(const UiBackend&) = delete;
        UiBackend& operator=(UiBackend&&) = delete;
        virtual ~UiBackend() = default;

        [[nodiscard]] virtual std::string_view name() const = 0;

        // Returns false when the backend could not start, in which case the caller falls back to
        // another one rather than failing to open.
        [[nodiscard]] virtual bool initialize(const std::filesystem::path& asset_root) = 0;

        virtual void shutdown() = 0;

        // Returns true when the interface consumed the event, in which case it must not also be
        // treated as an interaction with the scene behind it.
        virtual bool handle_event(const UiEvent& event, std::vector<UiCommand>& commands) = 0;

        // A press the scene takes ends keyboard focus in the interface, as a press anywhere
        // outside a focused control does; an edit in progress commits the way a blur does.
        virtual void release_focus(std::vector<UiCommand>&)
        {
        }

        virtual void build(const UiFrameContext& context, DrawList& list) = 0;

        // Reconciliation can end a disappearing edit or move focus as a surface opens.
        // Deliver those commands without waiting for another user input event.
        virtual void take_pending_commands(std::vector<UiCommand>&)
        {
        }

        [[nodiscard]] virtual FocusOwner focus_owner() const
        {
            return FocusOwner::scene;
        }
        [[nodiscard]] virtual EscapeTarget escape_target() const
        {
            return EscapeTarget::none;
        }
        [[nodiscard]] virtual bool wants_text_input() const
        {
            return false;
        }
        [[nodiscard]] virtual Rect text_input_area() const
        {
            return {};
        }
        [[nodiscard]] virtual CursorShape cursor() const
        {
            return CursorShape::arrow;
        }
        [[nodiscard]] virtual bool pointer_captured() const
        {
            return false;
        }
        [[nodiscard]] virtual std::optional<std::uint64_t> hovered_notification() const
        {
            return std::nullopt;
        }
        // Whether a visible surface of this backend lies under the point. Backends that place
        // surfaces themselves (anchored menus, cards beside a target) answer here; nullopt lets the
        // caller fall back to the layout's region rectangles.
        // The laid-out height in pixels of a region whose height follows its content, or zero.
        [[nodiscard]] virtual double measured_height(RegionId) const
        {
            return 0.0;
        }
        // Where the latest build placed the hover card, so the stage can keep its labels clear of
        // it; nullopt while no card is shown or the backend does not place one itself.
        [[nodiscard]] virtual std::optional<Rect> hover_card_bounds() const
        {
            return std::nullopt;
        }
        [[nodiscard]] virtual std::optional<bool> covers(const Vec2&) const
        {
            return std::nullopt;
        }
        // Whether this backend can draw the window's title bar controls, so the window can do
        // without the platform's own title bar.
        [[nodiscard]] virtual bool draws_window_controls() const
        {
            return false;
        }
        // What the point is part of, at the granularity of the backend's own controls; nullopt
        // lets the caller fall back to the layout's regions.
        [[nodiscard]] virtual std::optional<WindowPart> window_part(const Vec2&) const
        {
            return std::nullopt;
        }
        // Whether the point is bare title bar even while a menu is open; a press there closes the
        // menu, as one on a native title bar does.
        [[nodiscard]] virtual bool bare_title_bar(const Vec2&) const
        {
            return false;
        }
        virtual void set_clipboard_hooks(const ClipboardReader&, const ClipboardWriter&)
        {
        }
        virtual void reveal(std::string_view, std::string_view)
        {
        }
        // Puts the caret in the text field built for a control key, as a palette does on opening.
        virtual void focus_field(std::string_view)
        {
        }
        [[nodiscard]] virtual std::vector<UiCommand> emitted_commands() const
        {
            return {};
        }
    };

    using UiBackendPtr = std::unique_ptr<UiBackend>;

    enum class UiBackendKind
    {
        // Panels drawn through the project renderer. Needs no package beyond the renderer itself.
        overlay,

        // Panels presented as styled documents by RmlUi.
        document
    };

    [[nodiscard]] std::string_view to_string(UiBackendKind kind);
    [[nodiscard]] UiBackendKind backend_kind_from_name(std::string_view name);

    // Creates the requested backend, or nothing when it is not available in this build. The
    // document backend is present only when the project is configured with RmlUi enabled.
    [[nodiscard]] UiBackendPtr create_ui_backend(UiBackendKind kind);

} // namespace rigidbodies::ui
