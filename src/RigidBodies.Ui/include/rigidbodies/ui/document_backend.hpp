#pragma once

#include <rigidbodies/ui/ui_backend.hpp>

namespace rigidbodies::ui
{
    // RmlUi owns layout and input, while the application renderer replays immutable geometry.
    // No window or graphics device is needed to initialize, lay out, or test a document.
    class DocumentBackend final : public UiBackend
    {
    public:
        DocumentBackend();
        ~DocumentBackend() override;
        [[nodiscard]] std::string_view name() const override;
        [[nodiscard]] bool initialize(const std::filesystem::path& asset_root) override;
        void shutdown() override;
        bool handle_event(const UiEvent& event, std::vector<UiCommand>& commands) override;
        void release_focus(std::vector<UiCommand>& commands) override;
        void build(const UiFrameContext& context, DrawList& list) override;
        void take_pending_commands(std::vector<UiCommand>& commands) override;
        [[nodiscard]] FocusOwner focus_owner() const override;
        [[nodiscard]] EscapeTarget escape_target() const override;
        [[nodiscard]] bool wants_text_input() const override;
        [[nodiscard]] Rect text_input_area() const override;
        [[nodiscard]] CursorShape cursor() const override;
        [[nodiscard]] bool pointer_captured() const override;
        [[nodiscard]] std::optional<std::uint64_t> hovered_notification() const override;
        [[nodiscard]] std::optional<bool> covers(const Vec2& point) const override;
        [[nodiscard]] bool draws_window_controls() const override;
        [[nodiscard]] std::optional<WindowPart> window_part(const Vec2& point) const override;
        [[nodiscard]] bool bare_title_bar(const Vec2& point) const override;
        [[nodiscard]] double measured_height(RegionId id) const override;
        [[nodiscard]] std::optional<Rect> hover_card_bounds() const override;
        void set_clipboard_hooks(const ClipboardReader& reader, const ClipboardWriter& writer) override;
        void reveal(std::string_view key, std::string_view instance) override;
        void focus_field(std::string_view key) override;
        [[nodiscard]] std::vector<UiCommand> emitted_commands() const override;

        // Read-only inspection supports layout/accessibility regression tests and diagnostics.
        [[nodiscard]] std::string document_text() const;
        [[nodiscard]] std::optional<Rect> element_bounds(std::string_view id) const;
        [[nodiscard]] std::string focused_element() const;
        [[nodiscard]] std::size_t control_count() const;
        // Keyboard-reachable controls in traversal order.
        [[nodiscard]] const std::vector<std::string>& control_ids() const;
        // Whether the element and every ancestor are displayed after the latest layout.
        [[nodiscard]] bool element_visible(std::string_view id) const;
        [[nodiscard]] std::vector<std::string> child_ids(std::string_view id) const;
        [[nodiscard]] std::optional<std::string> element_attribute(std::string_view id, std::string_view name) const;
        // The text an element displays, which may be a shortened form of its row's text.
        [[nodiscard]] std::string element_text(std::string_view id) const;
        [[nodiscard]] bool document_has_class(std::string_view class_name) const;
        // The id of the innermost element with an id under a point, as a pointer press would hit it.
        [[nodiscard]] std::string element_at(const Vec2& point) const;
        [[nodiscard]] std::optional<Color> element_color(std::string_view id, std::string_view property) const;
        // The opaque colour a viewer sees behind an element: its own and every ancestor's computed
        // background composited from the document root down.
        [[nodiscard]] std::optional<Color> effective_background(std::string_view id) const;
        [[nodiscard]] bool element_has_class(std::string_view id, std::string_view class_name) const;
        [[nodiscard]] std::optional<std::string> element_for_key(std::string_view host, std::string_view key, std::string_view instance = {}) const;
        [[nodiscard]] std::vector<UiCommand> commands_for_key(std::string_view key) const;
        [[nodiscard]] std::optional<std::string> element_value(std::string_view id) const;
        [[nodiscard]] std::size_t structure_change_count() const;
        [[nodiscard]] std::size_t text_write_count(std::string_view id) const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
