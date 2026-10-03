#pragma once

#include <cstddef>
#include <iterator>
#include <type_traits>

namespace rigidbodies::math
{

    // A non-owning view over a contiguous sequence.
    //
    // This type provides the contiguous-view operations used by the project.
    //
    // Interfaces take one of these so that a caller may pass an array, a vector, or any other
    // contiguous storage without the interface committing to a container, and without copying.
    // A span does not own what it points at, so it must not outlive it.
    template <typename T>
    class Span
    {
    public:
        using element_type = T;
        using value_type = std::remove_cv_t<T>;
        using size_type = std::size_t;
        using pointer = T*;
        using reference = T&;
        using iterator = T*;

        constexpr Span() = default;

        constexpr Span(pointer first, size_type count) : data_(first), size_(count)
        {
        }

        // Accepts any contiguous container or array whose data pointer converts to this element
        // type, which covers arrays, std::array, and std::vector in both const and mutable form.
        // The guard keeps this from competing with the copy constructor.
        template <typename Container,
            typename = std::enable_if_t<
                !std::is_same_v<std::remove_cv_t<std::remove_reference_t<Container>>, Span> &&
                std::is_convertible_v<decltype(std::data(std::declval<Container&>())), pointer>>>
        constexpr Span(Container&& container) : data_(std::data(container)), size_(std::size(container))
        {
        }

        [[nodiscard]] constexpr pointer data() const
        {
            return data_;
        }

        [[nodiscard]] constexpr size_type size() const
        {
            return size_;
        }

        [[nodiscard]] constexpr bool empty() const
        {
            return size_ == 0;
        }

        [[nodiscard]] constexpr reference operator[](size_type index) const
        {
            return data_[index];
        }

        [[nodiscard]] constexpr reference front() const
        {
            return data_[0];
        }

        [[nodiscard]] constexpr reference back() const
        {
            return data_[size_ - 1];
        }

        [[nodiscard]] constexpr iterator begin() const
        {
            return data_;
        }

        [[nodiscard]] constexpr iterator end() const
        {
            return data_ + size_;
        }

        // Returns the remainder of the sequence after the first offset elements. An offset beyond
        // the end yields an empty span rather than an invalid one.
        [[nodiscard]] constexpr Span subspan(size_type offset) const
        {
            if (offset >= size_)
            {
                return {};
            }
            return { data_ + offset, size_ - offset };
        }

        [[nodiscard]] constexpr Span subspan(size_type offset, size_type count) const
        {
            if (offset >= size_)
            {
                return {};
            }
            const auto available = size_ - offset;
            return { data_ + offset, count < available ? count : available };
        }

    private:
        pointer data_ { nullptr };
        size_type size_ { 0 };
    };

} // namespace rigidbodies::math
