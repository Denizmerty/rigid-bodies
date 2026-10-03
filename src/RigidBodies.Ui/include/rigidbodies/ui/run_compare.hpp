#pragma once

#include <rigidbodies/ui/run_types.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

namespace rigidbodies::ui
{
    struct RunValueComparison
    {
        std::optional<double> a, b, delta, delta_percent;
    };

    inline RunValueComparison compare_run_value(const RunRecord& a, const RunRecord& b, std::size_t index)
    {
        RunValueComparison result;
        if (index < a.pinned_results.size())
            result.a = a.pinned_results[index];
        if (index < b.pinned_results.size())
            result.b = b.pinned_results[index];
        if (!result.a || !result.b)
            return result;
        result.delta = *result.b - *result.a;
        if (*result.a != 0.0)
            result.delta_percent = *result.delta / std::abs(*result.a);
        return result;
    }

    inline std::size_t setup_difference_count(const RunRecord& a, const RunRecord& b)
    {
        std::size_t count = 0;
        for (const auto& item : a.changes_from_original)
        {
            const auto other = std::find_if(b.changes_from_original.begin(), b.changes_from_original.end(), [&](const auto& value)
                {
                    return value.key == item.key;
                });
            if (other == b.changes_from_original.end() || other->current_text != item.current_text)
                ++count;
        }
        for (const auto& item : b.changes_from_original)
            if (std::none_of(a.changes_from_original.begin(), a.changes_from_original.end(), [&](const auto& value)
                    {
                        return value.key == item.key;
                    }))
                ++count;
        return count;
    }
}
