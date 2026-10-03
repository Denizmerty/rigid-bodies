#pragma once

#include <cstddef>
#include <functional>
#include <memory>

namespace rigidbodies::physics
{
    // Scheduling changes neither simulation data nor the order in which results are consumed.
    // Zero selects up to eight available hardware threads; explicit counts are bounded at 32.
    struct ParallelSettings
    {
        std::size_t worker_count { 0 };
        std::size_t minimum_batch_size { 64 };
    };

    // Persistent workers execute disjoint, contiguous ranges. Results belong to their partition,
    // never to arrival order. The caller participates, and batches below the threshold stay local.
    // Exceptions are rethrown after every partition completes, in partition order. Reentrant calls
    // on the same executor are rejected; independent callers are serialized.
    // The operation must restrict mutable output to its own range/partition or synchronize it.
    class ParallelExecutor
    {
    public:
        explicit ParallelExecutor(std::size_t worker_count = 0);
        ~ParallelExecutor();
        ParallelExecutor(const ParallelExecutor&) = delete;
        ParallelExecutor& operator=(const ParallelExecutor&) = delete;

        [[nodiscard]] std::size_t worker_count() const;
        std::size_t run(std::size_t item_count, std::size_t minimum_batch_size,
            const std::function<void(std::size_t begin, std::size_t end, std::size_t partition)>& operation);

    private:
        struct State;
        std::unique_ptr<State> state_;
    };
}
