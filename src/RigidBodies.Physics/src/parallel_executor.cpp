#include <rigidbodies/physics/parallel_executor.hpp>

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace rigidbodies::physics
{
    namespace
    {
        struct ActiveExecutor;
        thread_local const ActiveExecutor* active_executor = nullptr;

        struct ActiveExecutor
        {
            explicit ActiveExecutor(const void* executor) : identity(executor), previous(active_executor)
            {
                active_executor = this;
            }
            ~ActiveExecutor()
            {
                active_executor = previous;
            }
            const void* identity;
            const ActiveExecutor* previous;
        };
    }

    struct ParallelExecutor::State
    {
        explicit State(std::size_t requested)
        {
            if (requested > 32)
                throw std::invalid_argument("Parallel worker count must be between zero and 32");
            count = requested == 0 ? std::max<std::size_t>(1, std::min<std::size_t>(8, std::thread::hardware_concurrency())) : requested;
        }

        ~State()
        {
            {
                const std::lock_guard<std::mutex> lock(mutex);
                stopping = true;
            }
            ready.notify_all();
            for (auto& thread : workers)
                thread.join();
        }

        void start_workers()
        {
            while (workers.size() + 1 < count)
            {
                const auto partition = workers.size();
                workers.emplace_back([this, partition]
                    {
                        std::size_t previous_generation = 0;
                        for (;;)
                        {
                            std::unique_lock<std::mutex> lock(mutex);
                            ready.wait(lock, [&]
                                {
                                    return stopping || generation != previous_generation;
                                });
                            if (stopping)
                                return;
                            previous_generation = generation;
                            if (partition + 1 >= partitions)
                                continue;
                            lock.unlock();
                            invoke(partition);
                            lock.lock();
                            if (--pending == 0)
                                finished.notify_one();
                        }
                    });
            }
        }

        void invoke(std::size_t partition)
        {
            const ActiveExecutor active(this);
            try
            {
                // Quotient/remainder partitioning avoids multiplication overflow.
                const auto width = items / partitions;
                const auto remainder = items % partitions;
                const auto begin = partition * width + std::min(partition, remainder);
                operation(begin, begin + width + (partition < remainder ? 1 : 0), partition);
            }
            catch (...)
            {
                errors[partition] = std::current_exception();
            }
        }

        std::size_t count {};
        std::mutex batches;
        std::mutex mutex;
        std::condition_variable ready;
        std::condition_variable finished;
        std::vector<std::thread> workers;
        std::vector<std::exception_ptr> errors;
        std::function<void(std::size_t, std::size_t, std::size_t)> operation;
        std::size_t generation {};
        std::size_t partitions {};
        std::size_t pending {};
        std::size_t items {};
        bool stopping { false };
    };

    ParallelExecutor::ParallelExecutor(std::size_t worker_count) : state_(std::make_unique<State>(worker_count))
    {
    }
    ParallelExecutor::~ParallelExecutor() = default;

    std::size_t ParallelExecutor::worker_count() const
    {
        return state_->count;
    }

    std::size_t ParallelExecutor::run(std::size_t item_count, std::size_t minimum_batch_size,
        const std::function<void(std::size_t, std::size_t, std::size_t)>& operation)
    {
        if (minimum_batch_size == 0)
            throw std::invalid_argument("Parallel minimum batch size must be positive");
        if (item_count == 0)
            return 0;
        if (!operation)
            throw std::invalid_argument("Parallel operation cannot be empty");
        for (auto active = active_executor; active; active = active->previous)
            if (active->identity == state_.get())
                throw std::logic_error("Parallel executor cannot run reentrant batches");
        const std::lock_guard<std::mutex> batch_lock(state_->batches);
        const auto partitions = std::min(state_->count, std::max<std::size_t>(1, item_count / minimum_batch_size));
        if (partitions == 1)
        {
            const ActiveExecutor active(state_.get());
            operation(0, item_count, 0);
            return 1;
        }
        state_->start_workers();
        {
            const std::lock_guard<std::mutex> lock(state_->mutex);
            state_->operation = operation;
            state_->items = item_count;
            state_->partitions = partitions;
            state_->pending = partitions - 1;
            state_->errors.assign(partitions, {});
            ++state_->generation;
        }
        state_->ready.notify_all();
        state_->invoke(partitions - 1);
        {
            std::unique_lock<std::mutex> lock(state_->mutex);
            state_->finished.wait(lock, [&]
                {
                    return state_->pending == 0;
                });
            state_->operation = {};
        }
        for (const auto& error : state_->errors)
            if (error)
                std::rethrow_exception(error);
        return partitions;
    }
}
