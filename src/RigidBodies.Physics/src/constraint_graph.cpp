#include <rigidbodies/physics/constraint_graph.hpp>
#include <rigidbodies/physics/world.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {
        struct RowRef
        {
            ConstraintRow* row;
            std::size_t first, second;
        };
        struct Graph
        {
            std::vector<BodyId> ids;
            std::vector<RigidBody*> bodies;
            std::vector<std::vector<RowRef>> islands;
        };

        Graph build_graph(World& world, const std::vector<ConstraintPtr>& constraints)
        {
            Graph graph;
            graph.ids = world.body_ids();
            for (const auto id : graph.ids)
                graph.bodies.push_back(world.find_body(id));
            std::vector<std::size_t> parents(graph.ids.size());
            std::iota(parents.begin(), parents.end(), std::size_t {});
            const auto root = [&](std::size_t index)
            {
                while (parents[index] != index)
                    index = parents[index];
                return index;
            };
            const auto index_of = [&](BodyId id)
            {
                return static_cast<std::size_t>(std::find(graph.ids.begin(), graph.ids.end(), id) - graph.ids.begin());
            };
            std::vector<RowRef> rows;
            for (const auto& constraint : constraints)
            {
                if (!constraint || !constraint->is_enabled() || constraint->is_broken() ||
                    !world.is_valid(constraint->first_body()) || !world.is_valid(constraint->second_body()))
                    continue;
                auto* prepared = constraint->velocity_rows();
                if (!prepared)
                    continue;
                for (auto& row : *prepared)
                {
                    const auto a = index_of(row.first), b = index_of(row.second);
                    if (a >= graph.bodies.size() || b >= graph.bodies.size())
                        continue;
                    const auto active = [](const RigidBody& body)
                    {
                        return body.type() == BodyType::dynamic_body && body.is_awake();
                    };
                    if (!active(*graph.bodies[a]) && !active(*graph.bodies[b]))
                        continue;
                    if (!math::is_finite(row.linear_first) || !math::is_finite(row.linear_second) ||
                        !math::is_finite(row.angular_first) || !math::is_finite(row.angular_second) ||
                        !math::is_finite(row.target_velocity) || !math::is_finite(row.impulse) ||
                        std::isnan(row.lower_impulse) || std::isnan(row.upper_impulse) || row.lower_impulse > row.upper_impulse)
                        throw std::invalid_argument("Constraint rows must have finite Jacobians and ordered impulse bounds");
                    rows.push_back({ &row, a, b });
                    if (active(*graph.bodies[a]) && active(*graph.bodies[b]))
                    {
                        auto first = root(a), second = root(b);
                        if (first > second)
                            std::swap(first, second);
                        parents[second] = first;
                    }
                }
            }
            graph.islands.resize(graph.ids.size());
            for (const auto& row : rows)
            {
                const auto dynamic = graph.bodies[row.first]->type() == BodyType::dynamic_body && graph.bodies[row.first]->is_awake()
                    ? row.first
                    : row.second;
                graph.islands[root(dynamic)].push_back(row);
            }
            graph.islands.erase(std::remove_if(graph.islands.begin(), graph.islands.end(), [](const auto& island)
                                    {
                                        return island.empty();
                                    }),
                graph.islands.end());
            return graph;
        }

        Real speed(const RowRef& ref, const Graph& graph)
        {
            const auto& row = *ref.row;
            const auto& a = *graph.bodies[ref.first];
            const auto& b = *graph.bodies[ref.second];
            return math::dot(row.linear_first, a.linear_velocity_m_s()) + row.angular_first * a.angular_velocity_rad_s() +
                math::dot(row.linear_second, b.linear_velocity_m_s()) + row.angular_second * b.angular_velocity_rad_s();
        }

        Real coupling(const RowRef& left, const RowRef& right, const Graph& graph)
        {
            Real result = 0.0;
            const auto contribution = [&](std::size_t a, const math::Vec2& ja, Real wa, std::size_t b, const math::Vec2& jb, Real wb)
            {
                if (a == b && graph.bodies[a]->is_awake())
                    result += graph.bodies[a]->inverse_mass() * math::dot(ja, jb) + graph.bodies[a]->inverse_inertia() * wa * wb;
            };
            const auto& a = *left.row;
            const auto& b = *right.row;
            contribution(left.first, a.linear_first, a.angular_first, right.first, b.linear_first, b.angular_first);
            contribution(left.first, a.linear_first, a.angular_first, right.second, b.linear_second, b.angular_second);
            contribution(left.second, a.linear_second, a.angular_second, right.first, b.linear_first, b.angular_first);
            contribution(left.second, a.linear_second, a.angular_second, right.second, b.linear_second, b.angular_second);
            return result;
        }

        std::vector<Real> dense_solve(std::vector<Real> matrix, std::vector<Real> rhs)
        {
            const auto size = rhs.size();
            for (std::size_t pivot = 0; pivot < size; ++pivot)
            {
                auto best = pivot;
                for (std::size_t row = pivot + 1; row < size; ++row)
                    if (std::abs(matrix[row * size + pivot]) > std::abs(matrix[best * size + pivot]))
                        best = row;
                if (best != pivot)
                {
                    for (std::size_t column = pivot; column < size; ++column)
                        std::swap(matrix[pivot * size + column], matrix[best * size + column]);
                    std::swap(rhs[pivot], rhs[best]);
                }
                const auto diagonal = matrix[pivot * size + pivot];
                if (!(std::abs(diagonal) > std::numeric_limits<Real>::min()))
                    continue;
                for (std::size_t row = pivot + 1; row < size; ++row)
                {
                    const auto factor = matrix[row * size + pivot] / diagonal;
                    for (std::size_t column = pivot + 1; column < size; ++column)
                        matrix[row * size + column] -= factor * matrix[pivot * size + column];
                    rhs[row] -= factor * rhs[pivot];
                }
            }
            std::vector<Real> result(size);
            for (std::size_t reverse = size; reverse > 0; --reverse)
            {
                const auto row = reverse - 1;
                auto value = rhs[row];
                for (std::size_t column = row + 1; column < size; ++column)
                    value -= matrix[row * size + column] * result[column];
                const auto diagonal = matrix[row * size + row];
                result[row] = std::abs(diagonal) > std::numeric_limits<Real>::min() ? value / diagonal : 0.0;
            }
            return result;
        }

        void solve_island(Graph& graph, const std::vector<RowRef>& rows)
        {
            const auto size = rows.size();
            std::vector<Real> matrix(size * size), rhs(size), impulses(size), lower(size), upper(size);
            Real diagonal_scale = 0.0;
            for (std::size_t i = 0; i < size; ++i)
            {
                for (std::size_t j = 0; j <= i; ++j)
                    matrix[i * size + j] = matrix[j * size + i] = coupling(rows[i], rows[j], graph);
                diagonal_scale = std::max(diagonal_scale, matrix[i * size + i]);
                impulses[i] = rows[i].row->impulse;
                lower[i] = rows[i].row->lower_impulse;
                upper[i] = rows[i].row->upper_impulse;
            }
            if (!(diagonal_scale > 0.0))
                return;
            for (std::size_t i = 0; i < size; ++i)
            {
                rhs[i] = rows[i].row->target_velocity - speed(rows[i], graph);
                for (std::size_t j = 0; j < size; ++j)
                    rhs[i] += matrix[i * size + j] * impulses[j];
                matrix[i * size + i] += diagonal_scale * 1.0e-10;
            }
            std::vector<int> active(size);
            bool converged = false;
            for (std::size_t iteration = 0; iteration < 8 * size + 32; ++iteration)
            {
                std::vector<std::size_t> free;
                for (std::size_t i = 0; i < size; ++i)
                    if (active[i] == 0)
                        free.push_back(i);
                std::vector<Real> reduced(free.size() * free.size()), right(free.size());
                for (std::size_t i = 0; i < free.size(); ++i)
                {
                    right[i] = rhs[free[i]];
                    for (std::size_t j = 0; j < size; ++j)
                        if (active[j] != 0)
                            right[i] -= matrix[free[i] * size + j] * impulses[j];
                    for (std::size_t j = 0; j < free.size(); ++j)
                        reduced[i * free.size() + j] = matrix[free[i] * size + free[j]];
                }
                const auto solution = dense_solve(std::move(reduced), std::move(right));
                for (std::size_t i = 0; i < free.size(); ++i)
                    impulses[free[i]] = solution[i];
                bool changed = false;
                for (const auto i : free)
                {
                    if (impulses[i] < lower[i] || impulses[i] > upper[i])
                    {
                        active[i] = impulses[i] < lower[i] ? -1 : 1;
                        impulses[i] = math::clamp(impulses[i], lower[i], upper[i]);
                        changed = true;
                        break;
                    }
                }
                if (changed)
                    continue;
                for (std::size_t i = 0; i < size; ++i)
                {
                    if (active[i] == 0 || lower[i] == upper[i])
                        continue;
                    Real gradient = -rhs[i];
                    for (std::size_t j = 0; j < size; ++j)
                        gradient += matrix[i * size + j] * impulses[j];
                    if (static_cast<Real>(active[i]) * gradient > 1.0e-9)
                    {
                        active[i] = 0;
                        changed = true;
                        break;
                    }
                }
                if (!changed)
                {
                    converged = true;
                    break;
                }
            }
            // Degenerate bound combinations can cycle. A bounded projected sweep provides a
            // deterministic feasible fallback, without exposing an unbounded active-set iterate.
            if (!converged)
            {
                for (std::size_t i = 0; i < size; ++i)
                    impulses[i] = math::clamp(impulses[i], lower[i], upper[i]);
                for (std::size_t pass = 0; pass < 32; ++pass)
                    for (std::size_t i = 0; i < size; ++i)
                    {
                        auto residual = rhs[i];
                        for (std::size_t j = 0; j < size; ++j)
                            residual -= matrix[i * size + j] * impulses[j];
                        impulses[i] = math::clamp(impulses[i] + residual / matrix[i * size + i], lower[i], upper[i]);
                    }
            }
            std::vector<math::Vec2> linear(graph.bodies.size());
            std::vector<Real> angular(graph.bodies.size());
            for (std::size_t i = 0; i < size; ++i)
            {
                auto& row = *rows[i].row;
                if (!math::is_finite(impulses[i]))
                    throw std::overflow_error("Constraint graph produced an unrepresentable impulse");
                const auto delta = impulses[i] - row.impulse;
                row.impulse = impulses[i];
                linear[rows[i].first] += row.linear_first * delta;
                linear[rows[i].second] += row.linear_second * delta;
                angular[rows[i].first] += row.angular_first * delta;
                angular[rows[i].second] += row.angular_second * delta;
            }
            for (std::size_t i = 0; i < graph.bodies.size(); ++i)
            {
                auto& body = *graph.bodies[i];
                if (body.type() == BodyType::dynamic_body && body.is_awake())
                    body.set_simulated_velocity(body.linear_velocity_m_s() + linear[i] * body.inverse_mass(),
                        body.angular_velocity_rad_s() + angular[i] * body.inverse_inertia());
            }
        }

        ConstraintGraphStatistics statistics(const Graph& graph)
        {
            ConstraintGraphStatistics result;
            result.island_count = graph.islands.size();
            for (const auto& island : graph.islands)
                for (const auto& row : island)
                {
                    ++result.row_count;
                    const auto& equation = *row.row;
                    const auto mismatch = equation.target_velocity - speed(row, graph);
                    auto residual = std::abs(mismatch);
                    const auto tolerance = 1.0e-9 * std::max(1.0, std::abs(equation.impulse));
                    if (equation.lower_impulse == equation.upper_impulse)
                        residual = 0.0;
                    else if (equation.impulse <= equation.lower_impulse + tolerance)
                        residual = std::max(0.0, mismatch);
                    else if (equation.impulse >= equation.upper_impulse - tolerance)
                        residual = std::max(0.0, -mismatch);
                    result.velocity_residual = std::max(result.velocity_residual, residual);
                }
            return result;
        }
    }

    ConstraintGraphStatistics solve_constraint_graph(World& world, const std::vector<ConstraintPtr>& constraints)
    {
        auto graph = build_graph(world, constraints);
        for (const auto& island : graph.islands)
            solve_island(graph, island);
        return statistics(graph);
    }

    ConstraintGraphStatistics measure_constraint_graph(World& world, const std::vector<ConstraintPtr>& constraints)
    {
        return statistics(build_graph(world, constraints));
    }
}
