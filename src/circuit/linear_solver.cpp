#include "circuit/linear_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace srp::circuit::detail
{

namespace
{

constexpr double kSingularTolerance = 1e-12;

bool finite(double value)
{
    return std::isfinite(value);
}

}  // namespace

std::optional<std::vector<double>> solveLinearSystem(
    std::vector<std::vector<double>> matrix,
    std::vector<double> right_hand_side)
{
    const std::size_t size = right_hand_side.size();
    if (size == 0)
    {
        return std::vector<double>{};
    }

    // Flatten to a single row-major buffer: the elimination loops dominate
    // (O(n^3) flops), and contiguous storage avoids a pointer chase per
    // element while keeping the arithmetic order bit-identical.
    std::vector<double> flat(size * size);
    for (std::size_t row = 0; row < size; ++row)
    {
        const std::vector<double>& source = matrix[row];
        for (std::size_t column = 0; column < size; ++column)
        {
            flat[row * size + column] = source[column];
        }
    }

    for (std::size_t column = 0; column < size; ++column)
    {
        std::size_t pivot = column;
        double pivot_magnitude = std::abs(flat[column * size + column]);
        for (std::size_t row = column + 1; row < size; ++row)
        {
            const double candidate = std::abs(flat[row * size + column]);
            if (candidate > pivot_magnitude)
            {
                pivot = row;
                pivot_magnitude = candidate;
            }
        }

        if (pivot_magnitude < kSingularTolerance)
        {
            return std::nullopt;
        }

        if (pivot != column)
        {
            for (std::size_t entry = 0; entry < size; ++entry)
            {
                std::swap(
                    flat[pivot * size + entry],
                    flat[column * size + entry]);
            }
            std::swap(right_hand_side[pivot], right_hand_side[column]);
        }

        for (std::size_t row = column + 1; row < size; ++row)
        {
            const double factor =
                flat[row * size + column] / flat[column * size + column];
            flat[row * size + column] = 0.0;
            for (std::size_t entry = column + 1; entry < size; ++entry)
            {
                flat[row * size + entry] -=
                    factor * flat[column * size + entry];
            }
            right_hand_side[row] -= factor * right_hand_side[column];
        }
    }

    std::vector<double> solution(size, 0.0);
    for (std::size_t row = size; row-- > 0;)
    {
        double value = right_hand_side[row];
        for (std::size_t entry = row + 1; entry < size; ++entry)
        {
            value -= flat[row * size + entry] * solution[entry];
        }

        solution[row] = value / flat[row * size + row];
        if (!finite(solution[row]))
        {
            return std::nullopt;
        }
    }

    return solution;
}

}  // namespace srp::circuit::detail
