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

    for (std::size_t column = 0; column < size; ++column)
    {
        std::size_t pivot = column;
        double pivot_magnitude = std::abs(matrix[column][column]);
        for (std::size_t row = column + 1; row < size; ++row)
        {
            const double candidate = std::abs(matrix[row][column]);
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
            std::swap(matrix[pivot], matrix[column]);
            std::swap(right_hand_side[pivot], right_hand_side[column]);
        }

        for (std::size_t row = column + 1; row < size; ++row)
        {
            const double factor = matrix[row][column] / matrix[column][column];
            matrix[row][column] = 0.0;
            for (std::size_t entry = column + 1; entry < size; ++entry)
            {
                matrix[row][entry] -= factor * matrix[column][entry];
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
            value -= matrix[row][entry] * solution[entry];
        }

        solution[row] = value / matrix[row][row];
        if (!finite(solution[row]))
        {
            return std::nullopt;
        }
    }

    return solution;
}

}  // namespace srp::circuit::detail
