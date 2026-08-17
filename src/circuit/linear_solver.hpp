#pragma once

#include <optional>
#include <vector>

namespace srp::circuit::detail
{

std::optional<std::vector<double>> solveLinearSystem(
    std::vector<std::vector<double>> matrix,
    std::vector<double> right_hand_side);

}  // namespace srp::circuit::detail
