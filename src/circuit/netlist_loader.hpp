#pragma once

#include "circuit/circuit_model.hpp"

#include <filesystem>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

namespace srp::circuit
{

// Loads a JSON netlist into a CircuitModel. The format is documented in
// docs/netlist-format.md; node names "gnd", "ground" and "0" refer to the
// built-in ground node, and nodes not declared explicitly are created on
// first use.
std::optional<CircuitModel> loadNetlist(
    const nlohmann::json& json,
    std::string& error);

// Reads a netlist JSON file from disk and loads it.
std::optional<CircuitModel> loadNetlistFile(
    const std::filesystem::path& path,
    std::string& error);

}  // namespace srp::circuit
