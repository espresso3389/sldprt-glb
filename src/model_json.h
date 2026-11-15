// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ostream>

#include "model.h"

namespace swglb {

/// Write `model` as CADIR-shaped JSON: {"units":{...},"model":{...}} using
/// cadmpeg field names, so tools/compare_cadir.py can diff it against the
/// reference CADIR.
void write_model_json(std::ostream& out, const Model& model, bool include_mesh = true);

}  // namespace swglb
