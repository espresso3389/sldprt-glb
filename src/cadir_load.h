// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string_view>

#include "model.h"

namespace swglb {

/// Parse a cadmpeg CADIR JSON document into the Model fields the reference writer
/// reads (verification aid for the GLB writer).
Model load_cadir(std::string_view text);

}  // namespace swglb
