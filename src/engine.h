// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "container.h"
#include "model.h"

namespace swglb {

struct DecodeOptions {
  bool brep = true;  ///< Decode Parasolid B-rep (needed for face colours and edges).
};

/// Full decode pipeline (see docs/DESIGN.md). Throws DecodeError.
Model decode_document(const Container& container, const DecodeOptions& options = {});

}  // namespace swglb
