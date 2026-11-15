// SPDX-License-Identifier: Apache-2.0
// Scoped native appearance ownership (cadmpeg-codec-sldprt
// `appearance/visual_states.rs`, cadmpeg 5af95421, Apache-2.0): face colours
// from ThirdPtyStore/VisualStates and the feature sources the display stage
// may bind.
#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "container.h"
#include "model.h"
#include "stages.h"

namespace swglb {

struct NativePayloads {
  /// FACE_ID_2001 payloads of emitted faces (values[1] source, values[2] timestamp).
  std::vector<std::pair<std::string, std::vector<uint32_t>>> faces;
  /// ATOM_ID_2001 payloads of emitted bodies.
  std::vector<std::pair<std::string, std::vector<uint32_t>>> bodies;
};

/// Resolve the native appearance plan into `plan` (face id order; requires
/// the configurations to be assigned). Returns the plan's feature sources.
/// The plan is applied after tessellation ownership (apply_visual_state_plan).
std::set<uint32_t> resolve_visual_states(const Container& container, const NativePayloads& payloads,
                                         const Model& model, std::vector<VisualStateChoice>& plan,
                                         std::vector<UnresolvedAppearance>& unresolved);

}  // namespace swglb
