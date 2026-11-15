// SPDX-License-Identifier: Apache-2.0
// Stale display-mesh regeneration: the reference
// `B-rep -> STEP -> OCCT tessellation -> replace_surface` path, done in
// process by building the OCCT shape directly from the decoded B-rep.
#pragma once

#include "bytes.h"
#include "glb.h"
#include "model.h"

namespace swglb {

/// Replace the surface primitives of `saved_glb` (make_glb output) with a
/// tessellation of the model's B-rep. Throws DecodeError when the B-rep can
/// not be rebuilt or its bounds do not match `expected` (the reference writer then
/// falls back to the saved image).
Bytes regenerate_stale_surface(const Model& model, const Bytes& saved_glb, const Bounds& expected,
                               const std::string& extras_key);

/// The intermediate B-rep GLB alone (the reference STEP tessellation equivalent).
Bytes regenerate_brep_glb(const Model& model);
/// the reference writer replace_surface on an externally produced B-rep GLB.
Bytes replace_surface_glb(const Bytes& saved_glb, const Bytes& brep_glb, const Bounds& expected,
                          const std::string& extras_key);

}  // namespace swglb
