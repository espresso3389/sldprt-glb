// SPDX-License-Identifier: Apache-2.0
// GLB output: port of the reference CADIR-to-GLB writer.
#pragma once

#include <string>

#include <array>
#include <utility>
#include <vector>

#include "bytes.h"
#include "json_value.h"
#include "model.h"

namespace swglb {

struct GlbOptions {
  bool part = true;  ///< SLDPRT: allow stale-mesh regeneration from the B-rep.
  std::string generator = "sldprt-glb";               ///< asset.generator
  std::string extras_key = "solidworksPresentation";  ///< node extras key for presentation data
};

/// Build the GLB. Throws DecodeError when there is no usable display data
/// (the reference writer's ValueError paths).
Bytes write_glb(const Model& model, const GlbOptions& options);

/// glTF-space point (metres, Y up) and axis-aligned bounds (min, max).
using P3 = std::array<double, 3>;
using Bounds = std::pair<P3, P3>;

/// the reference writer wire_edges: CAD boundary line segments (pairs of points).
/// Throws DecodeError for unsupported edges.
std::vector<P3> wire_edges(const Model& model);
/// the reference writer bounds_differ.
bool bounds_differ(const Bounds& a, const Bounds& b);
/// the reference writer glb_encode (sets buffers[0].byteLength, pads both chunks).
Bytes glb_encode(JValue doc, Bytes binary);

}  // namespace swglb
