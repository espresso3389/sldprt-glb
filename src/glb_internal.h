// SPDX-License-Identifier: Apache-2.0
// Internal pieces of glb.cc and regenerate.cc, exposed for tests/glb_test.cc.
// Ports of the reference writer's table_color, reference_plane_color,
// stale_surface_bounds and glb_parts.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <unordered_map>

#include "bytes.h"
#include "glb.h"
#include "json_value.h"
#include "model.h"

namespace swglb::glb_internal {

/// Indexes for table_color.
struct Lookup {
  std::unordered_map<std::string, const Face*> faces;
  std::unordered_map<std::string, const Body*> bodies;
  /// Tessellation id -> bound appearance colour (nullopt: bound appearance
  /// without base colour).
  std::unordered_map<std::string, std::optional<std::array<double, 4>>> bound;
};

/// the reference writer table_color (returns nullopt for None): linear RGB +
/// alpha. Throws DecodeError for an out-of-range colour.
std::optional<std::array<double, 4>> table_color(const Tessellation& table, const Lookup& lookup);

/// the reference writer reference_plane_color: (selector, srgb, resolved, source).
struct PlaneColor {
  JValue selector;  // int or null
  std::array<double, 3> srgb;
  bool resolved;
  const char* source;
};

/// the reference writer reference_plane_color (its indexed branch is
/// reference_pen_color).
PlaneColor reference_plane_color(const ReferencePlane& plane);

/// the reference writer stale_surface_bounds, given the POSITION bounds of the
/// first surface primitive of the make_glb output.
std::optional<Bounds> stale_surface_bounds(const Model& model, const Bounds& surface);

struct GlbParts {
  JValue doc;
  Bytes binary;
};

/// the reference writer glb_parts. Throws DecodeError for a malformed GLB.
GlbParts glb_parts(const Bytes& blob);

}  // namespace swglb::glb_internal
