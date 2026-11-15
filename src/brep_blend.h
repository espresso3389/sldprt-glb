// SPDX-License-Identifier: Apache-2.0
// Solved caches of constant-radius rolling-ball blends: the ball's circular
// section swept along the spine between its two contact points, a rational
// surface cubic along the spine and quadratic across it
// (cadmpeg-codec-sldprt `brep/blend_surface.rs`, cadmpeg 5af95421,
// Apache-2.0).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "model.h"

namespace swglb::brep {

/// What a blend states about its surface, in model millimetres.
struct BlendInputs {
  const CurveGeometry* spine = nullptr;               ///< Locus of the ball centre.
  std::array<const SurfaceGeometry*, 2> supports{};  ///< Solved supports, stored order.
  double radius = 0;
  const std::vector<Vec3>* boundary = nullptr;  ///< Points on the face boundary.
  const std::vector<Vec3>* vertices = nullptr;  ///< The face's vertices.
  double tolerance = 1.0e-6;                    ///< Largest distance the cache may add.
};

struct BlendCache {
  NurbsSurfaceGeometry surface;
  double fit_tolerance = 0;
};

struct SolvedBlend {
  BlendCache cache;
  /// Every face vertex lies on the cache within its fit tolerance and the
  /// linear resolution.
  bool holds = false;
};

/// Points along one boundary edge over `range`, or over the whole period of
/// a circle or ellipse that states none (cadmpeg `edge_samples`).
std::vector<Vec3> blend_edge_samples(const CurveGeometry& curve,
                                     std::optional<std::array<double, 2>> range);

/// The cache of one blend over the stretch of spine its face covers
/// (cadmpeg `blend_surface::solve`).
std::optional<SolvedBlend> solve_blend(const BlendInputs& inputs);

/// The largest distance of a vertex from `cache` (cadmpeg `vertex_miss`).
std::optional<double> blend_vertex_miss(const BlendCache& cache, const CurveGeometry& spine,
                                        const std::vector<Vec3>& vertices);

/// The point of a solved curve carrier at `t` (line, circle, ellipse, NURBS
/// with the end spans extended).
std::optional<Vec3> blend_curve_point(const CurveGeometry& curve, double t);

/// The closest point of an analytic or NURBS support to `point`, with the
/// NURBS parameters that seed the next search (cadmpeg `foot`).
std::optional<std::pair<Vec3, std::optional<std::array<double, 2>>>> blend_support_foot(
    const SurfaceGeometry& support, Vec3 point, std::optional<std::array<double, 2>> seed);

// --- Fitted caches (cadmpeg-codec-sldprt `brep/blend_fit.rs`): a support
// pair, a nested blend, or a polyline spine chart.

/// The carriers a fitted cache reads (cadmpeg `CarrierIndex`), in stream
/// raw indices. Returned pointers stay valid for the resolver's lifetime.
class BlendCarriers {
 public:
  struct Blend {
    std::array<uint32_t, 2> supports{};
    uint32_t spine = 0;
    double signed_radius = 0;  ///< mm
  };
  virtual ~BlendCarriers() = default;
  /// A solved surface carrier (analytic or B-surface).
  virtual const SurfaceGeometry* surface(uint32_t index) = 0;
  virtual const CurveGeometry* curve(uint32_t index) = 0;
  virtual std::optional<Blend> blend(uint32_t index) = 0;
  /// The members of a zero-offset support pair.
  virtual std::optional<std::array<uint32_t, 2>> pair(uint32_t index) = 0;
  /// The blend a blend-boundary record names.
  virtual std::optional<uint32_t> bound(uint32_t index) = 0;
};

/// The member of a support pair the blend rolls on (cadmpeg
/// `blend_fit::pair_member`).
std::optional<size_t> blend_pair_member(const CurveGeometry& spine,
                                        std::array<const SurfaceGeometry*, 2> members,
                                        double signed_radius, const std::vector<Vec3>& vertices);

/// The fitted cache of a blend that `solve_blend` leaves without one (graph.rs
/// fallback: a support pair's wedge, else the support's surface or nested
/// blend). `references` are the stored supports, `resolved` the supports
/// after pair resolution.
std::optional<BlendCache> fit_blend(BlendCarriers& carriers, const CurveGeometry& spine,
                                    std::array<uint32_t, 2> references,
                                    std::array<uint32_t, 2> resolved, double signed_radius,
                                    const std::vector<Vec3>& vertices);

}  // namespace swglb::brep
