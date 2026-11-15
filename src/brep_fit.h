// SPDX-License-Identifier: Apache-2.0
// The solved cache of a parametric curve on a support: a cubic model-space
// B-curve fitted through the support in the pcurve's own parameter
// (cadmpeg-codec-sldprt `brep/fit.rs`, cadmpeg 5af95421, Apache-2.0).
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <vector>

#include "model.h"

namespace swglb::brep {

/// The fit's bound, millimetres.
inline constexpr double kFitToleranceMm = 1.0e-5;

/// A cubic B-curve within kFitToleranceMm of `point` (millimetres) over
/// `range`, in `point`'s parameter. `breaks` are the pcurve's knots. A span
/// that `resolved(low, high)` rejects is halved. nullopt when `point` does not
/// evaluate or the bound is not reached.
std::optional<NurbsCurveGeometry> fit_curve(const std::function<std::optional<Vec3>(double)>& point,
                                            std::array<double, 2> range,
                                            const std::vector<double>& breaks,
                                            const std::function<bool(double, double)>& resolved);

}  // namespace swglb::brep
