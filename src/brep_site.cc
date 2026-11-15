// SPDX-License-Identifier: Apache-2.0
// Site decoder: the topology walk, carrier typing and synthetic seam passes
// of cadmpeg-codec-sldprt `brep/graph.rs` (cadmpeg 5af95421, Apache-2.0),
// re-expressed over libopenparasolid's decoded node graph.
#include "brep_site.h"
#include "brep_blend.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <cstdlib>
#include <map>
#include <numbers>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "brep_fit.h"
#include "brep_xt.h"
#include "openparasolid/evaluate.h"
#include <cstdio>

namespace swglb {
namespace {

using openparasolid::DecodedNode;

constexpr double kMm = 1000.0;
constexpr double kTau = 2.0 * std::numbers::pi;
constexpr double kLinearResolution = 1.0e-5;
constexpr double kStoredRangeTolerance = 1.0e-7;
constexpr double kFullTurnTolerance = 1.0e-9;
constexpr double kEpsPointDistance = 1.0e-12;
constexpr double kF64Eps = 2.220446049250313e-16;
constexpr int64_t kSensePlus = '+';
constexpr int64_t kSenseMinus = '-';

// ---------------------------------------------------------------------------
// Small vector helpers
// ---------------------------------------------------------------------------
Vec3 v3(const double* p) { return Vec3{p[0], p[1], p[2]}; }
Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 scale(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(Vec3 a) { return std::hypot(std::hypot(a.x, a.y), a.z); }
double dist(Vec3 a, Vec3 b) { return norm(sub(a, b)); }
bool finite(const std::vector<double>& v) {
  return std::all_of(v.begin(), v.end(), [](double d) { return std::isfinite(d); });
}
// cadmpeg OrthonormalFrame3::new admits unit, perpendicular vectors.
bool unit(Vec3 a) { return std::abs(norm(a) - 1.0) <= 1.0e-6; }
bool frame_ok(Vec3 axis, Vec3 ref) {
  return unit(axis) && unit(ref) && std::abs(dot(axis, ref)) <= 1.0e-6;
}
double rem_euclid(double a, double b) {
  double r = std::fmod(a, b);
  if (r < 0) r += std::abs(b);
  return r;
}
std::array<double, 2> periodic_phase(std::array<double, 2> r) {
  double phase = rem_euclid(r[0], kTau);
  if (phase >= kTau) phase = 0.0;
  double shift = phase - r[0];
  return {phase, r[1] + shift};
}
double looser(double a, double b) { return a >= b ? a : b; }

/// Memoised carrier lookups for the blend fit (cadmpeg `CarrierIndex`).
class FnBlendCarriers : public brep::BlendCarriers {
 public:
  std::function<std::optional<SurfaceGeometry>(uint32_t)> surface_fn;
  std::function<std::optional<CurveGeometry>(uint32_t)> curve_fn;
  std::function<std::optional<Blend>(uint32_t)> blend_fn;
  std::function<std::optional<std::array<uint32_t, 2>>(uint32_t)> pair_fn;
  std::function<std::optional<uint32_t>(uint32_t)> bound_fn;

  const SurfaceGeometry* surface(uint32_t index) override {
    auto it = surfaces_.find(index);
    if (it == surfaces_.end()) it = surfaces_.emplace(index, surface_fn(index)).first;
    return it->second ? &*it->second : nullptr;
  }
  const CurveGeometry* curve(uint32_t index) override {
    auto it = curves_.find(index);
    if (it == curves_.end()) it = curves_.emplace(index, curve_fn(index)).first;
    return it->second ? &*it->second : nullptr;
  }
  std::optional<Blend> blend(uint32_t index) override { return blend_fn(index); }
  std::optional<std::array<uint32_t, 2>> pair(uint32_t index) override { return pair_fn(index); }
  std::optional<uint32_t> bound(uint32_t index) override { return bound_fn(index); }

 private:
  std::map<uint32_t, std::optional<SurfaceGeometry>> surfaces_;
  std::map<uint32_t, std::optional<CurveGeometry>> curves_;
};

std::string ident(const char* ns, uint32_t attr) {
  std::string s = "sldprt:brep:";
  s += ns;
  s += '#';
  s += std::to_string(attr);
  return s;
}

// ---------------------------------------------------------------------------
// Carriers
// ---------------------------------------------------------------------------
struct CurveCarrier {
  CurveGeometry g;
  bool typed = false;  ///< false: emitted as an opaque "unknown" curve
  std::optional<std::array<double, 2>> range;  ///< stored parameter range (source units)
  std::optional<bool> marker_reversed;         ///< analytic sense '-'
  bool closed_domain = false;                  ///< circle/ellipse/closed spline
  std::array<double, 2> domain{0.0, kTau};
  std::optional<std::array<Vec3, 2>> ends;     ///< spline domain end points
  bool on_sweep = false;                       ///< SP_CURVE on a swept/spun support
  /// INTERSECTION carriers: (parameter, position mm) chart nodes in parameter
  /// order, and the chordal error in mm.
  std::vector<std::pair<double, Vec3>> chart;
  std::optional<double> chart_tolerance;
  /// SP_CURVE carriers: stored (trimmed) or evaluated (whole) end points, mm,
  /// the low end of the parameter range first.
  std::optional<std::array<Vec3, 2>> surface_ends;
};

struct SurfaceCarrier {
  SurfaceGeometry g;
  bool orientation_reversed = false;  ///< torus with negative major radius
  bool marker_reversed = false;       ///< analytic sense '-'
  bool analytic = false;
  bool loopless = false;              ///< sphere, torus, doubly periodic spline
};

class Decoder {
 public:
  explicit Decoder(const xt::Stream& s) : s_(s) {}
  SiteBrep run();

 private:
  const xt::Stream& s_;
  std::unordered_map<uint32_t, std::optional<CurveCarrier>> curve_cache_;

  int64_t field(uint32_t index, std::string_view name) const {
    const auto* n = s_.node(index);
    return n ? s_.integer(*n, name) : 0;
  }
  uint32_t ptr(uint32_t index, std::string_view name) const { return s_.ptr(index, name); }

  std::optional<CurveCarrier> curve(uint32_t index, int depth = 0);
  std::optional<CurveCarrier> analytic_curve(uint32_t index);
  void intersection_chart(const openparasolid::IntersectionCurveGeometry& ic, CurveCarrier& c);
  std::optional<CurveCarrier> bounded_chart(const CurveCarrier& basis, std::array<double, 2> parameters,
                                            std::array<Vec3, 2> points);
  std::optional<CurveCarrier> surface_curve(uint32_t index, std::optional<std::array<double, 2>> trim,
                                           uint32_t trim_index = 0);
  std::optional<SurfaceCarrier> surface(uint32_t index);
  std::optional<Vec3> point_position(uint32_t point) const;
  std::optional<Vec3> curve_point(const CurveCarrier& c, double t) const;
};

std::optional<NurbsCurveGeometry> nurbs_curve(const openparasolid::NurbsCurveGeometry& g) {
  NurbsCurveGeometry out;
  out.degree = g.axis.degree;
  out.periodic = g.axis.periodic;
  for (size_t i = 0; i < g.axis.knots.size() && i < g.axis.multiplicities.size(); ++i)
    out.knots.insert(out.knots.end(), g.axis.multiplicities[i], g.axis.knots[i]);
  size_t dim = g.controls.dimension;
  if (dim < 3 || dim > 4) return std::nullopt;
  bool rational = g.controls.rational;
  size_t spatial = rational ? dim - 1 : dim;
  if (spatial != 3) return std::nullopt;
  size_t count = g.controls.values.size() / dim;
  for (size_t i = 0; i < count; ++i) {
    const double* p = &g.controls.values[i * dim];
    double w = rational ? p[spatial] : 1.0;
    if (!(w > 0)) return std::nullopt;
    out.poles.push_back(Vec3{p[0] / w * kMm, p[1] / w * kMm, p[2] / w * kMm});
    if (rational) out.weights.push_back(w);
  }
  return out;
}

std::optional<NurbsSurfaceGeometry> nurbs_surface(const openparasolid::NurbsSurfaceGeometry& g) {
  NurbsSurfaceGeometry out;
  out.u_degree = g.u.degree;
  out.v_degree = g.v.degree;
  out.u_periodic = g.u.periodic;
  out.v_periodic = g.v.periodic;
  for (size_t i = 0; i < g.u.knots.size() && i < g.u.multiplicities.size(); ++i)
    out.u_knots.insert(out.u_knots.end(), g.u.multiplicities[i], g.u.knots[i]);
  for (size_t i = 0; i < g.v.knots.size() && i < g.v.multiplicities.size(); ++i)
    out.v_knots.insert(out.v_knots.end(), g.v.multiplicities[i], g.v.knots[i]);
  size_t dim = g.controls.dimension;
  bool rational = g.controls.rational;
  if (dim != (rational ? 4u : 3u)) return std::nullopt;
  size_t nu = g.u.control_count, nv = g.v.control_count;
  if (g.controls.values.size() != nu * nv * dim) return std::nullopt;
  out.poles.assign(nu, std::vector<Vec3>(nv));
  if (rational) out.weights.assign(nu, std::vector<double>(nv, 1.0));
  for (size_t i = 0; i < nu; ++i)
    for (size_t j = 0; j < nv; ++j) {
      // XT stores contiguous v rows: control(u, v) at u * n_v + v.
      const double* p = &g.controls.values[(i * nv + j) * dim];
      double w = rational ? p[3] : 1.0;
      if (!(w > 0)) return std::nullopt;
      out.poles[i][j] = Vec3{p[0] / w * kMm, p[1] / w * kMm, p[2] / w * kMm};
      if (rational) out.weights[i][j] = w;
    }
  return out;
}

/// A clamped spline's natural domain [knot[p], knot[n]]; it is a closed
/// traversal when its end points coincide exactly (cadmpeg
/// `closed_traversal_range` + `curve_endpoints_coincident(.., 0.0)`).
void set_spline_domain(CurveCarrier& c) {
  const auto& n = *c.g.nurbs;
  size_t count = n.poles.size();
  if (n.knots.size() < count + n.degree + 1 || count == 0) return;
  c.domain = {n.knots[n.degree], n.knots[count]};
  const Vec3& a = (*c.ends)[0];
  const Vec3& b = (*c.ends)[1];
  c.closed_domain = c.domain[1] > c.domain[0] && a.x == b.x && a.y == b.y && a.z == b.z;
}

namespace {

/// The count of distinct knots at or below `value`: the piece it is in.
size_t distinct_knots_at_or_below(const std::vector<double>& knots, double value) {
  size_t pieces = 0;
  std::optional<double> previous;
  for (double k : knots) {
    if (k > value) break;
    if (!previous || *previous != k) ++pieces;
    previous = k;
  }
  return pieces;
}

bool within_two(const std::array<size_t, 3>& pieces) {
  auto [lo, hi] = std::minmax_element(pieces.begin(), pieces.end());
  return *hi - *lo <= 1;
}

std::vector<double> expanded(const openparasolid::SplineAxis& axis) {
  std::vector<double> out;
  for (size_t i = 0; i < axis.knots.size() && i < axis.multiplicities.size(); ++i)
    out.insert(out.end(), axis.multiplicities[i], axis.knots[i]);
  return out;
}

}  // namespace

namespace {

double distance(const Vec3& a, const Vec3& b) {
  double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

Vec3 mm(const openparasolid::Vector3& v) { return Vec3{v.x * kMm, v.y * kMm, v.z * kMm}; }

/// The terminator position nearest `endpoint` (cadmpeg `nearest_term`).
std::optional<Vec3> nearest_term(const openparasolid::IntersectionLimit& limit, const Vec3& endpoint) {
  std::optional<Vec3> best;
  double best_d = 0;
  for (const auto& p : limit.positions) {
    Vec3 q = mm(p);
    double d = distance(q, endpoint);
    if (!best || d < best_d) best = q, best_d = d;
  }
  return best;
}

/// The point of a degree-one chart at `t` (cadmpeg `chart_point`).
std::optional<Vec3> chart_point(const std::vector<std::pair<double, Vec3>>& nodes, double t) {
  for (size_t i = 0; i + 1 < nodes.size(); ++i) {
    auto [a, from] = nodes[i];
    auto [b, to] = nodes[i + 1];
    if (a <= t && t <= b) {
      double f = b > a ? (t - a) / (b - a) : 0.0;
      return Vec3{from.x + (to.x - from.x) * f, from.y + (to.y - from.y) * f,
                  from.z + (to.z - from.z) * f};
    }
  }
  return std::nullopt;
}

}  // namespace

/// The degree-one chart curve of an INTERSECTION (cadmpeg `solved_curve`):
/// knots at the base parameter plus the scaled chord length, terminators in
/// place of the chart's end positions. Without terminators or a monotone
/// parameterisation, the chart positions at index knots.
void Decoder::intersection_chart(const openparasolid::IntersectionCurveGeometry& ic, CurveCarrier& c) {
  const auto& pts = ic.chart.positions;
  if (pts.size() < 2) return;
  NurbsCurveGeometry nc;
  nc.degree = 1;
  std::vector<Vec3> poles;
  for (const auto& p : pts) poles.push_back(mm(p));
  auto first = nearest_term(ic.start, poles.front());
  auto last = nearest_term(ic.end, poles.back());
  std::vector<double> params{ic.chart.base_parameter};
  double t = ic.chart.base_parameter;
  for (size_t i = 1; i < pts.size(); ++i) {
    double dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y, dz = pts[i].z - pts[i - 1].z;
    t += std::sqrt(dx * dx + dy * dy + dz * dz) * ic.chart.base_scale;
    params.push_back(t);
  }
  bool ascending = true, descending = true;
  for (size_t i = 1; i < params.size(); ++i) {
    ascending = ascending && params[i - 1] < params[i];
    descending = descending && params[i - 1] > params[i];
  }
  if (first && last && (ascending || descending) && std::isfinite(t)) {
    poles.front() = *first;
    poles.back() = *last;
    if (descending) {
      std::reverse(params.begin(), params.end());
      std::reverse(poles.begin(), poles.end());
    }
    nc.knots.push_back(params.front());
    nc.knots.insert(nc.knots.end(), params.begin(), params.end());
    nc.knots.push_back(params.back());
    nc.poles = poles;
    for (size_t i = 0; i < poles.size(); ++i) c.chart.emplace_back(params[i], poles[i]);
    if (ic.chart.chordal_error && std::isfinite(*ic.chart.chordal_error))
      c.chart_tolerance = *ic.chart.chordal_error * kMm;
  } else {
    nc.knots.push_back(0);
    for (size_t i = 0; i < poles.size(); ++i) nc.knots.push_back(static_cast<double>(i));
    nc.knots.push_back(nc.knots.back());
    nc.poles = poles;
  }
  c.ends = std::array<Vec3, 2>{nc.poles.front(), nc.poles.back()};
  c.g.nurbs = std::move(nc);
  set_spline_domain(c);
}

/// A TRIMMED_CURVE on a chart curve (cadmpeg `IntersectionCarrier::bounded`):
/// the sub-chart between its parameters when its stored ends lie on the
/// chart within the chordal error.
std::optional<CurveCarrier> Decoder::bounded_chart(const CurveCarrier& basis,
                                                   std::array<double, 2> parameters,
                                                   std::array<Vec3, 2> points) {
  const auto& nodes = basis.chart;
  if (nodes.size() < 2 || !basis.chart_tolerance) return std::nullopt;
  double low = nodes.front().first, high = nodes.back().first;
  Vec3 seam = nodes.front().second, last_pole = nodes.back().second;
  double tolerance = *basis.chart_tolerance;
  bool ascending = parameters[0] < parameters[1];
  double from, to;
  Vec3 from_point, to_point;
  if (ascending) {
    from = parameters[0], to = parameters[1], from_point = points[0], to_point = points[1];
  } else if (parameters[0] > parameters[1]) {
    from = parameters[1], to = parameters[0], from_point = points[1], to_point = points[0];
  } else {
    return std::nullopt;
  }
  if (to <= low || from >= high) return std::nullopt;
  for (int i = 0; i < 2; ++i) {
    auto on = chart_point(nodes, std::clamp(parameters[i], low, high));
    if (!on || distance(*on, points[i]) > tolerance) return std::nullopt;
  }
  // A ring wrapper ends at the seam.
  if (seam.x == last_pole.x && seam.y == last_pole.y && seam.z == last_pole.z &&
      distance(points[0], points[1]) <= kLinearResolution) {
    auto on = chart_point(nodes, std::clamp(from, low, high));
    if (!on || distance(*on, seam) > tolerance) return std::nullopt;
    to = high;
    to_point = seam;
  }
  NurbsCurveGeometry nc;
  nc.degree = 1;
  nc.knots = {from, from};
  nc.poles = {from_point};
  for (const auto& [knot, pole] : nodes)
    if (knot > from && knot < to && knot > low && knot < high &&
        distance(pole, from_point) > kLinearResolution && distance(pole, to_point) > kLinearResolution) {
      nc.knots.push_back(knot);
      nc.poles.push_back(pole);
    }
  nc.knots.push_back(to);
  nc.knots.push_back(to);
  nc.poles.push_back(to_point);
  CurveCarrier c;
  c.g.kind = "nurbs";
  c.typed = true;
  c.ends = std::array<Vec3, 2>{nc.poles.front(), nc.poles.back()};
  c.g.nurbs = std::move(nc);
  set_spline_domain(c);
  c.range = ascending ? std::array<double, 2>{from, to} : std::array<double, 2>{to, from};
  return c;
}

/// An SP_CURVE as a curve (cadmpeg `attach_surface_curve`): the cubic
/// B-curve fitted through its support in the pcurve's parameter, over the
/// trim interval or the pcurve's whole knot domain. Untyped when the support
/// does not evaluate or the fit misses its bound.
std::optional<CurveCarrier> Decoder::surface_curve(uint32_t index,
                                                   std::optional<std::array<double, 2>> trim,
                                                   uint32_t trim_index) {
  CurveCarrier c;
  c.g.kind = "unknown";
  const openparasolid::Model* model = s_.model();
  if (uint32_t support = ptr(index, "surface"))
    c.on_sweep = s_.is(support, xt::kSweptSurf) || s_.is(support, xt::kSpunSurf);
  const auto* g = s_.geometry(index);
  const auto* sp = g ? std::get_if<openparasolid::SurfaceCurveGeometry>(&g->payload) : nullptr;
  const auto* pg = sp ? s_.geometry(sp->parameter_curve.value) : nullptr;
  const auto* pc = pg ? std::get_if<openparasolid::NurbsCurveGeometry>(&pg->payload) : nullptr;
  if (!model || !pc) return c;
  std::vector<double> knots = expanded(pc->axis);
  std::array<double, 2> range;
  if (trim) {
    range = {std::min((*trim)[0], (*trim)[1]), std::max((*trim)[0], (*trim)[1])};
  } else {
    size_t p = pc->axis.degree;
    if (knots.size() < p + 2) return c;
    range = {knots[p], knots[knots.size() - p - 1]};
  }
  c.domain = range;
  if (trim) c.range = *trim;
  openparasolid::NodeId curve_id{index};
  openparasolid::NodeId pcurve_id = sp->parameter_curve;
  // Outside its knot domain the pcurve continues its end span, as cadmpeg's
  // B-spline evaluation does (clamped span, unclamped parameter).
  std::array<double, 2> pdomain{0, 0};
  if (size_t p = pc->axis.degree; knots.size() >= p + 2)
    pdomain = {knots[p], knots[knots.size() - p - 1]};
  auto extrapolated = [&](double t) -> std::optional<Vec3> {
    size_t p = pc->axis.degree, dim = pc->controls.dimension;
    size_t n = dim ? pc->controls.values.size() / dim : 0;
    if (dim < 2 || n <= p || knots.size() < n + p + 1) return std::nullopt;
    size_t span = t <= knots[p] ? p : n - 1;
    std::vector<std::array<double, 3>> d(p + 1);
    for (size_t j = 0; j <= p; ++j) {
      const double* c = &pc->controls.values[(span - p + j) * dim];
      d[j] = {c[0], c[1], pc->controls.rational && dim >= 3 ? c[2] : 1.0};
    }
    for (size_t r = 1; r <= p; ++r)
      for (size_t j = p; j >= r; --j) {
        size_t i = span - p + j;
        double den = knots[i + p - r + 1] - knots[i];
        double a = den != 0 ? (t - knots[i]) / den : 0.0;
        for (int k = 0; k < 3; ++k) d[j][k] = (1 - a) * d[j - 1][k] + a * d[j][k];
      }
    if (!(d[p][2] != 0)) return std::nullopt;
    auto e = openparasolid::EvaluateSurface(*model, sp->surface, d[p][0] / d[p][2], d[p][1] / d[p][2]);
    if (!e.ok()) return std::nullopt;
    const auto& q = e.value().position;
    return Vec3{q.x * kMm, q.y * kMm, q.z * kMm};
  };
  auto point = [&](double t) -> std::optional<Vec3> {
    if (t < pdomain[0] || t > pdomain[1]) return extrapolated(t);
    auto e = openparasolid::EvaluateCurve(*model, curve_id, t);
    if (!e.ok()) return std::nullopt;
    const auto& p = e.value().position;
    return Vec3{p.x * kMm, p.y * kMm, p.z * kMm};
  };
  // The knot axes a span may cross at most once: a B-surface support's two
  // axes, or a swept/spun support's B-curve profile on u.
  std::vector<double> u_knots, v_knots;
  const auto* sg = s_.geometry(sp->surface.value);
  if (sg) {
    if (const auto* ns = std::get_if<openparasolid::NurbsSurfaceGeometry>(&sg->payload)) {
      u_knots = expanded(ns->u);
      v_knots = expanded(ns->v);
    } else {
      uint32_t profile = 0;
      if (const auto* sw = std::get_if<openparasolid::SweptSurfaceGeometry>(&sg->payload))
        profile = sw->section.value;
      else if (const auto* sn = std::get_if<openparasolid::SpunSurfaceGeometry>(&sg->payload))
        profile = sn->profile.value;
      if (const auto* pf = profile ? s_.geometry(profile) : nullptr)
        if (const auto* nc = std::get_if<openparasolid::NurbsCurveGeometry>(&pf->payload))
          u_knots = expanded(nc->axis);
    }
  }
  auto resolved = [&](double low, double high) {
    if (u_knots.empty() && v_knots.empty()) return true;
    std::array<size_t, 3> pu{}, pv{};
    double ts[3] = {low, 0.5 * (low + high), high};
    for (int i = 0; i < 3; ++i) {
      auto e = openparasolid::EvaluateCurve(*model, pcurve_id, ts[i]);
      if (!e.ok()) return true;
      pu[i] = distinct_knots_at_or_below(u_knots, e.value().position.x);
      pv[i] = distinct_knots_at_or_below(v_knots, e.value().position.y);
    }
    return within_two(pu) && within_two(pv);
  };
  // A bounded curve's stored ends lie on the support (cadmpeg
  // `surface_curve_at`, SURFACE_POINT_TOLERANCE_MM); else it is no carrier.
  if (trim) {
    const DecodedNode* tn = s_.node(trim_index);
    if (!tn) return std::nullopt;
    std::vector<double> stored[2] = {s_.reals(*tn, "point_1"), s_.reals(*tn, "point_2")};
    std::array<Vec3, 2> mm;
    for (int i = 0; i < 2; ++i) {
      if (stored[i].size() != 3) return std::nullopt;
      mm[i] = Vec3{stored[i][0] * kMm, stored[i][1] * kMm, stored[i][2] * kMm};
      auto on = point((*trim)[i]);
      if (!on) return std::nullopt;
      double dx = on->x - mm[i].x, dy = on->y - mm[i].y, dz = on->z - mm[i].z;
      double d = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (!(d <= brep::kFitToleranceMm)) return std::nullopt;
    }
    if ((*trim)[0] > (*trim)[1]) std::swap(mm[0], mm[1]);
    c.surface_ends = mm;
  } else {
    auto a = point(range[0]);
    auto b = point(range[1]);
    if (!a || !b) return std::nullopt;
    c.surface_ends = std::array<Vec3, 2>{*a, *b};
  }
  auto fitted = brep::fit_curve(point, range, knots, resolved);
  if (!fitted) return c;
  c.g.kind = "nurbs";
  c.typed = true;
  c.g.nurbs = std::move(*fitted);
  c.ends = std::array<Vec3, 2>{c.g.nurbs->poles.front(), c.g.nurbs->poles.back()};
  set_spline_domain(c);
  if (trim) c.range = *trim;
  return c;
}

std::optional<CurveCarrier> Decoder::analytic_curve(uint32_t index) {
  const DecodedNode* n = s_.node(index);
  if (!n) return std::nullopt;
  std::vector<double> v = s_.values_after_sense(*n);
  if (!finite(v)) return std::nullopt;
  CurveCarrier c;
  int64_t sense = s_.integer(*n, "sense");
  c.marker_reversed = sense == kSenseMinus;
  if (n->node_type == xt::kLine) {
    if (v.size() < 6 || !unit(v3(&v[3]))) return std::nullopt;
    c.g.kind = "line";
    c.g.origin = scale(v3(&v[0]), kMm);
    c.g.direction = v3(&v[3]);
  } else if (n->node_type == xt::kCircle) {
    if (v.size() < 10 || !frame_ok(v3(&v[3]), v3(&v[6])) || !(v[9] > 0)) return std::nullopt;
    c.g.kind = "circle";
    c.g.center = scale(v3(&v[0]), kMm);
    c.g.axis = v3(&v[3]);
    c.g.ref_direction = v3(&v[6]);
    c.g.radius = v[9] * kMm;
    c.closed_domain = true;
  } else if (n->node_type == xt::kEllipse) {
    if (v.size() < 11 || !frame_ok(v3(&v[3]), v3(&v[6])) || !(v[9] >= v[10] && v[10] > 0))
      return std::nullopt;
    c.g.kind = "ellipse";
    c.g.center = scale(v3(&v[0]), kMm);
    c.g.axis = v3(&v[3]);
    c.g.ref_direction = v3(&v[6]);
    c.g.radius = v[9] * kMm;
    c.g.minor_radius = v[10] * kMm;
    c.closed_domain = true;
  } else {
    return std::nullopt;
  }
  c.typed = true;
  return c;
}

std::optional<CurveCarrier> Decoder::curve(uint32_t index, int depth) {
  if (index == 0) return std::nullopt;
  if (depth == 0) {
    auto it = curve_cache_.find(index);
    if (it != curve_cache_.end()) return it->second;
  }
  std::optional<CurveCarrier> out;
  switch (s_.type(index)) {
    case xt::kLine:
    case xt::kCircle:
    case xt::kEllipse:
      out = analytic_curve(index);
      break;
    case xt::kTrimmedCurve: {
      if (depth > 4) break;
      const DecodedNode* n = s_.node(index);
      if (n && s_.is(ptr(index, "basis_curve"), xt::kSpCurve)) {
        // A bounded curve on a surface: fitted over its trim interval.
        auto p1 = s_.real(*n, "parm_1");
        auto p2 = s_.real(*n, "parm_2");
        if (p1 && p2) out = surface_curve(ptr(index, "basis_curve"), std::array<double, 2>{*p1, *p2}, index);
        break;
      }
      auto basis = curve(ptr(index, "basis_curve"), depth + 1);
      if (basis && n && !basis->chart.empty()) {
        // A bounded intersection: its stored ends on the chart (cadmpeg
        // `IntersectionCarrier::bounded`); else no carrier.
        auto p1 = s_.real(*n, "parm_1");
        auto p2 = s_.real(*n, "parm_2");
        std::vector<double> q1 = s_.reals(*n, "point_1"), q2 = s_.reals(*n, "point_2");
        if (p1 && p2 && q1.size() == 3 && q2.size() == 3)
          out = bounded_chart(*basis, {*p1, *p2},
                              {Vec3{q1[0] * kMm, q1[1] * kMm, q1[2] * kMm},
                               Vec3{q2[0] * kMm, q2[1] * kMm, q2[2] * kMm}});
        break;
      }
      if (basis && basis->typed && n) {
        auto p1 = s_.real(*n, "parm_1");
        auto p2 = s_.real(*n, "parm_2");
        if (p1 && p2 && std::isfinite(*p1) && std::isfinite(*p2)) basis->range = {*p1, *p2};
        out = basis;
      }
      break;
    }
    case xt::kBCurve: {
      CurveCarrier c;
      c.g.kind = "nurbs";
      c.typed = true;
      if (const auto* g = s_.geometry(index))
        if (const auto* nc = std::get_if<openparasolid::NurbsCurveGeometry>(&g->payload)) {
          c.g.nurbs = nurbs_curve(*nc);
          if (c.g.nurbs && !c.g.nurbs->poles.empty() && !nc->axis.periodic) {
            c.ends = std::array<Vec3, 2>{c.g.nurbs->poles.front(), c.g.nurbs->poles.back()};
            set_spline_domain(c);
          }
        }
      out = c;
      break;
    }
    case xt::kSpCurve:
      out = surface_curve(index, std::nullopt);
      break;
    case xt::kIntersection: {
      CurveCarrier c;
      c.g.kind = "nurbs";
      c.typed = true;
      if (const auto* g = s_.geometry(index))
        if (const auto* ic = std::get_if<openparasolid::IntersectionCurveGeometry>(&g->payload))
          intersection_chart(*ic, c);
      out = c;
      break;
    }
    default:
      break;
  }
  if (depth == 0) curve_cache_[index] = out;
  return out;
}

std::optional<SurfaceCarrier> Decoder::surface(uint32_t index) {
  const DecodedNode* n = s_.node(index);
  if (!n) return std::nullopt;
  SurfaceCarrier c;
  uint16_t t = n->node_type;
  if (t == xt::kPlane || t == xt::kCylinder || t == xt::kCone || t == xt::kSphere ||
      t == xt::kTorus) {
    std::vector<double> v = s_.values_after_sense(*n);
    if (!finite(v)) return std::nullopt;
    c.analytic = true;
    c.marker_reversed = s_.integer(*n, "sense") == kSenseMinus;
    auto& g = c.g;
    if (t == xt::kPlane) {
      if (v.size() < 9 || !frame_ok(v3(&v[3]), v3(&v[6]))) return std::nullopt;
      g.kind = "plane";
      g.origin = scale(v3(&v[0]), kMm);
      g.normal = v3(&v[3]);
      g.u_axis = v3(&v[6]);
    } else if (t == xt::kCylinder) {
      if (v.size() < 10 || !frame_ok(v3(&v[3]), v3(&v[7])) || !(v[6] > 0)) return std::nullopt;
      g.kind = "cylinder";
      g.origin = scale(v3(&v[0]), kMm);
      g.normal = v3(&v[3]);
      g.radius = v[6] * kMm;
      g.u_axis = v3(&v[7]);
    } else if (t == xt::kCone) {
      if (v.size() < 12 || !frame_ok(v3(&v[3]), v3(&v[9])) || !(v[6] >= 0)) return std::nullopt;
      double sine = v[7], cosine = v[8];
      auto unit_mag = [](double x) { return x != 0.0 && std::abs(x) < 1.0; };
      if (!unit_mag(sine) || !unit_mag(cosine) ||
          std::abs(sine * sine + cosine * cosine - 1.0) > 1.0e-9 || !(sine > 0 && cosine > 0))
        return std::nullopt;
      g.kind = "cone";
      g.origin = scale(v3(&v[0]), kMm);
      g.normal = v3(&v[3]);
      g.radius = v[6] * kMm;
      g.half_angle = std::atan2(sine, cosine);
      g.u_axis = v3(&v[9]);
    } else if (t == xt::kSphere) {
      if (v.size() < 10 || !frame_ok(v3(&v[4]), v3(&v[7])) || !(v[3] > 0)) return std::nullopt;
      g.kind = "sphere";
      g.origin = scale(v3(&v[0]), kMm);
      g.radius = v[3] * kMm;
      g.normal = v3(&v[4]);
      g.u_axis = v3(&v[7]);
      c.loopless = true;
    } else {
      if (v.size() < 11 || !frame_ok(v3(&v[3]), v3(&v[8])) ||
          !(std::abs(v[6]) > std::numeric_limits<double>::epsilon()) || !(v[7] > 0))
        return std::nullopt;
      g.kind = "torus";
      g.origin = scale(v3(&v[0]), kMm);
      g.normal = v3(&v[3]);
      g.radius = std::abs(v[6]) * kMm;
      g.minor_radius = v[7] * kMm;
      g.u_axis = v3(&v[8]);
      c.orientation_reversed = std::signbit(v[6]);
      c.loopless = true;
    }
    return c;
  }
  switch (t) {
    case xt::kBSurface: {
      c.g.kind = "nurbs";
      if (const auto* g = s_.geometry(index))
        if (const auto* ns = std::get_if<openparasolid::NurbsSurfaceGeometry>(&g->payload)) {
          c.g.nurbs = nurbs_surface(*ns);
          c.loopless = ns->u.periodic && ns->v.periodic;
        }
      return c;
    }
    case xt::kSweptSurf:
    case xt::kSpunSurf:
      c.g.kind = "nurbs";
      return c;
    case xt::kOffsetSurf:
    case xt::kBlendedEdge:
    case xt::kBlendBound:
      c.g.kind = "procedural";
      return c;
    default:
      return std::nullopt;
  }
}

std::optional<Vec3> Decoder::point_position(uint32_t point) const {
  const DecodedNode* n = s_.node(point);
  if (!n || n->node_type != xt::kPoint) return std::nullopt;
  auto p = s_.reals(*n, "position");
  if (p.size() != 3 || !finite(p)) return std::nullopt;
  return Vec3{p[0] * kMm, p[1] * kMm, p[2] * kMm};
}

std::optional<Vec3> Decoder::curve_point(const CurveCarrier& c, double t) const {
  const auto& g = c.g;
  if (g.kind == "line") return add(g.origin, scale(g.direction, t));
  if (g.kind == "circle" || g.kind == "ellipse") {
    Vec3 y = cross(g.axis, g.ref_direction);
    double minor = g.kind == "circle" ? g.radius : g.minor_radius;
    return add(g.center,
               add(scale(g.ref_direction, g.radius * std::cos(t)), scale(y, minor * std::sin(t))));
  }
  return std::nullopt;
}

/// cadmpeg `carrier_angle`.
std::optional<double> carrier_angle(const CurveGeometry& g, Vec3 point, double allowance) {
  if (g.kind != "circle" && g.kind != "ellipse") return std::nullopt;
  double major = g.radius;
  double minor = g.kind == "circle" ? g.radius : g.minor_radius;
  Vec3 binormal = cross(g.axis, g.ref_direction);
  Vec3 offset = sub(point, g.center);
  double x = dot(offset, g.ref_direction) / major;
  double y = dot(offset, binormal) / minor;
  double radius = std::hypot(x, y);
  if (std::isnan(radius) || radius <= 0.0) return std::nullopt;
  Vec3 nearest = add(g.center, add(scale(g.ref_direction, major * x / radius),
                                   scale(binormal, minor * y / radius)));
  if (!(dist(point, nearest) <= allowance)) return std::nullopt;
  double a = std::atan2(y, x);
  return periodic_phase({a, a})[0];
}

/// cadmpeg `conic_edge_range`.
std::optional<std::pair<std::array<double, 2>, bool>> conic_edge_range(
    const CurveGeometry& g, bool marker_reversed, std::array<Vec3, 2> ends,
    std::array<double, 2> allowances, bool closed) {
  auto sa = carrier_angle(g, ends[0], allowances[0]);
  auto ea = carrier_angle(g, ends[1], allowances[1]);
  if (!sa || !ea) return std::nullopt;
  if (closed) return std::make_pair(std::array<double, 2>{*sa, *sa + kTau}, marker_reversed);
  if (dist(ends[0], ends[1]) <= kLinearResolution) return std::nullopt;
  if (marker_reversed)
    return std::make_pair(std::array<double, 2>{*ea, *ea + rem_euclid(*sa - *ea, kTau)}, true);
  return std::make_pair(std::array<double, 2>{*sa, *sa + rem_euclid(*ea - *sa, kTau)}, false);
}

// ---------------------------------------------------------------------------
// Topology walk
// ---------------------------------------------------------------------------
struct WalkedFace {
  uint32_t face = 0;  ///< raw index
  uint32_t surface = 0;
  bool reversed = false;  ///< FACE.sense '-'
  std::vector<std::pair<uint32_t, std::vector<uint32_t>>> loops;
  std::vector<std::pair<uint32_t, uint32_t>> vertex_loops;
};

struct Incidence {
  uint32_t fin, own_vuse, ring_vuse;
};

struct EdgeEnds {
  uint32_t start, end, curve, fin;
};

}  // namespace

SiteBrep Decoder::run() {
  SiteBrep out;
  // Raw index -> attr (cadmpeg entity ids are index + 1).
  auto attr = [](uint32_t index) { return index + 1; };

  std::vector<uint32_t> face_nodes, fin_nodes, body_nodes;
  for (const DecodedNode* node_ptr : s_.nodes()) if (const DecodedNode& n = *node_ptr; true) {
    switch (n.node_type) {
      case xt::kFace: face_nodes.push_back(n.index); break;
      case xt::kFin: fin_nodes.push_back(n.index); break;
      case xt::kBody: body_nodes.push_back(n.index); break;
      default: break;
    }
    if (n.node_type >= xt::kBody && n.node_type <= xt::kRegion) ++out.entity_records;
  }
  std::sort(face_nodes.begin(), face_nodes.end());

  auto fin_loop = [&](uint32_t f) { return ptr(f, "loop"); };
  auto fin_forward = [&](uint32_t f) { return ptr(f, "forward"); };
  auto fin_backward = [&](uint32_t f) { return ptr(f, "backward"); };
  auto fin_vertex = [&](uint32_t f) { return ptr(f, "vertex"); };
  auto fin_other = [&](uint32_t f) { return ptr(f, "other"); };
  auto fin_edge = [&](uint32_t f) { return ptr(f, "edge"); };
  auto fin_curve = [&](uint32_t f) { return ptr(f, "curve"); };
  auto fin_forward_sense = [&](uint32_t f) { return field(f, "sense") != kSenseMinus; };
  auto is_fin = [&](uint32_t f) { return s_.is(f, xt::kFin); };

  // --- Walk every face (cadmpeg `walk_face`).
  std::vector<WalkedFace> faces;
  faces.reserve(face_nodes.size());
  for (uint32_t face : face_nodes) {
    WalkedFace w;
    w.face = face;
    w.surface = ptr(face, "surface");
    w.reversed = field(face, "sense") == kSenseMinus;
    std::unordered_set<uint32_t> loop_guard;
    for (uint32_t lp = ptr(face, "loop"); lp != 0; lp = ptr(lp, "next")) {
      if (!loop_guard.insert(lp).second || !s_.is(lp, xt::kLoop)) break;
      if (ptr(lp, "face") != face) break;
      uint32_t first = ptr(lp, "halfedge");
      if (is_fin(first) && fin_edge(first) == 0 && fin_loop(first) == lp &&
          (fin_forward(first) == first || fin_forward(first) == 0)) {
        w.vertex_loops.emplace_back(lp, fin_vertex(first));
        continue;
      }
      std::vector<uint32_t> ring;
      std::unordered_set<uint32_t> guard;
      bool closed = false;
      for (uint32_t ce = first; ce != 0;) {
        if (!guard.insert(ce).second || !is_fin(ce) || fin_loop(ce) != lp) break;
        ring.push_back(ce);
        ce = fin_forward(ce);
        if (ce == first) {
          closed = true;
          break;
        }
      }
      bool reciprocal = closed && std::all_of(ring.begin(), ring.end(), [&](uint32_t ce) {
                          uint32_t fwd = fin_forward(ce);
                          return is_fin(fwd) && fin_loop(fwd) == lp && fin_backward(fwd) == ce;
                        });
      if (closed && reciprocal) w.loops.emplace_back(lp, std::move(ring));
    }
    faces.push_back(std::move(w));
  }

  // --- Edge incidences.
  std::map<uint32_t, std::vector<Incidence>> incidence;  // edge -> fins
  for (const auto& f : faces)
    for (const auto& [lp, ring] : f.loops) {
      size_t k = ring.size();
      for (size_t i = 0; i < k; ++i) {
        uint32_t ce = ring[i];
        uint32_t prev = ring[(i + k - 1) % k];
        uint32_t e = fin_edge(ce);
        if (e != 0) incidence[e].push_back({ce, fin_vertex(ce), fin_vertex(prev)});
      }
    }
  std::unordered_map<uint32_t, std::vector<uint32_t>> fins_by_edge;
  for (uint32_t f : fin_nodes)
    if (uint32_t e = fin_edge(f)) fins_by_edge[e].push_back(f);

  std::unordered_set<uint32_t> kept_vertices, kept_points;
  auto keep_vertex = [&](uint32_t v) {
    if (v == 0 || !s_.is(v, xt::kVertex)) return;
    uint32_t p = ptr(v, "point");
    if (point_position(p)) {
      kept_vertices.insert(v);
      kept_points.insert(p);
    }
  };
  auto is_dummy = [&](uint32_t f) {
    return fin_loop(f) == 0 && fin_forward(f) == 0 && fin_backward(f) == 0;
  };

  std::map<uint32_t, EdgeEnds> edge_ends;
  for (const auto& [e, incs] : incidence) {
    // Canonical coedge (cadmpeg `canonical_coedge_attr`).
    uint32_t canonical = 0;
    uint32_t explicit_fin = ptr(e, "halfedge");
    if (explicit_fin != 0) {
      if (!is_fin(explicit_fin) || fin_edge(explicit_fin) != e || !fin_forward_sense(explicit_fin))
        continue;
      canonical = explicit_fin;
    } else {
      uint32_t found = 0, count = 0;
      for (uint32_t f : fins_by_edge[e])
        if (fin_forward_sense(f)) found = f, ++count;
      if (count != 1) continue;
      canonical = found;
    }
    uint32_t curve_index = ptr(e, "curve");
    if (curve_index == 0) {
      // Tolerant edge: the fin curve (cadmpeg `tolerant_fin_curve`).
      std::vector<uint32_t> candidates;
      for (uint32_t f : {canonical, fin_other(canonical)}) {
        if (!is_fin(f) || fin_edge(f) != e) continue;
        if (uint32_t c = fin_curve(f)) candidates.push_back(c);
      }
      // A curve stated on a non-swept support first, then any that decodes.
      for (uint32_t c : candidates)
        if (auto cc = curve(c); cc && !cc->on_sweep) {
          curve_index = c;
          break;
        }
      if (curve_index == 0)
        for (uint32_t c : candidates)
          if (curve(c)) {
            curve_index = c;
            break;
          }
      if (curve_index == 0 && !candidates.empty()) curve_index = candidates.front();
    }
    auto it = std::find_if(incs.begin(), incs.end(),
                           [&](const Incidence& i) { return i.fin == canonical; });
    if (it == incs.end()) {
      // Sheet-boundary dummy partner (cadmpeg `recover_dummy_partner_edge`).
      if (!is_dummy(canonical)) continue;
      uint32_t partner = fin_other(canonical);
      if (partner == 0 || partner == canonical || !is_fin(partner) ||
          fin_other(partner) != canonical || fin_edge(partner) != e)
        continue;
      auto pi = std::find_if(incs.begin(), incs.end(),
                             [&](const Incidence& i) { return i.fin == partner; });
      if (pi == incs.end()) continue;
      uint32_t fv = fin_vertex(canonical), pv = fin_vertex(partner);
      bool vertexless = fv == 0 && pv == 0 && pi->ring_vuse == 0;
      if (!vertexless && (fv == 0 || pv == 0 || fv != pi->ring_vuse)) continue;
      edge_ends[e] = {pv, fv, curve_index, partner};
      keep_vertex(pv);
      keep_vertex(fv);
      continue;
    }
    // Twin corroboration (cadmpeg `edge_ring_vuse`).
    uint32_t ring_vuse = it->ring_vuse;
    uint32_t twin = fin_other(canonical);
    if (twin != canonical && twin != 0 && is_fin(twin) && fin_other(twin) == canonical) {
      uint32_t tv = fin_vertex(twin);
      if (!(tv == ring_vuse || (tv == 0 && ring_vuse == 0))) continue;
    }
    edge_ends[e] = {ring_vuse, it->own_vuse, curve_index, canonical};
    keep_vertex(ring_vuse);
    keep_vertex(it->own_vuse);
  }
  for (const auto& f : faces)
    for (const auto& [lp, v] : f.vertex_loops) keep_vertex(v);

  // --- Points and vertices.
  std::map<std::string, Vec3> point_pos;  // point id -> position
  for (uint32_t p : kept_points) {
    Vec3 pos = *point_position(p);
    out.points.push_back({ident("point", attr(p)), pos});
  }
  std::map<std::string, double> vertex_bound;  // vertex id -> tolerance bound
  auto vertex_tolerance = [&](uint32_t v) -> std::optional<double> {
    const DecodedNode* n = s_.node(v);
    if (!n) return std::nullopt;
    auto t = s_.real(*n, "tolerance");
    if (t && std::isfinite(*t) && *t > 0) return *t * kMm;
    return std::nullopt;
  };
  for (uint32_t v : kept_vertices) {
    out.vertices.push_back({ident("vertex", attr(v)), ident("point", attr(ptr(v, "point")))});
  }

  // --- Curves and edges.
  std::set<uint32_t> emitted_curves;
  // Per emitted edge, what a blend boundary samples (cadmpeg
  // `edge_endpoint_positions`, `closed_edge_starts`, `edge_curve_spans`).
  struct BoundaryEdge {
    std::optional<std::array<Vec3, 2>> ends;
    std::optional<Vec3> closed_start;
    std::optional<CurveGeometry> curve;  ///< An indexed (non-SP) typed carrier.
    std::optional<std::array<double, 2>> range;
  };
  std::unordered_map<uint32_t, BoundaryEdge> boundary_edges;
  std::unordered_set<uint32_t> edge_set, reversed_edges;
  auto admit_vertexless = [&](uint32_t e, uint32_t fin, uint32_t curve_index, bool closes) {
    if (!is_fin(fin) || fin_edge(fin) != e || fin_vertex(fin) != 0) return false;
    uint32_t partner = fin_other(fin);
    if (partner == 0 || partner == fin || !is_fin(partner)) return false;
    if (fin_other(partner) != fin || fin_edge(partner) != e || fin_vertex(partner) != 0 ||
        fin_forward_sense(partner) == fin_forward_sense(fin))
      return false;
    if (!closes || curve_index == 0) return false;
    bool single = false, partner_single = is_dummy(partner);
    for (const auto& f : faces)
      for (const auto& [lp, ring] : f.loops) {
        if (ring.size() == 1 && ring[0] == fin) single = s_.is(ptr(lp, "face"), xt::kFace);
        if (ring.size() == 1 && ring[0] == partner) partner_single = true;
      }
    if (!single || !partner_single) return false;
    size_t dummies = 0;
    for (uint32_t f : fins_by_edge[e]) dummies += is_dummy(f);
    return dummies <= 1;
  };

  for (const auto& [e, ends] : edge_ends) {
    uint32_t start_v = ends.start, end_v = ends.end;
    std::optional<CurveCarrier> carrier = curve(ends.curve);
    bool resolved = kept_vertices.count(start_v) && kept_vertices.count(end_v);
    bool vertexless = !resolved && start_v == 0 && end_v == 0;
    std::optional<std::pair<Vec3, std::array<double, 2>>> closed_point;
    if (vertexless && carrier && carrier->typed && carrier->closed_domain && !carrier->surface_ends) {
      // cadmpeg `closed_edge_start`.
      std::array<double, 2> domain = carrier->domain;
      std::array<double, 2> r = carrier->range.value_or(domain);
      if (r[0] > r[1]) std::swap(r[0], r[1]);
      bool full = true;
      if (carrier->range) {
        if (carrier->g.kind == "circle" || carrier->g.kind == "ellipse")
          full = r[1] > r[0] && r[1] - r[0] == domain[1] - domain[0];
        else
          full = r == domain;
      }
      if (full) {
        if (carrier->g.kind == "nurbs" && carrier->ends)
          closed_point = std::make_pair((*carrier->ends)[0], r);
        else if (auto p = curve_point(*carrier, r[0]))
          closed_point = std::make_pair(*p, r);
      }
    }
    if (vertexless && !closed_point && carrier && carrier->surface_ends) {
      // cadmpeg `surface_curve_ring_start`: stored ends that meet within
      // RING_CLOSURE_MM; the low end and the sorted range.
      const auto& [a, b] = *carrier->surface_ends;
      double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <= 1.0e-5) {
        std::array<double, 2> r = carrier->range.value_or(carrier->domain);
        if (r[0] > r[1]) std::swap(r[0], r[1]);
        closed_point = std::make_pair(a, r);
      }
    }
    if (vertexless && !admit_vertexless(e, ends.fin, ends.curve, closed_point.has_value()))
      continue;
    if (!resolved && !closed_point) continue;

    Edge edge;
    edge.id = ident("edge", attr(e));
    std::string start_id, end_id;
    std::optional<std::array<Vec3, 2>> endpoints;
    if (closed_point) {
      std::string pid = "sldprt:brep:point#closed-circle-" + std::to_string(attr(e));
      std::string vid = "sldprt:brep:vertex#closed-circle-" + std::to_string(attr(e));
      out.points.push_back({pid, closed_point->first});
      out.vertices.push_back({vid, pid});
      start_id = end_id = vid;
    } else {
      start_id = ident("vertex", attr(start_v));
      end_id = ident("vertex", attr(end_v));
      auto ps = point_position(ptr(start_v, "point"));
      auto pe = point_position(ptr(end_v, "point"));
      if (ps && pe) endpoints = std::array<Vec3, 2>{*ps, *pe};
    }
    std::array<std::optional<double>, 2> tolerances = {vertex_tolerance(start_v),
                                                       vertex_tolerance(end_v)};
    // Parameter range (cadmpeg `edge_parameter_range`).
    std::optional<std::pair<std::array<double, 2>, bool>> range;
    if (carrier && carrier->typed) {
      const auto& g = carrier->g;
      auto allowances = [&](double exact) {
        std::array<double, 2> a{};
        for (int i = 0; i < 2; ++i) a[i] = tolerances[i] ? looser(exact, *tolerances[i]) : exact;
        return a;
      };
      if (!carrier->range) {
        if (endpoints)
          range = conic_edge_range(g, carrier->marker_reversed.value_or(false), *endpoints,
                                   allowances(kLinearResolution), start_v == end_v);
      } else {
        std::array<double, 2> r = *carrier->range;
        if (g.kind == "line") r = {r[0] * kMm, r[1] * kMm};
        if (r[0] > r[1]) std::swap(r[0], r[1]);
        if (g.kind == "circle" || g.kind == "ellipse") r = periodic_phase(r);
        if (!endpoints) {
          if (g.kind == "line" || g.kind == "circle" || g.kind == "ellipse")
            range = std::make_pair(r, false);
        } else {
          auto first = curve_point(*carrier, r[0]);
          auto second = curve_point(*carrier, r[1]);
          if (first && second) {
            auto al = allowances(kStoredRangeTolerance);
            std::array<double, 2> fwd = {dist(*first, (*endpoints)[0]),
                                         dist(*second, (*endpoints)[1])};
            std::array<double, 2> bwd = {dist(*second, (*endpoints)[0]),
                                         dist(*first, (*endpoints)[1])};
            auto fits = [&](std::array<double, 2> d) { return d[0] <= al[0] && d[1] <= al[1]; };
            auto worse = [](std::array<double, 2> d) { return std::max(d[0], d[1]); };
            bool ff = fits(fwd), fb = fits(bwd);
            if (ff && fb && worse(bwd) < worse(fwd)) range = std::make_pair(r, true);
            else if (ff) range = std::make_pair(r, false);
            else if (fb) range = std::make_pair(r, true);
          }
        }
      }
    }
    std::optional<std::array<double, 2>> param_range;
    if (range) param_range = range->first;
    else if (closed_point) param_range = closed_point->second;
    // Direction against the curve.
    bool against = false;
    if (range) {
      against = range->second;
    } else if (carrier && carrier->typed) {
      std::optional<bool> domain_reversed;
      if (carrier->ends && endpoints && !carrier->closed_domain) {
        const auto& ce = *carrier->ends;
        double fwd = dist(ce[0], (*endpoints)[0]) + dist(ce[1], (*endpoints)[1]);
        double bwd = dist(ce[1], (*endpoints)[0]) + dist(ce[0], (*endpoints)[1]);
        double b0 = tolerances[0] ? looser(*tolerances[0], kLinearResolution) : kLinearResolution;
        double b1 = tolerances[1] ? looser(*tolerances[1], kLinearResolution) : kLinearResolution;
        bool fwd_hit = dist(ce[0], (*endpoints)[0]) <= b0 && dist(ce[1], (*endpoints)[1]) <= b1;
        bool bwd_hit = dist(ce[1], (*endpoints)[0]) <= b0 && dist(ce[0], (*endpoints)[1]) <= b1;
        if (fwd_hit != bwd_hit) domain_reversed = bwd_hit;
        else if (fwd_hit) domain_reversed = bwd < fwd;
      }
      against = domain_reversed.value_or(carrier->marker_reversed.value_or(false));
    }
    if (against) {
      std::swap(start_id, end_id);
      reversed_edges.insert(e);
    }
    if (ends.curve != 0) {
      if (emitted_curves.insert(ends.curve).second) {
        Curve c;
        c.id = ident("curve", attr(ends.curve));
        if (carrier && carrier->typed) c.geometry = carrier->g;
        else c.geometry.kind = "unknown";
        out.curves.push_back(std::move(c));
      }
      edge.curve = ident("curve", attr(ends.curve));
    }
    edge.param_range = param_range;
    edge.start = start_id;
    edge.end = end_id;
    {
      BoundaryEdge& b = boundary_edges[e];
      b.ends = endpoints;
      if (closed_point) b.closed_start = closed_point->first;
      if (carrier && carrier->typed && !carrier->surface_ends) b.curve = carrier->g;
      b.range = param_range;
    }
    out.edges.push_back(std::move(edge));
    edge_set.insert(e);
  }

  // --- Loops and coedges.
  std::unordered_set<uint32_t> kept_loops;
  for (const auto& f : faces)
    for (const auto& [lp, ring] : f.loops) {
      bool ok = !ring.empty() && std::all_of(ring.begin(), ring.end(), [&](uint32_t c) {
        return edge_set.count(fin_edge(c)) != 0;
      });
      if (ok) kept_loops.insert(lp);
    }
  for (const auto& f : faces) {
    for (const auto& [lp, ring] : f.loops) {
      if (!kept_loops.count(lp)) continue;
      Loop loop;
      loop.id = ident("loop", attr(lp));
      loop.face = ident("face", attr(f.face));
      for (uint32_t ce : ring) {
        Coedge c;
        c.id = ident("coedge", attr(ce));
        c.owner_loop = loop.id;
        c.edge = ident("edge", attr(fin_edge(ce)));
        c.reversed = !fin_forward_sense(ce);
        if (reversed_edges.count(fin_edge(ce))) c.reversed = !c.reversed;
        loop.coedges.push_back(c.id);
        out.coedges.push_back(std::move(c));
      }
      out.loops.push_back(std::move(loop));
    }
    for (const auto& [lp, v] : f.vertex_loops) {
      if (!kept_vertices.count(v)) continue;
      Loop loop;
      loop.id = ident("loop", attr(lp));
      loop.face = ident("face", attr(f.face));
      out.loops.push_back(std::move(loop));
      kept_loops.insert(lp);
    }
  }

  // --- Body records (cadmpeg `typed_body_records`).
  struct ShellRec { uint32_t shell; std::vector<uint32_t> faces; };
  struct RegionRec { uint32_t region; bool solid; std::vector<ShellRec> shells; };
  struct BodyRec { uint32_t body; std::string kind; std::vector<RegionRec> regions; };
  std::vector<BodyRec> bodies;
  std::unordered_map<uint32_t, uint32_t> face_shell;
  bool hierarchy_ok = !body_nodes.empty();
  for (const auto& f : faces) {
    uint32_t sh = ptr(f.face, "shell");
    if (!s_.is(sh, xt::kShell)) hierarchy_ok = false;
    face_shell[f.face] = sh;
  }
  if (hierarchy_ok) {
    std::map<uint32_t, std::map<uint32_t, std::vector<uint32_t>>> body_shell_faces;
    std::map<uint32_t, std::set<uint32_t>> shell_region;
    for (const auto& f : faces) {
      uint32_t sh = face_shell[f.face];
      uint32_t region = ptr(sh, "region");
      uint32_t body = ptr(region, "body");
      if (!s_.is(region, xt::kRegion) || !s_.is(body, xt::kBody)) {
        hierarchy_ok = false;
        break;
      }
      uint32_t sb = ptr(sh, "body");
      if (sb != 0 && sb != body) {
        hierarchy_ok = false;
        break;
      }
      body_shell_faces[body][sh].push_back(f.face);
    }
    if (hierarchy_ok) {
      std::vector<uint32_t> relevant;
      if (faces.empty()) relevant = body_nodes;
      for (const auto& [b, _] : body_shell_faces) relevant.push_back(b);
      std::sort(relevant.begin(), relevant.end());
      relevant.erase(std::unique(relevant.begin(), relevant.end()), relevant.end());
      for (uint32_t b : relevant) {
        BodyRec rec;
        rec.body = b;
        switch (field(b, "body_type")) {
          case 2: rec.kind = "wire"; break;
          case 3: rec.kind = "sheet"; break;
          case 6: rec.kind = "general"; break;
          default: rec.kind = "solid"; break;
        }
        std::unordered_set<uint32_t> guard;
        bool has_material = false;
        for (uint32_t r = ptr(b, "region"); r != 0 && guard.insert(r).second; r = ptr(r, "next")) {
          if (!s_.is(r, xt::kRegion)) break;
          RegionRec rr;
          rr.region = r;
          rr.solid = field(r, "type") == 'S';
          auto& shells = body_shell_faces[b];
          for (auto& [sh, fs] : shells)
            if (ptr(sh, "region") == r) {
              ShellRec s{sh, fs};
              std::sort(s.faces.begin(), s.faces.end());
              rr.shells.push_back(std::move(s));
            }
          std::sort(rr.shells.begin(), rr.shells.end(),
                    [](const ShellRec& a, const ShellRec& b2) { return a.shell < b2.shell; });
          has_material |= rr.solid || !rr.shells.empty();
          rec.regions.push_back(std::move(rr));
        }
        std::erase_if(rec.regions, [&](const RegionRec& r) {
          return has_material && !r.solid && r.shells.empty();
        });
        std::sort(rec.regions.begin(), rec.regions.end(),
                  [](const RegionRec& a, const RegionRec& b2) { return a.region < b2.region; });
        bodies.push_back(std::move(rec));
      }
    }
  }
  std::unordered_map<uint32_t, uint32_t> claimed_shell;  // face -> shell (or group)
  if (hierarchy_ok) {
    for (const auto& b : bodies)
      for (const auto& r : b.regions)
        for (const auto& sh : r.shells)
          for (uint32_t f : sh.faces) claimed_shell[f] = sh.shell;
    std::erase_if(faces, [&](const WalkedFace& f) { return !claimed_shell.count(f.face); });
  }

  // --- Surfaces and faces.
  std::unordered_set<uint32_t> loop_owner_faces;
  for (const DecodedNode* node_ptr : s_.nodes()) if (const DecodedNode& n = *node_ptr; true)
    if (n.node_type == xt::kLoop) loop_owner_faces.insert(static_cast<uint32_t>(s_.integer(n, "face")));
  // Face colours (cadmpeg entity.rs `metadata` + graph.rs
  // `unique_face_colors`): a FACE's head ATTRIBUTE carries a colour when its
  // definition is SDL/TYSA_COLOUR (or is not a framed definition) and the
  // REAL_VALUES triple it references is framed in a run directly after an
  // ATTRIBUTE record linked to it.
  std::unordered_map<uint32_t, Color> head_colors;
  std::vector<std::pair<uint32_t, uint32_t>> unique_colors;  // (entity, colour record), raw
  {
    struct FaceColorRec {
      uint32_t attr, color_attr, seq;
      size_t order;
      Color color;
    };
    std::vector<FaceColorRec> colors;
    std::map<uint32_t, std::pair<uint32_t, size_t>> versions;  // attr -> current (seq, order)
    for (size_t part = 0; part < s_.parts(); ++part) {
      const auto& nodes = s_.part_nodes(part);
      // Attribute definition table of this stream (attrib.rs `Records`).
      std::unordered_map<uint32_t, std::string> keys;
      for (const auto& n : nodes)
        if (n.node_type == xt::kAttDefId) {
          std::string text = s_.chars(n, "string");
          if (!text.empty() && std::all_of(text.begin(), text.end(),
                                           [](char c) { return c > 0x20 && c < 0x7f; }))
            keys.emplace(n.index, std::move(text));
        }
      std::unordered_map<uint32_t, std::optional<std::string>> definitions;
      for (const auto& n : nodes) {
        if (n.node_type != xt::kAttribDef) continue;
        uint32_t identifier =
            static_cast<uint32_t>(std::max<int64_t>(0, s_.integer(n, "identifier")));
        auto fields = s_.integers(n, "fields");
        auto actions = s_.integers(n, "actions");
        bool owners = s_.field(n, "legal_owners") != nullptr;
        auto key = keys.find(identifier);
        if (identifier > 0 && owners &&
            std::all_of(fields.begin(), fields.end(), [](int64_t k) { return k <= 10; })) {
          definitions[n.index] =
              key == keys.end() ? std::nullopt : std::optional<std::string>(key->second);
        } else if (key != keys.end() && std::all_of(actions.begin(), actions.end(),
                                                    [](int64_t a) { return a <= 6; })) {
          definitions[n.index] = std::nullopt;
        }
      }
      auto color_at = [&](size_t i) -> std::optional<std::pair<uint32_t, Color>> {
        if (i >= nodes.size() || nodes[i].node_type != xt::kRealValues || nodes[i].index == 0)
          return std::nullopt;
        auto v = s_.reals(nodes[i], "values");
        if (v.size() != 3) return std::nullopt;
        for (double d : v)
          if (!(std::isfinite(d) && d >= 0.0 && d <= 1.0)) return std::nullopt;
        return std::make_pair(nodes[i].index,
                              Color{static_cast<float>(v[0]), static_cast<float>(v[1]),
                                    static_cast<float>(v[2]), 1.0f});
      };
      auto adjacent = [&](size_t i) {
        return i + 1 < nodes.size() && nodes[i].offset + nodes[i].size == nodes[i + 1].offset;
      };
      struct Entity {
        size_t pos;
        uint32_t attr, seq, def;
        std::vector<uint32_t> refs;
        size_t end;  ///< byte offset where a framed colour run would start
      };
      std::vector<Entity> entities;
      // Node position by byte offset (colour records found at an entity end).
      std::unordered_map<size_t, size_t> at_offset;
      for (size_t i = 0; i < nodes.size(); ++i) at_offset.emplace(nodes[i].offset, i);
      const bool deltas = s_.part_is_deltas(part);
      const ByteSpan bytes = s_.part_bytes(part);
      // entity.rs `scan_entities` on a deltas journal (`prefixed`): the
      // reference list is the leading `01 <word>` run, ended by a 00 byte;
      // the record's colour run would follow that byte.
      auto prefixed_refs = [&](const DecodedNode& n, std::vector<uint32_t>& refs, size_t& end) {
        size_t p = n.offset + 2;
        if (p < bytes.size() && bytes[p] == 0xff) ++p;
        auto pointer = [&](size_t& at) -> bool {
          if (at + 2 > bytes.size()) return false;
          auto first = static_cast<int16_t>((bytes[at] << 8) | bytes[at + 1]);
          at += 2;
          if (first >= 0) return true;
          if (at + 2 > bytes.size()) return false;
          auto quotient = static_cast<int16_t>((bytes[at] << 8) | bytes[at + 1]);
          at += 2;
          return quotient > 0;
        };
        size_t at = p + 4;  // flags
        if (!pointer(at)) return false;  // attr
        at += 4;                         // seq
        if (!pointer(at)) return false;  // disc
        size_t q = at;
        std::vector<uint32_t> out;
        while (q + 3 <= bytes.size() && bytes[q] == 1) {
          uint32_t word = (bytes[q + 1] << 8) | bytes[q + 2];
          out.push_back(word > 0 ? word - 1 : 0);
          q += 3;
        }
        if (!out.empty() && q < bytes.size() && bytes[q] == 0) {
          refs = std::move(out);
          end = q + 1;
        } else {
          refs.clear();
          end = at;
        }
        return true;
      };
      for (size_t i = 0; i < nodes.size(); ++i) {
        const auto& n = nodes[i];
        if (n.node_type != xt::kAttribute || n.index == 0) continue;
        int64_t seq = s_.integer(n, "node_id");
        auto fields = s_.integers(n, "fields");
        if (seq == 0 || fields.empty() || fields.size() > 32) continue;
        Entity e{i, n.index, static_cast<uint32_t>(seq),
                 static_cast<uint32_t>(std::max<int64_t>(0, s_.integer(n, "definition"))), {},
                 n.offset + n.size};
        if (deltas) {
          if (!prefixed_refs(n, e.refs, e.end)) continue;
        } else {
          for (const char* name : {"owner", "next", "previous", "next_of_type", "previous_of_type"})
            e.refs.push_back(static_cast<uint32_t>(std::max<int64_t>(0, s_.integer(n, name))));
          for (int64_t f : fields) e.refs.push_back(static_cast<uint32_t>(std::max<int64_t>(0, f)));
        }
        entities.push_back(std::move(e));
      }
      struct Framed {
        Color color;
        uint32_t parent_seq;
      };
      std::map<std::pair<uint32_t, uint32_t>, std::vector<Framed>> linked;
      for (const auto& parent : entities) {
        std::set<uint32_t> faces_linked(parent.refs.begin(), parent.refs.end());
        faces_linked.insert(parent.attr);
        auto first = at_offset.find(parent.end);
        if (first == at_offset.end()) continue;
        for (size_t at = first->second;; ++at) {
          auto c = color_at(at);
          if (!c) break;
          for (uint32_t f : faces_linked)
            if (f > 0) linked[{f, c->first}].push_back({c->second, parent.seq});
          if (!adjacent(at)) break;
        }
      }
      for (const auto& e : entities) {
        auto d = definitions.find(e.def);
        bool named =
            d != definitions.end() && d->second == std::optional<std::string>("SDL/TYSA_COLOUR");
        uint32_t linked_attr = e.refs.size() > 5 ? e.refs[5] : 0;
        std::optional<std::pair<uint32_t, Color>> direct;
        if (auto it = at_offset.find(e.end); it != at_offset.end()) direct = color_at(it->second);
        bool unnamed = d == definitions.end() &&
                       ((linked_attr > 0 && linked.count({e.attr, linked_attr})) || direct);
        if (!named && !unnamed) continue;
        auto version = std::make_pair(e.seq, part);
        auto [vit, fresh] = versions.emplace(e.attr, version);
        if (!fresh) vit->second = std::max(vit->second, version);
        if (linked_attr > 0) {
          auto it = linked.find({e.attr, linked_attr});
          if (it == linked.end()) continue;
          uint32_t current = 0;
          for (const auto& c : it->second) current = std::max(current, c.parent_seq);
          std::optional<Color> pick;
          bool agree = true;
          for (const auto& c : it->second) {
            if (c.parent_seq != current) continue;
            if (!pick) pick = c.color;
            else agree &= *pick == c.color;
          }
          if (pick && agree) colors.push_back({e.attr, linked_attr, e.seq, part, *pick});
        } else if (direct) {
          colors.push_back({e.attr, direct->first, e.seq, part, direct->second});
        }
      }
    }
    // graph.rs `unique_face_colors`.
    std::map<uint32_t, std::vector<const FaceColorRec*>> by_face;
    for (const auto& c : colors)
      if (versions[c.attr] == std::make_pair(c.seq, c.order)) by_face[c.attr].push_back(&c);
    std::map<uint32_t, std::vector<const FaceColorRec*>> by_record;
    for (const auto& [attr, cands] : by_face) {
      bool same = std::all_of(cands.begin(), cands.end(), [&](const FaceColorRec* c) {
        return c->color_attr == cands[0]->color_attr && c->color == cands[0]->color;
      });
      if (same) by_record[cands[0]->color_attr].push_back(cands[0]);
    }
    for (const auto& [record, owners] : by_record) {
      bool same = std::all_of(owners.begin(), owners.end(),
                              [&](const FaceColorRec* o) { return o->color == owners[0]->color; });
      if (same)
        for (const FaceColorRec* o : owners) {
          head_colors.emplace(o->attr, o->color);
          unique_colors.emplace_back(o->attr, o->color_attr);
        }
    }
  }
  auto face_color = [&](uint32_t face) -> std::optional<Color> {
    uint32_t head = ptr(face, "attributes_features");
    auto it = head_colors.find(head);
    if (it == head_colors.end()) return std::nullopt;
    return it->second;
  };
  // The solved cache of a rolling-ball blend face (cadmpeg graph.rs blend
  // emission: support resolution, `blend_boundary`, `blend_surface::solve`,
  // the `blend_fit::fit` fallback and the vertex-miss admission).
  struct RawBlend {
    int64_t type = 0;
    std::array<uint32_t, 2> refs{};
    uint32_t spine = 0;
    std::array<double, 2> offsets{};
  };
  auto raw_blend = [&](uint32_t index) -> std::optional<RawBlend> {
    if (!s_.is(index, xt::kBlendedEdge)) return std::nullopt;
    const DecodedNode* n = s_.node(index);
    if (!n) return std::nullopt;
    int64_t sense = s_.integer(*n, "sense");
    if (sense != 0x2b && sense != 0x2d) return std::nullopt;
    RawBlend b;
    b.type = s_.integer(*n, "blend_type");
    if (b.type != 0x45 && b.type != 0x52) return std::nullopt;
    auto refs = s_.integers(*n, "surface");
    auto range = s_.reals(*n, "range");
    auto thumb = s_.reals(*n, "thumb_weight");
    if (refs.size() != 2 || range.size() != 2 || thumb.size() != 2) return std::nullopt;
    for (double v : {range[0], range[1], thumb[0], thumb[1]})
      if (!std::isfinite(v)) return std::nullopt;
    if (std::fabs(std::fabs(thumb[0]) - 1.0) > 1.0e-12 ||
        std::fabs(std::fabs(thumb[1]) - 1.0) > 1.0e-12)
      return std::nullopt;
    b.spine = ptr(index, "spine");
    if (refs[0] <= 0 || refs[1] <= 0 || b.spine == 0) return std::nullopt;
    b.refs = {static_cast<uint32_t>(refs[0]), static_cast<uint32_t>(refs[1])};
    b.offsets = {range[0], range[1]};
    return b;
  };
  // cadmpeg `blend_from_raw`: a nonzero constant radius.
  auto blend_carrier = [&](uint32_t index) -> std::optional<brep::BlendCarriers::Blend> {
    auto b = raw_blend(index);
    if (!b) return std::nullopt;
    if (std::fabs(b->offsets[0]) <= kF64Eps ||
        std::fabs(std::fabs(b->offsets[0]) - std::fabs(b->offsets[1])) > 1.0e-12)
      return std::nullopt;
    return brep::BlendCarriers::Blend{b->refs, b->spine, b->offsets[0] * kMm};
  };
  // cadmpeg `pair_from_raw`: two surfaces at zero offsets about an intersection.
  auto support_pair = [&](uint32_t index) -> std::optional<RawBlend> {
    auto b = raw_blend(index);
    if (!b || b->type != 0x52 || std::fabs(b->offsets[0]) > kF64Eps ||
        std::fabs(b->offsets[1]) > kF64Eps)
      return std::nullopt;
    return b;
  };
  FnBlendCarriers carriers;
  carriers.surface_fn = [&](uint32_t index) -> std::optional<SurfaceGeometry> {
    auto sc = surface(index);
    if (!sc) return std::nullopt;
    if (sc->analytic || (s_.is(index, xt::kBSurface) && sc->g.nurbs)) return sc->g;
    return std::nullopt;
  };
  carriers.curve_fn = [&](uint32_t index) -> std::optional<CurveGeometry> {
    auto cc = curve(index);
    if (!cc || !cc->typed || cc->surface_ends) return std::nullopt;
    return cc->g;
  };
  carriers.blend_fn = blend_carrier;
  carriers.pair_fn = [&](uint32_t index) -> std::optional<std::array<uint32_t, 2>> {
    auto b = support_pair(index);
    if (!b) return std::nullopt;
    return b->refs;
  };
  carriers.bound_fn = [&](uint32_t index) -> std::optional<uint32_t> {
    if (!s_.is(index, xt::kBlendBound)) return std::nullopt;
    const DecodedNode* n = s_.node(index);
    if (!n) return std::nullopt;
    int64_t sense = s_.integer(*n, "sense");
    int64_t side = s_.integer(*n, "boundary");
    uint32_t blend = ptr(index, "blend");
    if ((sense != 0x2b && sense != 0x2d) || (side != 0 && side != 1) || blend == 0)
      return std::nullopt;
    return blend;
  };
  auto blend_cache = [&](const WalkedFace& f) -> std::optional<NurbsSurfaceGeometry> {
    auto blend = blend_carrier(f.surface);
    if (!blend) return std::nullopt;
    double radius = std::fabs(blend->signed_radius);
    const CurveGeometry* spine = carriers.curve(blend->spine);
    std::unordered_set<uint32_t> face_edges;
    for (const auto& [lp, ring] : f.loops)
      for (uint32_t fin : ring)
        if (uint32_t e = fin_edge(fin)) face_edges.insert(e);
    // cadmpeg `face_vertex_positions`.
    std::vector<Vec3> fin_vertices;
    for (const auto& [lp, ring] : f.loops)
      for (uint32_t fin : ring)
        if (uint32_t v = fin_vertex(fin))
          if (auto p = point_position(ptr(v, "point"))) fin_vertices.push_back(*p);
    // A support pair names the member the stored spine lies at the radius
    // from, else the one member that shares an edge with the face.
    std::array<uint32_t, 2> supports{};
    for (size_t slot = 0; slot < 2; ++slot) {
      uint32_t ref = blend->supports[slot];
      auto pair = support_pair(ref);
      if (!pair) {
        supports[slot] = ref;
        continue;
      }
      if (!carriers.curve(pair->spine)) return std::nullopt;
      std::optional<size_t> chosen;
      const SurfaceGeometry* a = carriers.surface(pair->refs[0]);
      const SurfaceGeometry* b = carriers.surface(pair->refs[1]);
      if (spine && a && b)
        chosen = brep::blend_pair_member(*spine, {a, b}, blend->signed_radius, fin_vertices);
      if (chosen) {
        supports[slot] = pair->refs[*chosen];
        continue;
      }
      std::vector<uint32_t> adjacent;
      for (uint32_t member : pair->refs) {
        bool touches = false;
        for (const auto& g : faces) {
          if (g.surface != member) continue;
          for (const auto& [lp, ring] : g.loops)
            for (uint32_t fin : ring) touches |= face_edges.count(fin_edge(fin)) != 0;
        }
        if (touches) adjacent.push_back(member);
      }
      if (adjacent.size() != 1) return std::nullopt;
      supports[slot] = adjacent[0];
    }
    // cadmpeg `blend_boundary`.
    std::vector<Vec3> vertices, boundary;
    for (const auto& [lp, ring] : f.loops)
      for (uint32_t fin : ring) {
        auto it = boundary_edges.find(fin_edge(fin));
        if (it == boundary_edges.end()) continue;
        const BoundaryEdge& b = it->second;
        if (b.ends) {
          vertices.insert(vertices.end(), b.ends->begin(), b.ends->end());
          boundary.insert(boundary.end(), b.ends->begin(), b.ends->end());
        }
        if (b.closed_start) {
          vertices.push_back(*b.closed_start);
          boundary.push_back(*b.closed_start);
        }
        if (b.curve) {
          auto samples = brep::blend_edge_samples(*b.curve, b.range);
          boundary.insert(boundary.end(), samples.begin(), samples.end());
        }
      }
    std::optional<brep::BlendCache> cache, off_vertex;
    const SurfaceGeometry* first = carriers.surface(supports[0]);
    const SurfaceGeometry* second = carriers.surface(supports[1]);
    if (spine && first && second) {
      brep::BlendInputs inputs;
      inputs.spine = spine;
      inputs.supports = {first, second};
      inputs.radius = radius;
      inputs.boundary = &boundary;
      inputs.vertices = &vertices;
      if (auto solved = brep::solve_blend(inputs)) {
        if (solved->holds) cache = std::move(solved->cache);
        else off_vertex = std::move(solved->cache);
      }
    }
    if (cache) return std::move(cache->surface);
    if (!spine) return std::nullopt;
    auto fitted = brep::fit_blend(carriers, *spine, blend->supports, supports,
                                  blend->signed_radius, vertices);
    if (!fitted) fitted = std::move(off_vertex);
    if (!fitted) return std::nullopt;
    auto miss = brep::blend_vertex_miss(*fitted, *spine, vertices);
    if (miss && *miss <= radius * 0.25) return std::move(fitted->surface);
    return std::nullopt;
  };
  std::unordered_set<std::string> spine_curves;
  std::unordered_set<uint32_t> face_carriers;
  std::vector<uint32_t> blend_faces;
  for (const auto& f : faces) {
    bool vertex_ok = std::all_of(f.vertex_loops.begin(), f.vertex_loops.end(),
                                 [&](const auto& vl) { return kept_loops.count(vl.first) != 0; });
    if (!vertex_ok) continue;
    auto sc = surface(f.surface);
    bool has_loop = std::any_of(f.loops.begin(), f.loops.end(),
                                [&](const auto& l) { return kept_loops.count(l.first) != 0; }) ||
                    std::any_of(f.vertex_loops.begin(), f.vertex_loops.end(),
                                [&](const auto& l) { return kept_loops.count(l.first) != 0; });
    bool whole = f.loops.empty() && f.vertex_loops.empty() && ptr(f.face, "loop") == 0 &&
                 sc && sc->loopless && !loop_owner_faces.count(f.face);
    if (!has_loop && !whole) continue;
    Face face;
    face.id = ident("face", attr(f.face));
    Surface surf;
    surf.id = ident("surf", attr(f.face));
    bool orientation = false;
    if (sc) {
      surf.geometry = sc->g;
      orientation = sc->orientation_reversed != sc->marker_reversed;
    } else {
      surf.geometry.kind = "unknown";
    }
    if (s_.is(f.surface, xt::kBlendedEdge))
      if (auto cache = blend_cache(f)) {
        surf.geometry = SurfaceGeometry{};
        surf.geometry.kind = "nurbs";
        surf.geometry.nurbs = std::move(*cache);
      }
    out.surfaces.push_back(std::move(surf));
    // A rolling-ball blend emits its spine curve (cadmpeg `blend_spine`).
    if (s_.is(f.surface, xt::kBlendedEdge))
      if (uint32_t spine = ptr(f.surface, "spine"))
        if (auto sp = curve(spine); sp && sp->typed) {
          spine_curves.insert(ident("curve", attr(spine)));
          if (emitted_curves.insert(spine).second) {
            Curve c;
            c.id = ident("curve", attr(spine));
            c.geometry = sp->g;
            out.curves.push_back(std::move(c));
          }
        }
    face.surface = ident("surf", attr(f.face));
    face.reversed = f.reversed != orientation;
    for (const auto& [lp, _] : f.loops)
      if (kept_loops.count(lp)) face.loops.push_back(ident("loop", attr(lp)));
    for (const auto& [lp, _] : f.vertex_loops)
      if (kept_loops.count(lp)) face.loops.push_back(ident("loop", attr(lp)));
    face.color = face_color(f.face);
    face.shell = ident("shell", attr(hierarchy_ok ? claimed_shell[f.face] : 0) - (hierarchy_ok ? 0 : 1));
    out.faces.push_back(std::move(face));
    face_carriers.insert(f.surface);
    if (s_.is(f.surface, xt::kBlendedEdge)) blend_faces.push_back(f.surface);
  }
  // Face colour targets: the first emitted walked face the entity heads.
  {
    std::unordered_set<std::string> emitted;
    for (const auto& f : out.faces) emitted.insert(f.id);
    for (const auto& [entity, record] : unique_colors) {
      BrepFaceColor fc;
      fc.face_attr = attr(entity);
      fc.color_attr = attr(record);
      fc.color = head_colors[entity];
      for (const auto& f : faces)
        if (ptr(f.face, "attributes_features") == entity) {
          std::string id = ident("face", attr(f.face));
          if (emitted.count(id)) fc.target = id;
          break;
        }
      out.face_colors.push_back(std::move(fc));
    }
  }
  // Blend supports no face carries (cadmpeg `ensure_surface_support`):
  // hidden support surfaces, opaque for constructions.
  {
    std::set<uint32_t> hidden;
    for (uint32_t blend : blend_faces) {
      const DecodedNode* n = s_.node(blend);
      if (!n) continue;
      std::vector<int64_t> supports = s_.integers(*n, "surface");
      if (supports.size() != 2) continue;
      for (int64_t raw : supports) {
        uint32_t support = raw > 0 ? static_cast<uint32_t>(raw) : 0;
        if (support == 0 || face_carriers.count(support) || !hidden.insert(support).second) continue;
        auto sc = surface(support);
        if (!sc) continue;
        // A zero-radius blend is no support carrier.
        if (s_.is(support, xt::kBlendedEdge))
          if (const DecodedNode* bn = s_.node(support)) {
            std::vector<double> r = s_.reals(*bn, "range");
            if (r.empty() || std::all_of(r.begin(), r.end(), [](double v) { return v == 0.0; })) continue;
          }
        Surface surf;
        surf.id = ident("hidden-support-surf", attr(support));
        if (sc->analytic || sc->g.kind == "nurbs") surf.geometry = sc->g;
        else surf.geometry.kind = "unknown";
        out.surfaces.push_back(std::move(surf));
      }
    }
  }

  // --- Native attributes (cadmpeg attrib.rs).
  {
    struct Def { std::string name; std::vector<int64_t> fields; };
    std::unordered_map<uint32_t, std::optional<Def>> defs;
    std::unordered_map<uint32_t, std::vector<uint32_t>> lists;
    for (const DecodedNode* node_ptr : s_.nodes()) if (const DecodedNode& n = *node_ptr; true) {
      if (n.node_type == xt::kAttribDef) {
        uint32_t identifier = static_cast<uint32_t>(s_.integer(n, "identifier"));
        auto fields = s_.integers(n, "fields");
        bool owners = s_.field(n, "legal_owners") != nullptr;
        if (identifier > 0 && owners &&
            std::all_of(fields.begin(), fields.end(), [](int64_t k) { return k <= 10; })) {
          std::string name;
          if (const DecodedNode* k = s_.node(identifier)) name = s_.chars(*k, "string");
          defs[n.index] = Def{name, fields};
        }
      } else if (n.node_type == xt::kIntValues) {
        auto values = s_.integers(n, "values");
        if (values.size() == 1 || (values.size() >= 4 && values.size() <= 7)) {
          std::vector<uint32_t> u;
          for (int64_t v : values) u.push_back(static_cast<uint32_t>(v));
          lists[n.index] = std::move(u);
        }
      }
    }
    auto family_of = [&](uint32_t def) -> int {
      auto it = defs.find(def);
      if (it == defs.end() || !it->second) return -1;
      const Def& d = *it->second;
      if (d.name == "FACE_ID_2001" && d.fields == std::vector<int64_t>{9, 9, 1, 1}) return 0;
      if (d.name == "ATOM_ID_2001" && d.fields == std::vector<int64_t>{9, 1}) return 1;
      if (d.name == "SWEntUnchanged" && d.fields == std::vector<int64_t>{1}) return 2;
      return -1;
    };
    // owner attr -> payload (nullopt = conflicting)
    std::map<uint32_t, std::optional<std::vector<uint32_t>>> face_ids, body_ids, keys;
    std::map<uint32_t, std::optional<PersistentFaceIdentity>> atoms;
    std::set<uint32_t> atom_owners;
    auto consistent = [](auto& map, uint32_t owner, const auto& value) {
      auto it = map.find(owner);
      if (it == map.end()) map.emplace(owner, value);
      else if (it->second && *it->second != value) it->second.reset();
    };
    for (const DecodedNode* node_ptr : s_.nodes()) if (const DecodedNode& n = *node_ptr; true) {
      if (n.node_type != xt::kAttribute) continue;
      int64_t node_id = s_.integer(n, "node_id");
      if ((node_id >> 16) != 0 || node_id == 0) continue;
      int family = family_of(static_cast<uint32_t>(s_.integer(n, "definition")));
      if (family < 0) continue;
      int64_t owner_raw = s_.integer(n, "owner");
      if (owner_raw <= 0) continue;
      uint32_t owner = static_cast<uint32_t>(owner_raw) + 1;
      auto refs = s_.integers(n, "fields");
      static const std::vector<int64_t> kFace{9, 9, 1, 1}, kBody{9, 1}, kKey{1};
      const auto& kinds = family == 0 ? kFace : family == 1 ? kBody : kKey;
      if (refs.size() != kinds.size()) continue;
      size_t pos = static_cast<size_t>(std::find(kinds.begin(), kinds.end(), 1) - kinds.begin());
      auto li = lists.find(static_cast<uint32_t>(refs[pos]));
      if (li == lists.end()) continue;
      const auto& values = li->second;
      auto source_ok = [](uint32_t v) { return v != 0 && v != UINT32_MAX; };
      if (family == 0) {
        bool valid = values.size() >= 4 && values.size() <= 7 && values[3] == 0 &&
                     (values.size() != 4 || values[0] == 36) && source_ok(values[1]) &&
                     values[2] != 0 && values[2] != UINT32_MAX;
        if (valid) consistent(face_ids, owner, std::optional<std::vector<uint32_t>>(values));
        if (values.size() >= 5 && values.size() <= 7 && values[3] == 0) {
          std::optional<PersistentFaceIdentity> id;
          if (source_ok(values[1]))
            id = PersistentFaceIdentity{values[1], values[4],
                                        std::vector<uint32_t>(values.begin() + 5, values.end())};
          auto it = atoms.find(owner);
          if (it == atoms.end()) atoms.emplace(owner, id);
          else if (it->second != id) it->second.reset(), atom_owners.insert(owner);
        }
      } else {
        bool valid = values.size() == 1 && values[0] != 0 && values[0] != UINT32_MAX;
        if (valid)
          consistent(family == 1 ? body_ids : keys, owner, std::optional<std::vector<uint32_t>>(values));
      }
    }
    std::unordered_set<std::string> emitted;
    for (const auto& f : out.faces) emitted.insert(f.id);
    for (const auto& [owner, v] : face_ids)
      if (v && emitted.count(ident("face", owner)))
        out.face_payloads.emplace_back(ident("face", owner), *v);
    for (const auto& [owner, v] : keys)
      if (v && v->size() == 1 && emitted.count(ident("face", owner)))
        out.face_keys.emplace_back(ident("face", owner), (*v)[0]);
    for (const auto& [owner, id] : atoms)
      if (id && !atom_owners.count(owner) && emitted.count(ident("face", owner)))
        out.face_identities.emplace_back(ident("face", owner), *id);
    // Bodies are emitted below; body payloads are bound after grouping.
    for (const auto& [owner, v] : body_ids)
      if (v) out.body_payloads.emplace_back(ident("body", owner), *v);
  }

  // --- Synthetic seams (cadmpeg `synthesize_cylinder_seams`,
  // `synthesize_sphere_seams`) and full-turn phases.
  {
    std::unordered_map<std::string, size_t> surf_at, loop_at, coedge_at, edge_at, curve_at,
        vertex_at, point_at;
    auto index_all = [&] {
      surf_at.clear(); loop_at.clear(); coedge_at.clear(); edge_at.clear(); curve_at.clear();
      vertex_at.clear(); point_at.clear();
      for (size_t i = 0; i < out.surfaces.size(); ++i) surf_at[out.surfaces[i].id] = i;
      for (size_t i = 0; i < out.loops.size(); ++i) loop_at[out.loops[i].id] = i;
      for (size_t i = 0; i < out.coedges.size(); ++i) coedge_at[out.coedges[i].id] = i;
      for (size_t i = 0; i < out.edges.size(); ++i) edge_at[out.edges[i].id] = i;
      for (size_t i = 0; i < out.curves.size(); ++i) curve_at[out.curves[i].id] = i;
      for (size_t i = 0; i < out.vertices.size(); ++i) vertex_at[out.vertices[i].id] = i;
      for (size_t i = 0; i < out.points.size(); ++i) point_at[out.points[i].id] = i;
    };
    index_all();
    auto curve_of = [&](const Edge& e) -> const Curve* {
      if (!e.curve) return nullptr;
      auto it = curve_at.find(*e.curve);
      return it == curve_at.end() ? nullptr : &out.curves[it->second];
    };
    auto set_point = [&](const std::string& vertex, Vec3 pos) {
      auto vi = vertex_at.find(vertex);
      if (vi == vertex_at.end()) return;
      auto pi = point_at.find(out.vertices[vi->second].point);
      if (pi != point_at.end()) out.points[pi->second].position = pos;
    };
    // Cylinder seams.
    struct CylSeam {
      size_t face;
      std::string loop_a, loop_b, circle_a, circle_b, vertex_a, vertex_b;
      Vec3 pa, pb;
    };
    std::vector<CylSeam> cyl;
    for (size_t fi = 0; fi < out.faces.size(); ++fi) {
      const Face& face = out.faces[fi];
      auto si = surf_at.find(*face.surface);
      if (si == surf_at.end() || out.surfaces[si->second].geometry.kind != "cylinder") continue;
      Vec3 ref = out.surfaces[si->second].geometry.u_axis;
      if (face.loops.size() != 2) continue;
      auto la = loop_at.find(face.loops[0]), lb = loop_at.find(face.loops[1]);
      if (la == loop_at.end() || lb == loop_at.end()) continue;
      const Loop& a = out.loops[la->second];
      const Loop& b = out.loops[lb->second];
      if (a.coedges.size() != 1 || b.coedges.size() != 1) continue;
      auto ca = coedge_at.find(a.coedges[0]), cb = coedge_at.find(b.coedges[0]);
      if (ca == coedge_at.end() || cb == coedge_at.end()) continue;
      auto ea = edge_at.find(out.coedges[ca->second].edge);
      auto eb = edge_at.find(out.coedges[cb->second].edge);
      if (ea == edge_at.end() || eb == edge_at.end()) continue;
      auto seam_point = [&](const Edge& e) -> std::optional<Vec3> {
        if (e.start != e.end) return std::nullopt;
        const Curve* c = curve_of(e);
        if (!c || c->geometry.kind != "circle") return std::nullopt;
        return sub(c->geometry.center, scale(ref, c->geometry.radius));
      };
      const Edge& edge_a = out.edges[ea->second];
      const Edge& edge_b = out.edges[eb->second];
      auto pa = seam_point(edge_a), pb = seam_point(edge_b);
      if (pa && pb)
        cyl.push_back({fi, a.id, b.id, a.coedges[0], b.coedges[0], *edge_a.start, *edge_b.start,
                       *pa, *pb});
    }
    std::unordered_set<std::string> removed_loops;
    for (const auto& c : cyl) {
      set_point(c.vertex_a, c.pa);
      set_point(c.vertex_b, c.pb);
      Vec3 d = sub(c.pb, c.pa);
      double n = norm(d);
      if (n == 0.0) continue;
      d = scale(d, 1.0 / n);
      std::string key = out.faces[c.face].id.substr(out.faces[c.face].id.find('#') + 1);
      std::string curve_id = "sldprt:brep:curve#seam:" + key;
      std::string edge_id = "sldprt:brep:edge#seam:" + key;
      std::string seam_a = "sldprt:brep:coedge#seam:" + key + ":0";
      std::string seam_b = "sldprt:brep:coedge#seam:" + key + ":1";
      Curve curve;
      curve.id = curve_id;
      curve.geometry.kind = "line";
      curve.geometry.origin = c.pa;
      curve.geometry.direction = d;
      out.curves.push_back(curve);
      Edge edge;
      edge.id = edge_id;
      edge.curve = curve_id;
      edge.param_range = std::array<double, 2>{0.0, n};
      edge.start = c.vertex_a;
      edge.end = c.vertex_b;
      out.edges.push_back(edge);
      out.coedges.push_back({seam_a, c.loop_a, edge_id, false});
      out.coedges.push_back({seam_b, c.loop_a, edge_id, true});
      for (auto& ce : out.coedges)
        if (ce.id == c.circle_b) ce.owner_loop = c.loop_a;
      for (auto& lp : out.loops)
        if (lp.id == c.loop_a) lp.coedges = {c.circle_a, seam_a, c.circle_b, seam_b};
      out.faces[c.face].loops = {c.loop_a};
      removed_loops.insert(c.loop_b);
    }
    std::erase_if(out.loops, [&](const Loop& l) { return removed_loops.count(l.id) != 0; });
    index_all();

    // Sphere seams.
    std::unordered_map<std::string, Vec3> vertex_points;
    for (const auto& v : out.vertices) {
      auto pi = point_at.find(v.point);
      if (pi != point_at.end()) vertex_points[v.id] = out.points[pi->second].position;
    }
    auto is_circle_edge = [&](const std::string& coedge) {
      auto ci = coedge_at.find(coedge);
      if (ci == coedge_at.end()) return false;
      auto ei = edge_at.find(out.coedges[ci->second].edge);
      if (ei == edge_at.end()) return false;
      const Curve* c = curve_of(out.edges[ei->second]);
      return c && c->geometry.kind == "circle";
    };
    std::vector<std::pair<size_t, Vec3>> existing;
    for (const Face& face : out.faces) {
      auto si = surf_at.find(*face.surface);
      if (si == surf_at.end() || out.surfaces[si->second].geometry.kind != "sphere") continue;
      const auto& g = out.surfaces[si->second].geometry;
      if (face.loops.size() != 1) continue;
      auto li = loop_at.find(face.loops[0]);
      if (li == loop_at.end() || out.loops[li->second].coedges.size() != 4) continue;
      std::vector<size_t> seam_edges;
      size_t circles = 0;
      for (const auto& ce : out.loops[li->second].coedges) {
        auto ci = coedge_at.find(ce);
        if (ci == coedge_at.end()) continue;
        auto ei = edge_at.find(out.coedges[ci->second].edge);
        if (ei == edge_at.end()) continue;
        if (!out.edges[ei->second].curve) seam_edges.push_back(ei->second);
        if (is_circle_edge(ce)) ++circles;
      }
      if (seam_edges.size() == 1 && circles == 3)
        existing.emplace_back(seam_edges[0], add(g.origin, scale(g.normal, g.radius)));
    }
    for (const auto& [ei, pole] : existing) {
      Edge& e = out.edges[ei];
      if (e.start) set_point(*e.start, pole);
      if (e.end) set_point(*e.end, pole);
      std::string curve_id =
          "sldprt:brep:curve#sphere-seam:" + e.id.substr(e.id.find('#') + 1);
      Curve c;
      c.id = curve_id;
      c.geometry.kind = "degenerate";
      c.geometry.center = pole;
      out.curves.push_back(c);
      e.curve = curve_id;
    }
    index_all();
    struct SphereSeam {
      size_t face;
      std::string loop;
      Vec3 pole;
      std::optional<std::string> pole_vertex;
    };
    std::vector<SphereSeam> spheres;
    for (size_t fi = 0; fi < out.faces.size(); ++fi) {
      const Face& face = out.faces[fi];
      auto si = surf_at.find(*face.surface);
      if (si == surf_at.end() || out.surfaces[si->second].geometry.kind != "sphere") continue;
      const auto& g = out.surfaces[si->second].geometry;
      if (face.loops.size() != 1) continue;
      auto li = loop_at.find(face.loops[0]);
      if (li == loop_at.end()) continue;
      const Loop& lp = out.loops[li->second];
      if (lp.coedges.size() != 3) continue;
      if (!std::all_of(lp.coedges.begin(), lp.coedges.end(), is_circle_edge)) continue;
      Vec3 pole = add(g.origin, scale(g.normal, g.radius));
      std::vector<std::string> poles;
      for (const auto& ce : lp.coedges) {
        auto ci = coedge_at.find(ce);
        if (ci == coedge_at.end()) continue;
        auto ei = edge_at.find(out.coedges[ci->second].edge);
        if (ei == edge_at.end()) continue;
        for (const auto& v : {out.edges[ei->second].start, out.edges[ei->second].end}) {
          if (!v) continue;
          auto vp = vertex_points.find(*v);
          if (vp == vertex_points.end()) continue;
          Vec3 d = sub(vp->second, pole);
          if (dot(d, d) <= kEpsPointDistance) poles.push_back(*v);
        }
      }
      std::sort(poles.begin(), poles.end());
      SphereSeam s{fi, lp.id, pole, std::nullopt};
      if (!poles.empty()) s.pole_vertex = poles.front();
      spheres.push_back(std::move(s));
    }
    for (const auto& s : spheres) {
      std::string key = "sphere-seam-face:" + std::to_string(s.face);
      std::string vertex;
      if (s.pole_vertex) {
        vertex = *s.pole_vertex;
      } else {
        std::string pid = "sldprt:brep:point#" + key;
        vertex = "sldprt:brep:vertex#" + key;
        out.points.push_back({pid, s.pole});
        out.vertices.push_back({vertex, pid});
      }
      std::string curve_id = "sldprt:brep:curve#" + key;
      std::string edge_id = "sldprt:brep:edge#" + key;
      std::string coedge_id = "sldprt:brep:coedge#" + key;
      auto li = loop_at.find(s.loop);
      auto& ring = out.loops[li->second].coedges;
      size_t after = ring.size();
      for (size_t i = 0; i < ring.size(); ++i) {
        auto ci = coedge_at.find(ring[i]);
        if (ci == coedge_at.end()) continue;
        const Coedge& ce = out.coedges[ci->second];
        auto ei = edge_at.find(ce.edge);
        if (ei == edge_at.end()) continue;
        const Edge& e = out.edges[ei->second];
        if ((!ce.reversed ? e.end : e.start) == std::optional<std::string>(vertex)) {
          after = i + 1;
          break;
        }
      }
      Curve c;
      c.id = curve_id;
      c.geometry.kind = "degenerate";
      c.geometry.center = s.pole;
      out.curves.push_back(c);
      Edge e;
      e.id = edge_id;
      e.curve = curve_id;
      e.start = vertex;
      e.end = vertex;
      out.edges.push_back(e);
      ring.insert(ring.begin() + static_cast<std::ptrdiff_t>(after), coedge_id);
      out.coedges.push_back({coedge_id, s.loop, edge_id, false});
    }
    index_all();

    // Full-turn phases (cadmpeg `phase_full_turn_edge_ranges`).
    for (auto& e : out.edges) {
      if (e.start != e.end || !e.param_range || !e.start) continue;
      auto [lo, hi] = *e.param_range;
      if (std::abs(std::abs(hi - lo) - kTau) > kFullTurnTolerance) continue;
      const Curve* c = curve_of(e);
      if (!c) continue;
      auto vi = vertex_at.find(*e.start);
      if (vi == vertex_at.end()) continue;
      auto pi = point_at.find(out.vertices[vi->second].point);
      if (pi == point_at.end()) continue;
      double bound = kLinearResolution;
      auto phase = carrier_angle(c->geometry, out.points[pi->second].position, bound);
      if (!phase || std::abs(*phase - std::min(lo, hi)) <= kFullTurnTolerance) continue;
      e.param_range = std::array<double, 2>{*phase, *phase + kTau};
    }
  }

  // --- Prune (cadmpeg `prune_rejected_topology`).
  {
    std::unordered_set<std::string> keep;
    for (const auto& f : out.faces) keep.insert(f.loops.begin(), f.loops.end());
    std::erase_if(out.loops, [&](const Loop& l) { return !keep.count(l.id); });
    keep.clear();
    for (const auto& l : out.loops) keep.insert(l.coedges.begin(), l.coedges.end());
    std::erase_if(out.coedges, [&](const Coedge& c) { return !keep.count(c.id); });
    keep.clear();
    for (const auto& c : out.coedges) keep.insert(c.edge);
    std::erase_if(out.edges, [&](const Edge& e) { return !keep.count(e.id); });
    keep.clear();
    for (const auto& e : out.edges) {
      if (e.start) keep.insert(*e.start);
      if (e.end) keep.insert(*e.end);
    }
    // Vertex loops keep their vertex.
    for (const auto& f : faces)
      for (const auto& [lp, v] : f.vertex_loops)
        if (kept_vertices.count(v)) keep.insert(ident("vertex", attr(v)));
    std::erase_if(out.vertices, [&](const Vertex& v) { return !keep.count(v.id); });
    keep.clear();
    for (const auto& v : out.vertices) keep.insert(v.point);
    std::erase_if(out.points, [&](const Point& p) { return !keep.count(p.id); });
    keep.clear();
    for (const auto& e : out.edges)
      if (e.curve) keep.insert(*e.curve);
    keep.insert(spine_curves.begin(), spine_curves.end());
    std::erase_if(out.curves, [&](const Curve& c) { return !keep.count(c.id); });
  }

  if (out.faces.empty()) {
    SiteBrep empty;
    empty.entity_records = out.entity_records;
    return empty;
  }

  // --- Bodies, regions and shells, splitting each native shell into its
  // edge-connected face components (cadmpeg `shell_face_components`).
  std::unordered_map<std::string, std::vector<std::string>> loop_edges;
  for (const auto& c : out.coedges) loop_edges[c.owner_loop].push_back(c.edge);
  std::unordered_map<std::string, size_t> face_at;
  for (size_t i = 0; i < out.faces.size(); ++i) face_at[out.faces[i].id] = i;
  auto components = [&](const std::string& shell_id) {
    std::vector<std::string> candidates;
    for (const auto& f : out.faces)
      if (f.shell == shell_id) candidates.push_back(f.id);
    std::unordered_map<std::string, std::vector<std::string>> faces_by_edge;
    for (const auto& fid : candidates)
      for (const auto& lp : out.faces[face_at[fid]].loops)
        for (const auto& e : loop_edges[lp]) faces_by_edge[e].push_back(fid);
    std::unordered_map<std::string, std::set<std::string>> neighbours;
    for (auto& [e, fs] : faces_by_edge)
      for (const auto& a : fs)
        for (const auto& b : fs)
          if (a != b) neighbours[a].insert(b);
    std::unordered_set<std::string> assigned;
    std::vector<std::vector<std::string>> result;
    for (const auto& fid : candidates) {
      if (!assigned.insert(fid).second) continue;
      std::vector<std::string> comp, pending{fid};
      while (!pending.empty()) {
        std::string cur = pending.back();
        pending.pop_back();
        comp.push_back(cur);
        for (const auto& nb : neighbours[cur])
          if (assigned.insert(nb).second) pending.push_back(nb);
      }
      std::sort(comp.begin(), comp.end());
      result.push_back(std::move(comp));
    }
    return result;
  };
  auto emit_shells = [&](const std::string& native_shell, const std::string& region_id,
                         Region& region) {
    auto comps = components(native_shell);
    for (size_t k = 0; k < comps.size(); ++k) {
      std::string shell_id = k == 0 ? native_shell : native_shell + ".component-" + std::to_string(k);
      for (const auto& fid : comps[k]) out.faces[face_at[fid]].shell = shell_id;
      out.shells.push_back({shell_id, region_id, comps[k]});
      region.shells.push_back(shell_id);
    }
  };
  if (hierarchy_ok) {
    for (const auto& b : bodies) {
      Body body;
      body.id = ident("body", attr(b.body));
      body.kind = b.kind;
      for (const auto& r : b.regions) {
        Region region;
        region.id = ident("region", attr(r.region));
        region.body = body.id;
        for (const auto& sh : r.shells) emit_shells(ident("shell", attr(sh.shell)), region.id, region);
        body.regions.push_back(region.id);
        out.regions.push_back(std::move(region));
      }
      out.bodies.push_back(std::move(body));
    }
  } else {
    // Synthetic grouping: one body, region and shell (cadmpeg group 0).
    Body body{"sldprt:brep:body#0", "solid", {}, std::nullopt};
    Region region{"sldprt:brep:region#0", body.id, {}};
    for (auto& f : out.faces) f.shell = "sldprt:brep:shell#0";
    emit_shells("sldprt:brep:shell#0", region.id, region);
    body.regions.push_back(region.id);
    out.regions.push_back(std::move(region));
    out.bodies.push_back(std::move(body));
  }
  {
    std::unordered_set<std::string> body_ids;
    for (const auto& b : out.bodies) body_ids.insert(b.id);
    std::erase_if(out.body_payloads, [&](const auto& p) { return !body_ids.count(p.first); });
  }
  return out;
}

std::optional<SiteBrep> decode_site(const std::vector<const PsStream*>& streams) {
  std::optional<SiteBrep> out;
  // cadmpeg `scan_site`: partition streams first, then the deltas journals,
  // which take part in face-colour scanning only.
  std::vector<std::unique_ptr<xt::Stream>> parsed;
  for (const PsStream* ps : streams)
    if (!is_deltas_stream(*ps))
      {
        auto stream = xt::Stream::parse(ps->payload);
        if (stream) parsed.push_back(std::move(stream));
      }
  for (const PsStream* ps : streams)
    if (is_deltas_stream(*ps) && !parsed.empty()) parsed.front()->apply_deltas(ps->payload);
  for (const auto& stream : parsed) {
    Decoder decoder(*stream);
    SiteBrep site = decoder.run();
    if (!out) out = std::move(site);
    else merge_site(*out, std::move(site));
  }
  return out;
}

namespace {
void q(std::string& s, const std::string& suffix) { s += suffix; }
void q(std::optional<std::string>& s, const std::string& suffix) {
  if (s) *s += suffix;
}
void q(std::vector<std::string>& v, const std::string& suffix) {
  for (auto& s : v) s += suffix;
}
template <typename T>
void append(std::vector<T>& into, std::vector<T>& from) {
  into.insert(into.end(), std::make_move_iterator(from.begin()),
              std::make_move_iterator(from.end()));
}
}  // namespace

void qualify_site(SiteBrep& b, const std::string& site) {
  const std::string sfx = "@" + site;
  for (auto& x : b.bodies) q(x.id, sfx), q(x.regions, sfx);
  for (auto& x : b.regions) q(x.id, sfx), q(x.body, sfx), q(x.shells, sfx);
  for (auto& x : b.shells) q(x.id, sfx), q(x.region, sfx), q(x.faces, sfx);
  for (auto& x : b.faces) q(x.id, sfx), q(x.shell, sfx), q(x.surface, sfx), q(x.loops, sfx);
  for (auto& x : b.loops) q(x.id, sfx), q(x.face, sfx), q(x.coedges, sfx);
  for (auto& x : b.coedges) q(x.id, sfx), q(x.owner_loop, sfx), q(x.edge, sfx);
  for (auto& x : b.edges) q(x.id, sfx), q(x.curve, sfx), q(x.start, sfx), q(x.end, sfx);
  for (auto& x : b.vertices) q(x.id, sfx), q(x.point, sfx);
  for (auto& x : b.points) q(x.id, sfx);
  for (auto& x : b.surfaces) q(x.id, sfx);
  for (auto& x : b.curves) q(x.id, sfx);
  for (auto& x : b.face_keys) q(x.first, sfx);
  for (auto& x : b.face_identities) q(x.first, sfx);
  for (auto& x : b.face_payloads) q(x.first, sfx);
  for (auto& x : b.body_payloads) q(x.first, sfx);
  for (auto& x : b.face_colors) {
    if (x.target) q(*x.target, sfx);
    x.site = site;
  }
}

void merge_site(SiteBrep& into, SiteBrep&& from) {
  append(into.bodies, from.bodies);
  append(into.regions, from.regions);
  append(into.shells, from.shells);
  append(into.faces, from.faces);
  append(into.loops, from.loops);
  append(into.coedges, from.coedges);
  append(into.edges, from.edges);
  append(into.vertices, from.vertices);
  append(into.points, from.points);
  append(into.surfaces, from.surfaces);
  append(into.curves, from.curves);
  append(into.face_keys, from.face_keys);
  append(into.face_identities, from.face_identities);
  append(into.face_payloads, from.face_payloads);
  append(into.body_payloads, from.body_payloads);
  append(into.face_colors, from.face_colors);
  into.entity_records += from.entity_records;
}

}  // namespace swglb
