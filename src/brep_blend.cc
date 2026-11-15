// SPDX-License-Identifier: Apache-2.0
// Port of cadmpeg-codec-sldprt `brep/blend_surface.rs` (cadmpeg 5af95421,
// Apache-2.0). NURBS support and cache evaluation goes through the ownership
// stage's port of cadmpeg-ir `eval`, so a cache is measured the same way
// there.
#include "brep_blend.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <tuple>

#include "ownership_internal.h"

namespace swglb::brep {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kTau = 6.283185307179586;
constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

constexpr size_t MAX_PATCHES = 512;
constexpr size_t SEED_PATCHES = 4;
constexpr size_t SPINE_SAMPLES = 256;
constexpr size_t PROJECTION_STEPS = 80;
constexpr size_t EDGE_SAMPLES = 32;
constexpr double CONTACT_RELATIVE_TOLERANCE = 1.0e-3;
constexpr double CENTER_RELATIVE_TOLERANCE = 1.0e-10;
constexpr size_t CENTER_STEPS = 16;
constexpr double MIN_SECTION_ANGLE = 1.0e-6;
constexpr double TANGENT_STEP = 1.0e-6;
constexpr double RING_GAP_SHARE = 0.125;
constexpr double EXTENT_MARGIN = 0.05;
constexpr double NODE_FRACTIONS[4] = {0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0};
constexpr double PROBE_U[3] = {1.0 / 6.0, 0.5, 5.0 / 6.0};
constexpr double PROBE_V[5] = {0.0, 0.25, 0.5, 0.75, 1.0};
constexpr double VERTEX_ALLOWANCE = 1.0e-5;
constexpr double VERTEX_SEEDS_V[3] = {0.0, 0.5, 1.0};

using V = std::array<double, 3>;
using H = std::array<double, 4>;
using Seed = std::optional<std::array<double, 2>>;

V to3(const Vec3& p) { return {p.x, p.y, p.z}; }
Vec3 from3(const V& p) { return Vec3{p[0], p[1], p[2]}; }
V add(V a, V b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V sub(V a, V b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V scale(V a, double f) { return {a[0] * f, a[1] * f, a[2] * f}; }
double dot(V a, V b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V cross(V a, V b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double norm(V a) { return std::sqrt(dot(a, a)); }
std::optional<V> unit(V a) {
  double l = norm(a);
  if (!(l > 0.0 && std::isfinite(l))) return std::nullopt;
  return scale(a, 1.0 / l);
}
double lerp(std::array<double, 2> r, double f) { return r[0] * (1.0 - f) + r[1] * f; }
double fraction_of(size_t index, size_t count) {
  return count > 0 ? static_cast<double>(index) / static_cast<double>(count) : NAN;
}
double rem_euclid(double a, double b) {
  double r = std::fmod(a, b);
  return r < 0 ? r + std::fabs(b) : r;
}
H homogeneous(V p, double w) { return {p[0] * w, p[1] * w, p[2] * w, w}; }
bool finite3(const V& p) { return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]); }

// --- NURBS curves (cadmpeg-ir `nurbs_curve_point_evaluation`).
size_t bspline_span(const std::vector<double>& knots, size_t degree, size_t count, double t) {
  if (t >= knots[count]) return count - 1;
  if (t <= knots[degree]) return degree;
  size_t low = degree, high = count;
  while (high - low > 1) {
    size_t mid = (low + high) / 2;
    if (t < knots[mid]) high = mid;
    else low = mid;
  }
  return low;
}

std::optional<V> nurbs_curve_point(const NurbsCurveGeometry& c, double t) {
  size_t p = c.degree, n = c.poles.size();
  if (n == 0 || p >= n + 1 || c.knots.size() < n + p + 1 || !std::isfinite(t)) return std::nullopt;
  if (!c.weights.empty() && c.weights.size() != n) return std::nullopt;
  size_t span = bspline_span(c.knots, p, n, t);
  std::vector<H> d(p + 1);
  for (size_t j = 0; j <= p; ++j) {
    size_t i = span - p + j;
    double w = c.weights.empty() ? 1.0 : c.weights[i];
    d[j] = homogeneous(to3(c.poles[i]), w);
  }
  for (size_t r = 1; r <= p; ++r)
    for (size_t j = p; j >= r; --j) {
      size_t i = span - p + j;
      double denom = c.knots[i + p - r + 1] - c.knots[i];
      double a = denom == 0.0 ? 0.0 : (t - c.knots[i]) / denom;
      for (int k = 0; k < 4; ++k) d[j][k] = (1.0 - a) * d[j - 1][k] + a * d[j][k];
    }
  const H& h = d[p];
  if (!(h[3] != 0.0)) return std::nullopt;
  V out{h[0] / h[3], h[1] / h[3], h[2] / h[3]};
  if (!finite3(out)) return std::nullopt;
  return out;
}

std::optional<V> curve_point(const CurveGeometry& g, double t) {
  if (!std::isfinite(t)) return std::nullopt;
  if (g.kind == "line") return add(to3(g.origin), scale(to3(g.direction), t));
  if (g.kind == "circle" || g.kind == "ellipse") {
    V y = cross(to3(g.axis), to3(g.ref_direction));
    double minor = g.kind == "circle" ? g.radius : g.minor_radius;
    return add(to3(g.center), add(scale(to3(g.ref_direction), g.radius * std::cos(t)),
                                  scale(y, minor * std::sin(t))));
  }
  if (g.kind == "nurbs" && g.nurbs) return nurbs_curve_point(*g.nurbs, t);
  return std::nullopt;
}

// --- Support feet.
std::optional<std::pair<double, double>> least_squares_step(V du, V dv, V residual) {
  if (!finite3(residual)) return std::nullopt;
  double du_scale = std::max({std::fabs(du[0]), std::fabs(du[1]), std::fabs(du[2])});
  double dv_scale = std::max({std::fabs(dv[0]), std::fabs(dv[1]), std::fabs(dv[2])});
  if (du_scale == 0.0 || dv_scale == 0.0) return std::nullopt;
  du = scale(du, 1.0 / du_scale);
  dv = scale(dv, 1.0 / dv_scale);
  double a = dot(du, du), m = dot(du, dv), b = dot(dv, dv);
  double det = std::fma(a, b, -m * m);
  if (det <= kEpsilon * a * b) return std::nullopt;
  double ru = dot(du, residual), rv = dot(dv, residual);
  double su = (b * ru - m * rv) / det, sv = (a * rv - m * ru) / det;
  su /= du_scale;
  sv /= dv_scale;
  if (!std::isfinite(su) || !std::isfinite(sv)) return std::nullopt;
  return std::make_pair(su, sv);
}

/// cadmpeg-ir `nurbs_surface_parameter_near_point` with an optional seed.
std::optional<std::array<double, 2>> nurbs_near(const NurbsSurfaceGeometry& s, V point, Seed seed) {
  if (!seed) {
    auto p = ownership::nurbs_surface_parameter_near_point(s, from3(point));
    if (!p) return std::nullopt;
    return std::array<double, 2>{p->u, p->v};
  }
  if (!finite3(point)) return std::nullopt;
  size_t ud = s.u_degree, vd = s.v_degree, uc = s.poles.size();
  size_t vc = s.poles.empty() ? 0 : s.poles[0].size();
  if (ud > uc || uc >= s.u_knots.size() || vd > vc || vc >= s.v_knots.size()) return std::nullopt;
  double u0 = s.u_knots[ud], u1 = s.u_knots[uc], v0 = s.v_knots[vd], v1 = s.v_knots[vc];
  if (!(u0 < u1) || !(v0 < v1)) return std::nullopt;
  if (!std::isfinite((*seed)[0]) || !std::isfinite((*seed)[1])) {
    auto p = ownership::nurbs_surface_parameter_near_point(s, from3(point));
    if (!p) return std::nullopt;
    return std::array<double, 2>{p->u, p->v};
  }
  std::array<double, 2> q{std::clamp((*seed)[0], u0, u1), std::clamp((*seed)[1], v0, v1)};
  for (size_t it = 0; it < 24; ++it) {
    auto partials = ownership::nurbs_surface_partials(s, q[0], q[1]);
    if (!partials) return std::nullopt;
    double current = norm(sub(to3(partials->point), point));
    if (current == 0.0) return q;
    auto step = least_squares_step(to3(partials->du), to3(partials->dv),
                                   sub(to3(partials->point), point));
    if (!step) break;
    double f = 1.0;
    bool accepted = false;
    for (size_t search = 0; search < 12; ++search) {
      std::array<double, 2> c{std::clamp(q[0] - f * step->first, u0, u1),
                              std::clamp(q[1] - f * step->second, v0, v1)};
      auto cp = ownership::nurbs_surface_point(s, c[0], c[1]);
      if (!cp) return std::nullopt;
      if (norm(sub(to3(*cp), point)) < current) {
        q = c;
        accepted = true;
        break;
      }
      f *= 0.5;
    }
    if (!accepted) break;
  }
  return q;
}

std::optional<std::pair<V, Seed>> foot(const SurfaceGeometry& g, V c, Seed seed) {
  V o = to3(g.origin), axis = to3(g.normal);
  if (g.kind == "plane") {
    return std::make_pair(sub(c, scale(axis, dot(sub(c, o), axis))), Seed{});
  }
  if (g.kind == "cylinder") {
    V d = sub(c, o);
    double along = dot(d, axis);
    V radial = sub(d, scale(axis, along));
    auto r = unit(radial);
    if (!r) {
      auto ref = unit(sub(to3(g.u_axis), scale(axis, dot(to3(g.u_axis), axis))));
      if (!ref) return std::nullopt;
      r = ref;
    }
    return std::make_pair(add(add(o, scale(axis, along)), scale(*r, g.radius)), Seed{});
  }
  if (g.kind == "sphere") {
    auto r = unit(sub(c, o));
    if (!r) return std::nullopt;
    return std::make_pair(add(o, scale(*r, g.radius)), Seed{});
  }
  if (g.kind == "torus") {
    V d = sub(c, o);
    double along = dot(d, axis);
    V radial = sub(d, scale(axis, along));
    auto r = unit(radial);
    if (!r) return std::nullopt;
    V ring = add(o, scale(*r, g.radius));
    double rho = norm(radial) - g.radius;
    double angle = std::atan2(along, rho);
    V p = add(ring, add(scale(*r, g.minor_radius * std::cos(angle)),
                        scale(axis, g.minor_radius * std::sin(angle))));
    return std::make_pair(p, Seed{});
  }
  if (g.kind == "cone") {
    if (g.ratio != 1.0) return std::nullopt;
    V ref = to3(g.u_axis);
    V d = sub(c, o);
    double along = dot(d, axis), across = dot(d, ref), transverse = dot(d, cross(axis, ref));
    double slope = std::tan(g.half_angle);
    double radius = g.radius;
    double height = (along + slope * (std::hypot(across, transverse) - radius)) / (1.0 + slope * slope);
    if (radius + height * slope <= 0.0) return std::nullopt;
    double u = std::atan2(transverse, across);
    double local = radius + height * slope;
    V p = add(add(o, scale(axis, height)),
              add(scale(ref, local * std::cos(u)), scale(cross(axis, ref), local * std::sin(u))));
    if (!finite3(p)) return std::nullopt;
    return std::make_pair(p, Seed{});
  }
  if (g.kind == "nurbs" && g.nurbs) {
    auto q = nurbs_near(*g.nurbs, c, seed);
    if (!q) return std::nullopt;
    auto p = ownership::nurbs_surface_point(*g.nurbs, (*q)[0], (*q)[1]);
    if (!p) return std::nullopt;
    return std::make_pair(to3(*p), Seed{*q});
  }
  return std::nullopt;
}

// --- Spine.
enum class Domain { Line, Bounded, Periodic };

struct Spine {
  const CurveGeometry* curve = nullptr;
  Domain domain = Domain::Line;
  std::array<double, 2> range{0, 0};
  std::vector<double> knots;  ///< Interior knots of a NURBS spine.

  static std::optional<Spine> of(const CurveGeometry& c) {
    Spine s;
    s.curve = &c;
    if (c.kind == "line") {
      s.domain = Domain::Line;
    } else if (c.kind == "circle" || c.kind == "ellipse") {
      s.domain = Domain::Periodic;
      s.range = {0.0, kTau};
    } else if (c.kind == "nurbs" && c.nurbs) {
      const auto& n = *c.nurbs;
      size_t p = n.degree, count = n.poles.size();
      if (count >= n.knots.size() || p >= n.knots.size()) return std::nullopt;
      double first = n.knots[p], last = n.knots[count];
      if (first >= last) return std::nullopt;
      for (double k : n.knots)
        if (k > first && k < last) s.knots.push_back(k);
      auto a = curve_point(c, first), b = curve_point(c, last);
      if (!a || !b) return std::nullopt;
      double gap = norm(sub(*a, *b));
      double sc = std::max(norm(*a), 1.0);
      s.domain = (n.periodic || gap <= sc * kEpsilon * 1024.0) ? Domain::Periodic : Domain::Bounded;
      s.range = {first, last};
    } else {
      return std::nullopt;
    }
    return s;
  }

  double wrap(double t) const {
    if (domain == Domain::Periodic) return range[0] + rem_euclid(t - range[0], range[1] - range[0]);
    return t;
  }

  std::optional<V> point(double t) const { return curve_point(*curve, wrap(t)); }

  std::optional<double> project(V p) const {
    if (domain == Domain::Line) {
      auto o = point(0.0), u = point(1.0);
      if (!o || !u) return std::nullopt;
      V d = sub(*u, *o);
      double l = dot(d, d);
      if (l <= 0.0) return std::nullopt;
      return dot(sub(p, *o), d) / l;
    }
    auto [first, last] = range;
    std::optional<std::pair<size_t, double>> best;
    for (size_t i = 0; i <= SPINE_SAMPLES; ++i) {
      auto sample = point(lerp(range, fraction_of(i, SPINE_SAMPLES)));
      if (!sample) continue;
      double d = norm(sub(*sample, p));
      if (!best || d < best->second) best = std::make_pair(i, d);
    }
    if (!best) return std::nullopt;
    double step = (last - first) / static_cast<double>(SPINE_SAMPLES);
    double centre = lerp(range, fraction_of(best->first, SPINE_SAMPLES));
    double low = centre - step, high = centre + step;
    if (domain == Domain::Bounded) {
      low = std::max(low, first);
      high = std::min(high, last);
    }
    const double ratio = (std::sqrt(5.0) - 1.0) / 2.0;
    for (size_t i = 0; i < PROJECTION_STEPS; ++i) {
      double left = high - ratio * (high - low);
      double right = low + ratio * (high - low);
      auto al = point(left), ar = point(right);
      if (!al || !ar) return std::nullopt;
      if (norm(sub(*al, p)) <= norm(sub(*ar, p))) high = right;
      else low = left;
    }
    return wrap(0.5 * (low + high));
  }

  double within(double t, std::array<double, 2> extent) const {
    if (domain == Domain::Periodic) t = extent[0] + rem_euclid(t - extent[0], range[1] - range[0]);
    return std::clamp(t, extent[0], extent[1]);
  }

  std::array<double, 2> widen(std::array<double, 2> e) const {
    double margin = (e[1] - e[0]) * EXTENT_MARGIN;
    switch (domain) {
      case Domain::Line:
        return {e[0] - margin, e[1] + margin};
      case Domain::Bounded:
        return {std::max(e[0] - margin, range[0]), std::min(e[1] + margin, range[1])};
      case Domain::Periodic: {
        double period = range[1] - range[0];
        if (e[1] - e[0] + 2.0 * margin >= period) return {e[0], e[0] + period};
        return {e[0] - margin, e[1] + margin};
      }
    }
    return e;
  }

  std::optional<std::array<double, 2>> extent(std::vector<double> sorted) const {
    std::sort(sorted.begin(), sorted.end());
    if (sorted.empty()) return std::nullopt;
    double low = sorted.front(), high = sorted.back();
    std::array<double, 2> e{low, high};
    if (domain == Domain::Periodic) {
      double period = range[1] - range[0];
      double g0 = low + period - high, g1 = high, g2 = low + period;
      for (size_t i = 0; i + 1 < sorted.size(); ++i)
        if (sorted[i + 1] - sorted[i] > g0) g0 = sorted[i + 1] - sorted[i], g1 = sorted[i], g2 = sorted[i + 1];
      if (g0 <= period * RING_GAP_SHARE) {
        e = {g2, g2 + period};
      } else {
        double end = g1 < g2 ? g1 + period : g1;
        e = {g2, end};
      }
    }
    if (!(e[0] < e[1])) return std::nullopt;
    return e;
  }
};

struct Section {
  V center;
  std::array<V, 2> contacts;
  std::array<Seed, 2> seeds;
  double deviation = 0;
};

struct Patch {
  std::array<double, 2> range;
  std::array<std::array<H, 3>, 4> poles;
  double error = 0;
};

std::optional<std::array<H, 3>> section_poles(const Section& s) {
  V first = sub(s.contacts[0], s.center), second = sub(s.contacts[1], s.center);
  double fl = norm(first), sl = norm(second);
  double angle = std::atan2(norm(cross(first, second)), dot(first, second));
  if (!(std::isfinite(angle) && angle <= kPi - MIN_SECTION_ANGLE)) return std::nullopt;
  double weight = std::cos(0.5 * angle);
  auto bisector = unit(add(scale(first, 1.0 / fl), scale(second, 1.0 / sl)));
  if (!bisector) return std::nullopt;
  double radius = 0.5 * (fl + sl);
  V middle = add(s.center, scale(*bisector, radius / weight));
  return std::array<H, 3>{homogeneous(s.contacts[0], 1.0), homogeneous(middle, weight),
                          homogeneous(s.contacts[1], 1.0)};
}

std::array<H, 4> interpolating_bezier(const std::array<H, 4>& v) {
  auto combine = [&](std::array<double, 4> w) {
    H pole{0, 0, 0, 0};
    for (size_t i = 0; i < 4; ++i)
      for (size_t k = 0; k < 4; ++k) pole[k] += w[i] * v[i][k];
    return pole;
  };
  return {v[0], combine({-5.0 / 6.0, 3.0, -1.5, 1.0 / 3.0}),
          combine({1.0 / 3.0, -1.5, 3.0, -5.0 / 6.0}), v[3]};
}

std::optional<V> patch_point(const std::array<std::array<H, 3>, 4>& poles, double along, double across) {
  double rest = 1.0 - along;
  double cubic[4] = {rest * rest * rest, 3.0 * rest * rest * along, 3.0 * rest * along * along,
                     along * along * along};
  double spare = 1.0 - across;
  double quad[3] = {spare * spare, 2.0 * spare * across, across * across};
  H sum{0, 0, 0, 0};
  for (size_t i = 0; i < 4; ++i)
    for (size_t j = 0; j < 3; ++j)
      for (size_t k = 0; k < 4; ++k) sum[k] += cubic[i] * quad[j] * poles[i][j][k];
  if (!(sum[3] > 0.0)) return std::nullopt;
  return V{sum[0] / sum[3], sum[1] / sum[3], sum[2] / sum[3]};
}

double arc_distance(V point, const Section& s) {
  V start = s.contacts[0], end = s.contacts[1];
  V first = sub(start, s.center), second = sub(end, s.center);
  double ends = std::min(norm(sub(point, start)), norm(sub(point, end)));
  auto normal = unit(cross(first, second));
  auto first_unit = unit(first);
  if (!normal || !first_unit) return ends;
  V offset = sub(point, s.center);
  double height = dot(offset, *normal);
  V planar = sub(offset, scale(*normal, height));
  double angle = std::atan2(norm(cross(first, second)), dot(first, second));
  double position = std::atan2(dot(cross(*first_unit, planar), *normal), dot(*first_unit, planar));
  if (!(position >= 0.0 && position <= angle)) return ends;
  double radius = 0.5 * (norm(first) + norm(second));
  return std::hypot(height, norm(planar) - radius);
}

std::optional<NurbsSurfaceGeometry> build_surface(const std::vector<Patch>& patches) {
  if (patches.empty()) return std::nullopt;
  NurbsSurfaceGeometry s;
  s.u_degree = 3;
  s.v_degree = 2;
  s.u_knots.assign(4, patches[0].range[0]);
  for (size_t index = 0; index < patches.size(); ++index) {
    const Patch& patch = patches[index];
    for (size_t row = index > 0 ? 1 : 0; row < 4; ++row) {
      std::vector<Vec3> points;
      std::vector<double> weights;
      for (const H& pole : patch.poles[row]) {
        if (pole[3] == 0.0 || !std::isfinite(pole[3])) return std::nullopt;
        points.push_back(Vec3{pole[0] / pole[3], pole[1] / pole[3], pole[2] / pole[3]});
        weights.push_back(pole[3]);
      }
      s.poles.push_back(std::move(points));
      s.weights.push_back(std::move(weights));
    }
    s.u_knots.insert(s.u_knots.end(), 3, patch.range[1]);
  }
  s.u_knots.push_back(s.u_knots.back());
  s.v_knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  // cadmpeg `NurbsSurface::new`: knot counts and non-decreasing knots.
  if (s.u_knots.size() != s.poles.size() + 4) return std::nullopt;
  for (size_t i = 0; i + 1 < s.u_knots.size(); ++i)
    if (!(s.u_knots[i] <= s.u_knots[i + 1])) return std::nullopt;
  return s;
}

class Solver {
 public:
  Solver(const BlendInputs& inputs, const Spine& spine, std::array<double, 2> extent)
      : inputs_(inputs), spine_(spine), extent_(extent) {}

  std::map<uint64_t, Section> sections;

  const Section* section(double t) {
    uint64_t key = std::bit_cast<uint64_t>(t);
    auto it = sections.find(key);
    if (it == sections.end()) {
      auto s = solve_section(t);
      if (!s) return nullptr;
      it = sections.emplace(key, *s).first;
    }
    return &it->second;
  }

  std::optional<Section> solve_section(double t) const {
    auto stored = spine_.point(t);
    auto tangent = this->tangent(t);
    if (!stored || !tangent) return std::nullopt;
    std::array<Seed, 2> seeds{};
    {
      const Section* nearest = nullptr;
      double best = 0;
      for (const auto& [key, s] : sections) {
        double d = std::fabs(std::bit_cast<double>(key) - t);
        if (!nearest || d < best) nearest = &s, best = d;
      }
      if (nearest) seeds = nearest->seeds;
    }
    double radius = inputs_.radius;
    V center = *stored;
    std::optional<Section> best;
    for (size_t step = 0; step < CENTER_STEPS; ++step) {
      std::array<V, 2> contacts{};
      std::array<double, 2> residuals{};
      for (size_t side = 0; side < 2; ++side) {
        auto f = foot(*inputs_.supports[side], center, seeds[side]);
        if (!f) return std::nullopt;
        contacts[side] = f->first;
        seeds[side] = f->second;
        residuals[side] = radius - norm(sub(center, f->first));
      }
      double deviation = std::max(std::fabs(residuals[0]), std::fabs(residuals[1]));
      if (!best || deviation < best->deviation) best = Section{center, contacts, seeds, deviation};
      if (deviation <= radius * CENTER_RELATIVE_TOLERANCE) break;
      auto first = unit(sub(center, contacts[0])), second = unit(sub(center, contacts[1]));
      if (!first || !second) return std::nullopt;
      double det = dot(*first, cross(*second, *tangent));
      if (std::fabs(det) <= kEpsilon || !std::isfinite(det)) return std::nullopt;
      V move = scale(add(scale(cross(*second, *tangent), residuals[0]),
                         scale(cross(*tangent, *first), residuals[1])),
                     1.0 / det);
      center = add(center, move);
    }
    if (!best) return std::nullopt;
    if (!(best->deviation <= radius * CONTACT_RELATIVE_TOLERANCE &&
          norm(sub(best->center, *stored)) <= radius))
      return std::nullopt;
    V first = sub(best->contacts[0], best->center), second = sub(best->contacts[1], best->center);
    if (dot(cross(first, second), *tangent) > 0.0) std::swap(best->contacts[0], best->contacts[1]);
    return best;
  }

  std::optional<V> tangent(double t) const {
    double step = (extent_[1] - extent_[0]) * TANGENT_STEP;
    auto before = spine_.point(std::max(t - step, extent_[0]));
    auto after = spine_.point(std::min(t + step, extent_[1]));
    if (!before || !after) return std::nullopt;
    return unit(sub(*after, *before));
  }

  std::vector<std::array<double, 2>> seed_ranges(std::array<double, 2> extent) const {
    std::vector<double> breaks{extent[0], extent[1]};
    std::optional<double> period;
    if (spine_.domain == Domain::Periodic) period = spine_.range[1] - spine_.range[0];
    for (double knot : spine_.knots)
      for (double shift : {-1.0, 0.0, 1.0, 2.0}) {
        double candidate = knot + (period ? shift * *period : 0.0);
        if (candidate > extent[0] && candidate < extent[1]) breaks.push_back(candidate);
      }
    std::sort(breaks.begin(), breaks.end());
    breaks.erase(std::unique(breaks.begin(), breaks.end()), breaks.end());
    size_t gaps = breaks.size() - 1;
    size_t pieces = std::max<size_t>((SEED_PATCHES + gaps - 1) / gaps, 1);
    std::vector<std::array<double, 2>> ranges;
    for (size_t i = 0; i + 1 < breaks.size(); ++i)
      for (size_t piece = 0; piece < pieces; ++piece)
        ranges.push_back({lerp({breaks[i], breaks[i + 1]}, fraction_of(piece, pieces)),
                          lerp({breaks[i], breaks[i + 1]}, fraction_of(piece + 1, pieces))});
    std::reverse(ranges.begin(), ranges.end());
    return ranges;
  }

  std::optional<Patch> fit(std::array<double, 2> range) {
    std::array<std::array<H, 3>, 4> nodes{};
    for (size_t i = 0; i < 4; ++i) {
      const Section* s = section(lerp(range, NODE_FRACTIONS[i]));
      if (!s) return std::nullopt;
      auto p = section_poles(*s);
      if (!p) return std::nullopt;
      nodes[i] = *p;
    }
    Patch patch;
    patch.range = range;
    for (size_t column = 0; column < 3; ++column) {
      auto bezier = interpolating_bezier({nodes[0][column], nodes[1][column], nodes[2][column],
                                          nodes[3][column]});
      for (size_t row = 0; row < 4; ++row) {
        if (bezier[row][3] <= 0.0) return std::nullopt;
        patch.poles[row][column] = bezier[row];
      }
    }
    double error = 0.0;
    for (double fraction : PROBE_U) {
      const Section* s = section(lerp(range, fraction));
      if (!s) return std::nullopt;
      for (double across : PROBE_V) {
        auto p = patch_point(patch.poles, fraction, across);
        if (!p) return std::nullopt;
        error = std::max(error, arc_distance(*p, *s));
      }
    }
    patch.error = error;
    return patch;
  }

 private:
  const BlendInputs& inputs_;
  const Spine& spine_;
  std::array<double, 2> extent_;
};

std::optional<BlendCache> solve_over(const BlendInputs& inputs, const Spine& spine,
                                     std::array<double, 2> extent) {
  Solver solver(inputs, spine, extent);
  auto pending = solver.seed_ranges(extent);
  std::vector<Patch> accepted;
  while (!pending.empty()) {
    auto range = pending.back();
    pending.pop_back();
    if (accepted.size() + pending.size() >= MAX_PATCHES) return std::nullopt;
    auto patch = solver.fit(range);
    if (!patch) return std::nullopt;
    if (patch->error <= inputs.tolerance) {
      accepted.push_back(std::move(*patch));
      continue;
    }
    double middle = lerp(range, 0.5);
    if (!(middle > range[0] && middle < range[1])) return std::nullopt;
    pending.push_back({middle, range[1]});
    pending.push_back({range[0], middle});
  }
  std::stable_sort(accepted.begin(), accepted.end(),
                   [](const Patch& a, const Patch& b) { return a.range[0] < b.range[0]; });
  double fit_error = 0.0;
  for (const auto& p : accepted) fit_error = std::max(fit_error, p.error);
  double deviation = 0.0;
  for (const auto& [_, s] : solver.sections) deviation = std::max(deviation, s.deviation);
  auto surface = build_surface(accepted);
  if (!surface) return std::nullopt;
  return BlendCache{std::move(*surface), fit_error + deviation};
}

double largest_miss(const BlendCache& cache, const Spine& spine, std::array<double, 2> extent,
                    const std::vector<Vec3>& vertices) {
  double largest = 0.0;
  for (const Vec3& vertex : vertices) {
    V point = to3(vertex);
    auto projected = spine.project(point);
    double along = projected ? spine.within(*projected, extent) : lerp(extent, 0.5);
    double nearest = std::numeric_limits<double>::infinity();
    std::array<Seed, 4> seeds{Seed{{along, VERTEX_SEEDS_V[0]}}, Seed{{along, VERTEX_SEEDS_V[1]}},
                              Seed{{along, VERTEX_SEEDS_V[2]}}, Seed{}};
    for (const Seed& seed : seeds) {
      auto q = nurbs_near(cache.surface, point, seed);
      if (!q) continue;
      if (auto f = ownership::nurbs_surface_point(cache.surface, (*q)[0], (*q)[1]))
        nearest = std::min(nearest, norm(sub(to3(*f), point)));
    }
    largest = std::max(largest, nearest);
  }
  return largest;
}

}  // namespace

std::optional<Vec3> blend_curve_point(const CurveGeometry& curve, double t) {
  auto p = curve_point(curve, t);
  if (!p) return std::nullopt;
  return from3(*p);
}

std::optional<std::pair<Vec3, std::optional<std::array<double, 2>>>> blend_support_foot(
    const SurfaceGeometry& support, Vec3 point, std::optional<std::array<double, 2>> seed) {
  auto f = foot(support, to3(point), seed);
  if (!f) return std::nullopt;
  return std::make_pair(from3(f->first), f->second);
}

std::vector<Vec3> blend_edge_samples(const CurveGeometry& curve,
                                     std::optional<std::array<double, 2>> range) {
  std::array<double, 2> r{};
  if (range) r = *range;
  else if (curve.kind == "circle" || curve.kind == "ellipse") r = {0.0, kTau};
  else return {};
  std::vector<Vec3> points;
  for (size_t i = 0; i <= EDGE_SAMPLES; ++i)
    if (auto p = curve_point(curve, lerp(r, fraction_of(i, EDGE_SAMPLES)))) points.push_back(from3(*p));
  return points;
}

std::optional<SolvedBlend> solve_blend(const BlendInputs& inputs) {
  if (!(std::isfinite(inputs.radius) && inputs.radius > 0.0) || !inputs.boundary ||
      inputs.boundary->empty() || !inputs.spine || !inputs.supports[0] || !inputs.supports[1])
    return std::nullopt;
  auto spine = Spine::of(*inputs.spine);
  if (!spine) return std::nullopt;
  std::vector<double> parameters;
  for (const Vec3& p : *inputs.boundary) {
    auto t = spine->project(to3(p));
    if (!t) return std::nullopt;
    parameters.push_back(*t);
  }
  auto extent = spine->extent(parameters);
  if (!extent) return std::nullopt;
  auto widened = spine->widen(*extent);
  for (auto stretch : {widened, *extent}) {
    if (auto cache = solve_over(inputs, *spine, stretch)) {
      static const std::vector<Vec3> kNone;
      double miss = largest_miss(*cache, *spine, stretch, inputs.vertices ? *inputs.vertices : kNone);
      bool holds = miss <= cache->fit_tolerance + VERTEX_ALLOWANCE;
      return SolvedBlend{std::move(*cache), holds};
    }
    if (widened == *extent) break;
  }
  return std::nullopt;
}

std::optional<double> blend_vertex_miss(const BlendCache& cache, const CurveGeometry& spine_curve,
                                        const std::vector<Vec3>& vertices) {
  auto spine = Spine::of(spine_curve);
  if (!spine) return std::nullopt;
  std::vector<double> parameters;
  for (const Vec3& v : vertices) {
    auto t = spine->project(to3(v));
    if (!t) return std::nullopt;
    parameters.push_back(*t);
  }
  auto extent = spine->extent(parameters);
  if (!extent) return std::nullopt;
  return largest_miss(cache, *spine, *extent, vertices);
}

}  // namespace swglb::brep

// ===========================================================================
// Port of cadmpeg-codec-sldprt `brep/blend_fit.rs` and the blend sides of
// `brep/index.rs` (`support_pair_wedge`, `blend_side`,
// `nested_blend_geometry`) at cadmpeg 5af95421 (Apache-2.0).
//
// Analytic feet are solved in closed form rather than by cadmpeg's
// Gauss-Newton refinement of the surface's own inverse; the natural normal
// keeps one orientation per surface, which is all the solves read.
// ===========================================================================
namespace swglb::brep {

namespace {

constexpr double BLEND_FIT_TOLERANCE_MM = 1.0e-5;
constexpr double BLEND_FIT_LIMIT_MM = 1.0e-3;
constexpr double DOMAIN_MARGIN_FRACTION = 0.05;
constexpr size_t SEED_SPANS = 4;
constexpr size_t MAX_SPLIT_DEPTH = 12;
constexpr size_t MAX_SPANS = size_t{1} << 10;
constexpr double DIFFERENCE_STEP_FRACTION = 1.0e-4;
constexpr double CHECK_FRACTIONS[3] = {0.25, 0.5, 0.75};
constexpr double CONTACT_RADIUS_RELATIVE = 1.0e-6;
constexpr double MIN_HALF_TURN_GAP = 1.0e-6;
constexpr size_t FIT_SPINE_SAMPLES = 64;
constexpr size_t SPINE_SECTION_STEPS = 40;
constexpr size_t LOCAL_SPINE_SAMPLES = 16;
constexpr double CENTER_STEP_TOLERANCE_MM = 1.0e-11;
constexpr size_t MAX_CENTER_STEPS = 32;
constexpr size_t MAX_CENTER_SOLVES = size_t{1} << 20;
constexpr double MIN_CENTER_TRANSVERSALITY = 1.0e-8;
constexpr double COINCIDENT_OFFSETS_MM = 1.0e-7;
constexpr double EDGE_STEP_TOLERANCE_MM = 1.0e-11;
constexpr size_t MAX_EDGE_STEPS = 32;
constexpr double MIN_EDGE_TRANSVERSALITY = 1.0e-8;
constexpr double LOCAL_SEARCH_FRACTION = 0.1;
constexpr double SPINE_CACHE_TOLERANCE_MM = 1.0e-7;
constexpr size_t MAX_SPINE_CACHE_DEPTH = 10;
constexpr double TRACK_DIFFERENCE_STEP = 1.0e-7;
constexpr size_t MAX_NEAREST_STEPS = 16;
constexpr double NEAREST_STEP_TOLERANCE_MM = 1.0e-10;
constexpr size_t PAIR_STATIONS = 5;
constexpr size_t MAX_BLEND_NESTING = 3;
constexpr double ROUNDING_RELATIVE = 16.0 * kEpsilon;
constexpr double kInf = std::numeric_limits<double>::infinity();

using Sec = std::array<H, 3>;  ///< Homogeneous section controls.
using Node = std::tuple<double, Sec, Sec>;
using SpineNode = std::tuple<double, V, V>;

double midpoint(double a, double b) { return (a + b) / 2.0; }
double rounding_scale(V p) {
  return ROUNDING_RELATIVE * std::max({std::fabs(p[0]), std::fabs(p[1]), std::fabs(p[2])});
}
double distance(V a, V b) { return norm(sub(a, b)); }

bool analytic(const SurfaceGeometry& s) {
  return s.kind == "plane" || s.kind == "cylinder" || s.kind == "cone" || s.kind == "sphere" ||
         s.kind == "torus";
}

/// The nearest point of an analytic surface and the unit natural normal
/// there (cadmpeg `evaluation::analytic_surface_foot`).
std::optional<std::pair<V, V>> analytic_foot(const SurfaceGeometry& g, V p) {
  if (!analytic(g)) return std::nullopt;
  auto f = foot(g, p, std::nullopt);
  if (!f) return std::nullopt;
  V at = f->first, o = to3(g.origin), axis = to3(g.normal);
  std::optional<V> normal;
  if (g.kind == "plane") {
    normal = unit(axis);
  } else if (g.kind == "cylinder") {
    V d = sub(at, o);
    normal = unit(sub(d, scale(axis, dot(d, axis))));
  } else if (g.kind == "sphere") {
    normal = unit(sub(at, o));
  } else if (g.kind == "torus") {
    V d = sub(at, o);
    auto r = unit(sub(d, scale(axis, dot(d, axis))));
    if (!r) return std::nullopt;
    normal = unit(sub(at, add(o, scale(*r, g.radius))));
  } else if (g.kind == "cone") {
    V d = sub(at, o);
    double h = dot(d, axis);
    auto r = unit(sub(d, scale(axis, h)));
    if (!r) return std::nullopt;
    normal = unit(sub(*r, scale(axis, std::tan(g.half_angle))));
  }
  if (!normal) return std::nullopt;
  return std::make_pair(at, *normal);
}

std::optional<std::array<double, 2>> nurbs_domain(const NurbsCurveGeometry& n) {
  size_t p = n.degree, count = n.poles.size();
  if (count >= n.knots.size() || p >= n.knots.size()) return std::nullopt;
  double lo = n.knots[p], hi = n.knots[count];
  if (!(lo < hi)) return std::nullopt;
  return std::array<double, 2>{lo, hi};
}

bool closed_nurbs(const NurbsCurveGeometry& n) {
  if (n.periodic) return true;
  if (!n.weights.empty() || n.poles.empty()) return false;
  const Vec3 &a = n.poles.front(), &b = n.poles.back();
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

/// Distinct knots (consecutive duplicates removed) within the domain.
std::vector<double> domain_knots(const NurbsCurveGeometry& n, std::array<double, 2> d) {
  std::vector<double> k = n.knots;
  k.erase(std::unique(k.begin(), k.end()), k.end());
  std::erase_if(k, [&](double x) { return !(x >= d[0] && x <= d[1]); });
  return k;
}

struct Polyline {
  std::vector<double> parameters;
  std::vector<V> points;
  static std::optional<Polyline> of(const CurveGeometry& c) {
    if (c.kind != "nurbs" || !c.nurbs) return std::nullopt;
    const auto& n = *c.nurbs;
    if (!n.weights.empty()) return std::nullopt;
    if (n.degree != 1 || n.periodic || n.knots.size() != n.poles.size() + 2) return std::nullopt;
    Polyline p;
    p.parameters.assign(n.knots.begin() + 1, n.knots.begin() + 1 + n.poles.size());
    for (size_t i = 0; i + 1 < p.parameters.size(); ++i)
      if (p.parameters[i] > p.parameters[i + 1]) return std::nullopt;
    for (const Vec3& q : n.poles) p.points.push_back(to3(q));
    return p;
  }
  std::optional<V> point(double s) const {
    if (parameters.empty()) return std::nullopt;
    double lower = parameters.front(), upper = parameters.back();
    if (!(s >= lower && s <= upper)) return std::nullopt;
    size_t pp = std::partition_point(parameters.begin(), parameters.end(),
                                     [&](double x) { return x <= s; }) -
                parameters.begin();
    size_t index = std::clamp<size_t>(pp, 1, std::max<size_t>(parameters.size(), 2) - 1) - 1;
    if (index + 1 >= parameters.size()) return std::nullopt;
    double from = parameters[index], to = parameters[index + 1];
    V start = points[index], end = points[index + 1];
    double span = to - from;
    if (!(span > 0.0 && std::isfinite(span))) return end;
    double f = (s - from) / span;
    return V{start[0] + (end[0] - start[0]) * f, start[1] + (end[1] - start[1]) * f,
             start[2] + (end[2] - start[2]) * f};
  }
};

std::optional<V> hermite_spine_point(const std::vector<SpineNode>& nodes, double s) {
  if (nodes.empty()) return std::nullopt;
  double first = std::get<0>(nodes.front()), last = std::get<0>(nodes.back());
  if (!(s >= first && s <= last)) return std::nullopt;
  size_t pp = std::partition_point(nodes.begin(), nodes.end(),
                                   [&](const SpineNode& n) { return std::get<0>(n) <= s; }) -
              nodes.begin();
  if (pp == 0) return std::nullopt;
  size_t index = pp - 1;
  const SpineNode& start = nodes[index];
  const SpineNode& end = index + 1 < nodes.size() ? nodes[index + 1] : start;
  double span = std::get<0>(end) - std::get<0>(start);
  if (span <= 0.0) return std::get<1>(start);
  double f = (s - std::get<0>(start)) / span;
  double h00 = 2 * f * f * f - 3 * f * f + 1, h10 = f * f * f - 2 * f * f + f,
         h01 = -2 * f * f * f + 3 * f * f, h11 = f * f * f - f * f;
  V out;
  for (int k = 0; k < 3; ++k)
    out[k] = h00 * std::get<1>(start)[k] + h10 * span * std::get<2>(start)[k] +
             h01 * std::get<1>(end)[k] + h11 * span * std::get<2>(end)[k];
  return out;
}

struct ExactSpine {
  std::vector<std::vector<SpineNode>> runs;
  std::optional<V> point(double s) const {
    for (const auto& run : runs)
      if (auto p = hermite_spine_point(run, s)) return p;
    return std::nullopt;
  }
};

class Geometry;

struct Wedge {
  std::array<const SurfaceGeometry*, 2> members{};
  std::array<double, 2> outside{};
};

struct Side {
  enum Kind { Surface, Blend, WedgeSide } kind = Surface;
  const SurfaceGeometry* surface = nullptr;
  std::shared_ptr<Geometry> blend;
  Wedge wedge;
};

struct Fit {
  NurbsSurfaceGeometry surface;
  double fit_tolerance = 0;
};

class Geometry {
 public:
  Geometry(const CurveGeometry& spine, std::array<std::vector<Side>, 2> sides, double radius)
      : spine_(spine), sides_(std::move(sides)), radius_(radius), polyline_(Polyline::of(spine)) {}

  /// cadmpeg `BlendGeometry::new`.
  static std::shared_ptr<Geometry> make(const CurveGeometry& spine,
                                        std::array<std::vector<Side>, 2> sides,
                                        double signed_radius) {
    auto g = std::make_shared<Geometry>(spine, std::move(sides), std::fabs(signed_radius));
    if (!(g->analytic() && g->radius_ > 0.0 && std::isfinite(g->radius_))) return nullptr;
    return g;
  }

  void reset_seed() {
    last_parameter_.reset();
    center_solves_ = 0;
  }

  bool analytic() const {
    for (const auto& side : sides_) {
      if (side.empty()) return false;
      for (const auto& c : side)
        if (c.kind == Side::Surface && !(c.surface && brep::analytic(*c.surface))) return false;
    }
    return true;
  }

  std::optional<std::pair<V, V>> pipe_foot(V point) {
    auto near = nearest_on(point, [&](double s) { return center(s); });
    if (!near) return std::nullopt;
    V offset = sub(point, std::get<1>(*near));
    double length = norm(offset);
    if (!(length > 0.0 && std::isfinite(length))) return std::nullopt;
    V normal = scale(offset, 1.0 / length);
    return std::make_pair(add(std::get<1>(*near), scale(normal, radius_)), normal);
  }

  std::optional<size_t> pair_member(const std::vector<Vec3>& vertices) {
    auto r = range(vertices);
    if (!r) return std::nullopt;
    auto [low, high] = *r;
    std::array<bool, 2> qualifies{true, true};
    for (size_t index = 0; index < PAIR_STATIONS; ++index) {
      double s = low + (high - low) * static_cast<double>(index) /
                           static_cast<double>(PAIR_STATIONS - 1);
      auto center = spine_point(s);
      if (!center) return std::nullopt;
      std::array<double, 2> gaps{};
      for (size_t k = 0; k < 2; ++k) {
        auto f = side_normal(sides_[k][0], *center);
        if (!f) return std::nullopt;
        gaps[k] = distance(f->first, *center) - radius_;
      }
      for (size_t member = 0; member < 2; ++member) {
        size_t other = 1 - member;
        if (!(std::fabs(gaps[member]) <= radius_ / 4.0 && std::fabs(gaps[other]) > radius_ / 2.0))
          qualifies[member] = false;
      }
    }
    if (qualifies[0] && !qualifies[1]) return 0;
    if (!qualifies[0] && qualifies[1]) return 1;
    return std::nullopt;
  }

  std::optional<Fit> fit(const std::vector<Vec3>& vertices) {
    if (!(radius_ > 0.0 && std::isfinite(radius_))) return std::nullopt;
    if (!analytic()) return std::nullopt;
    auto r = range(vertices);
    if (!r) return std::nullopt;
    double margin = ((*r)[1] - (*r)[0]) * DOMAIN_MARGIN_FRACTION;
    auto [low, high] = clamp({(*r)[0] - margin, (*r)[1] + margin});
    if (!(high > low)) return std::nullopt;
    std::vector<double> breaks{low};
    if (polyline_)
      for (double knot : polyline_->parameters)
        if (low < knot && knot < high) breaks.push_back(knot);
    breaks.push_back(high);
    size_t pieces = breaks.size() - 1;
    size_t seeds = std::max<size_t>((SEED_SPANS + pieces - 1) / pieces, 1);
    std::vector<Node> fitted;
    double fit_tolerance = BLEND_FIT_TOLERANCE_MM;
    for (size_t i = 0; i + 1 < breaks.size(); ++i) {
      double from = breaks[i], to = breaks[i + 1];
      std::vector<Node> nodes;
      for (size_t index = 0; index <= seeds; ++index) {
        double s = from + (to - from) * static_cast<double>(index) / static_cast<double>(seeds);
        auto sec = section(s);
        if (!sec) return std::nullopt;
        auto der = derivative(s, {from, to});
        if (!der) return std::nullopt;
        nodes.emplace_back(s, *sec, *der);
      }
      fitted.push_back(nodes[0]);
      for (size_t k = 0; k + 1 < nodes.size(); ++k) {
        if (!split(nodes[k], nodes[k + 1], {from, to}, 0, fitted, fit_tolerance)) return std::nullopt;
        if (fitted.size() > MAX_SPANS) return std::nullopt;
      }
    }
    auto surface = build(fitted);
    if (!surface) return std::nullopt;
    return Fit{std::move(*surface), fit_tolerance};
  }

  std::optional<std::array<double, 2>> range(const std::vector<Vec3>& vertices) {
    std::vector<double> parameters;
    std::optional<double> reference;
    for (const Vec3& v : vertices) {
      auto p = spine_parameter(to3(v), reference);
      if (!p) return std::nullopt;
      if (!reference) reference = *p;
      parameters.push_back(*p);
    }
    double low = kInf, high = -kInf;
    for (double p : parameters) low = std::fmin(low, p), high = std::fmax(high, p);
    if (!(std::isfinite(low) && std::isfinite(high))) return std::nullopt;
    return std::array<double, 2>{low, high};
  }

 private:
  const CurveGeometry& spine_;
  std::array<std::vector<Side>, 2> sides_;
  double radius_;
  size_t center_solves_ = 0;
  std::optional<double> last_parameter_;
  std::vector<std::pair<std::pair<size_t, size_t>, std::shared_ptr<ExactSpine>>> exact_spines_;
  std::optional<Polyline> polyline_;

  const NurbsCurveGeometry* nurbs() const {
    return spine_.kind == "nurbs" && spine_.nurbs ? &*spine_.nurbs : nullptr;
  }

  std::optional<size_t> member(size_t side, V point) {
    const auto& candidates = sides_[side];
    if (candidates.size() == 1) return 0;
    std::optional<std::pair<double, size_t>> best;
    for (size_t index = 0; index < candidates.size(); ++index) {
      auto f = side_normal(candidates[index], point);
      if (!f) continue;
      double gap = std::fabs(distance(f->first, point) - radius_);
      if (!best || gap < best->first) best = std::make_pair(gap, index);
    }
    if (!best) return std::nullopt;
    return best->second;
  }

  std::optional<std::pair<V, V>> side_foot(size_t side, V point) {
    std::optional<std::pair<double, std::pair<V, V>>> best;
    for (const auto& c : sides_[side]) {
      auto f = side_normal(c, point);
      if (!f) continue;
      double gap = std::fabs(distance(f->first, point) - radius_);
      if (!best || gap < best->first) best = std::make_pair(gap, *f);
    }
    if (!best) return std::nullopt;
    return best->second;
  }

  std::optional<double> nearest_spine_parameter(V point) {
    std::optional<double> seed = last_parameter_;
    std::optional<double> parameter;
    const NurbsCurveGeometry* n = nurbs();
    if (seed && n) {
      auto d = nurbs_domain(*n);
      if (!d) return std::nullopt;
      auto [lower, upper] = *d;
      double window = (upper - lower) * LOCAL_SEARCH_FRACTION;
      auto found = golden({std::max(*seed - window, lower), std::min(*seed + window, upper)}, point);
      if (!found) return std::nullopt;
      double beyond0 = *seed + window - upper, beyond1 = lower + window - *seed;
      std::optional<std::array<double, 2>> wrapped;
      if (closed_nurbs(*n)) {
        if (beyond0 > 0.0) wrapped = std::array<double, 2>{lower, lower + beyond0};
        else if (beyond1 > 0.0) wrapped = std::array<double, 2>{upper - beyond1, upper};
      }
      if (wrapped) {
        auto across = golden(*wrapped, point);
        if (!across) return std::nullopt;
        auto pa = spine_point(*across);
        if (!pa) return std::nullopt;
        auto pf = spine_point(*found);
        if (!pf) return std::nullopt;
        parameter = distance(*pa, point) < distance(*pf, point) ? *across : *found;
      } else {
        parameter = found;
      }
    } else {
      parameter = spine_parameter(point, seed);
      if (!parameter) return std::nullopt;
    }
    last_parameter_ = parameter;
    return parameter;
  }

  std::optional<double> golden(std::array<double, 2> r, V point) {
    auto [a, b] = r;
    double step = (b - a) / static_cast<double>(LOCAL_SPINE_SAMPLES);
    std::pair<double, double> best{kInf, a};
    for (size_t index = 0; index <= LOCAL_SPINE_SAMPLES; ++index) {
      double s = a + step * static_cast<double>(index);
      auto p = spine_point(s);
      if (!p) return std::nullopt;
      double gap = distance(*p, point);
      if (gap < best.first) best = {gap, s};
    }
    return refine_nearest({std::max(best.second - step, a), std::min(best.second + step, b)}, point);
  }

  std::optional<double> refine_nearest(std::array<double, 2> r, V point) {
    auto [a, b] = r;
    const double ratio = (std::sqrt(5.0) - 1.0) / 2.0;
    for (size_t i = 0; i < SPINE_SECTION_STEPS; ++i) {
      double left = b - ratio * (b - a), right = a + ratio * (b - a);
      auto pl = spine_point(left);
      if (!pl) return std::nullopt;
      auto pr = spine_point(right);
      if (!pr) return std::nullopt;
      if (distance(*pl, point) <= distance(*pr, point)) b = right;
      else a = left;
    }
    return midpoint(a, b);
  }

  template <typename F>
  std::optional<std::tuple<double, V, V>> nearest_on(V point, F&& curve) {
    auto start = nearest_spine_parameter(point);
    if (!start) return std::nullopt;
    double s = *start;
    double lower = -kInf, upper = kInf;
    if (const NurbsCurveGeometry* n = nurbs()) {
      auto d = nurbs_domain(*n);
      if (!d) return std::nullopt;
      lower = (*d)[0], upper = (*d)[1];
    }
    std::optional<std::pair<double, double>> previous;
    const double step = TRACK_DIFFERENCE_STEP;
    for (size_t i = 0; i < MAX_NEAREST_STEPS; ++i) {
      auto at = curve(s);
      if (!at) return std::nullopt;
      double ahead = std::min(s + step, upper), behind = std::max(s - step, lower);
      auto after = curve(ahead);
      if (!after) return std::nullopt;
      auto before = curve(behind);
      if (!before) return std::nullopt;
      V derivative = sub(*after, *before);
      double width = ahead - behind;
      if (!(width > 0.0 && std::isfinite(width))) return std::nullopt;
      derivative = V{derivative[0] / width, derivative[1] / width, derivative[2] / width};
      double speed = dot(derivative, derivative);
      if (!(speed > 0.0 && std::isfinite(speed))) return std::nullopt;
      V offset = sub(point, *at);
      double projected = dot(derivative, offset) / speed;
      bool outward = (s >= upper && projected > 0.0) || (s <= lower && projected < 0.0);
      if (outward || std::fabs(projected * std::sqrt(speed)) <= NEAREST_STEP_TOLERANCE_MM) {
        last_parameter_ = s;
        return std::make_tuple(s, *at, derivative);
      }
      double delta;
      if (previous && std::signbit(projected) != std::signbit(previous->second)) {
        double secant = projected * (s - previous->first) / (previous->second - projected);
        delta = std::isfinite(secant) ? secant : projected;
      } else {
        double curving = 0.0;
        if (s - step >= lower && s + step <= upper)
          curving = (dot(sub(*after, *at), offset) + dot(sub(*before, *at), offset)) / (step * step);
        double denominator = speed - curving;
        double newton = dot(derivative, offset) / denominator;
        delta = denominator > 0.0 && std::isfinite(newton) ? newton : projected;
      }
      previous = std::make_pair(s, projected);
      s = std::clamp(s + delta, lower, upper);
    }
    return std::nullopt;
  }

  std::optional<std::array<size_t, 2>> members_at(double s) {
    auto point = spine_point(s);
    if (!point) return std::nullopt;
    auto a = member(0, *point);
    if (!a) return std::nullopt;
    auto b = member(1, *point);
    if (!b) return std::nullopt;
    return std::array<size_t, 2>{*a, *b};
  }

  std::optional<std::pair<V, V>> side_normal(const Side& c, V point) {
    switch (c.kind) {
      case Side::Surface:
        return analytic_foot(*c.surface, point);
      case Side::Blend:
        return c.blend->pipe_foot(point);
      case Side::WedgeSide:
        return wedge_foot(c.wedge, point);
    }
    return std::nullopt;
  }

  static std::optional<std::pair<V, V>> wedge_foot(const Wedge& w, V point) {
    for (size_t slot = 0; slot < 2; ++slot) {
      auto f = analytic_foot(*w.members[slot], point);
      if (!f) return std::nullopt;
      if (w.outside[slot] * dot(sub(point, f->first), f->second) <= 0.0) continue;
      size_t other = 1 - slot;
      auto o = analytic_foot(*w.members[other], f->first);
      if (!o) return std::nullopt;
      if (w.outside[other] * dot(sub(f->first, o->first), o->second) <= 0.0) return f;
    }
    auto edge = edge_point(w, point);
    if (!edge) return std::nullopt;
    V offset = sub(point, *edge);
    double length = norm(offset);
    if (!(length > 0.0 && std::isfinite(length))) return std::nullopt;
    return std::make_pair(*edge, scale(offset, 1.0 / length));
  }

  static std::optional<V> edge_point(const Wedge& w, V point) {
    V current = point;
    for (size_t i = 0; i < MAX_EDGE_STEPS; ++i) {
      auto fa = analytic_foot(*w.members[0], current);
      if (!fa) return std::nullopt;
      auto fb = analytic_foot(*w.members[1], current);
      if (!fb) return std::nullopt;
      auto [a, na] = *fa;
      auto [b, nb] = *fb;
      V c = cross(na, nb);
      if (dot(c, c) < MIN_EDGE_TRANSVERSALITY) return std::nullopt;
      double aa = dot(na, na), ab = dot(na, nb), bb = dot(nb, nb);
      double ra = dot(na, sub(a, point)), rb = dot(nb, sub(b, point));
      double det = aa * bb - ab * ab;
      double la = (ra * bb - rb * ab) / det, lb = (rb * aa - ra * ab) / det;
      V next = add(point, add(scale(na, la), scale(nb, lb)));
      if (!finite3(next)) return std::nullopt;
      double step = distance(next, current);
      current = next;
      if (step <= std::max(EDGE_STEP_TOLERANCE_MM, rounding_scale(current))) return current;
    }
    return std::nullopt;
  }

  std::array<double, 2> clamp(std::array<double, 2> r) const {
    if (const NurbsCurveGeometry* n = nurbs())
      if (auto d = nurbs_domain(*n)) return {std::max(r[0], (*d)[0]), std::min(r[1], (*d)[1])};
    return r;
  }

  std::optional<double> spine_parameter(V point, std::optional<double> reference) {
    auto near = [&](double angle) {
      if (!reference) return angle;
      return angle + std::round((*reference - angle) / kTau) * kTau;
    };
    const CurveGeometry& g = spine_;
    if (g.kind == "line") return dot(sub(point, to3(g.origin)), to3(g.direction));
    if (g.kind == "circle" || g.kind == "ellipse") {
      V offset = sub(point, to3(g.center));
      V ref = to3(g.ref_direction), bin = cross(to3(g.axis), ref);
      double x = dot(offset, ref), y = dot(offset, bin);
      if (g.kind == "ellipse") x /= g.radius, y /= g.minor_radius;
      return near(std::atan2(y, x));
    }
    if (const NurbsCurveGeometry* n = nurbs()) {
      auto d = nurbs_domain(*n);
      if (!d) return std::nullopt;
      auto [lower, upper] = *d;
      std::vector<double> breaks = domain_knots(*n, *d);
      double best_gap = kInf, a = lower, b = upper;
      for (size_t i = 0; i + 1 < breaks.size(); ++i) {
        double step = (breaks[i + 1] - breaks[i]) / static_cast<double>(FIT_SPINE_SAMPLES);
        for (size_t index = 0; index <= FIT_SPINE_SAMPLES; ++index) {
          double s = breaks[i] + step * static_cast<double>(index);
          auto p = spine_point(s);
          if (!p) return std::nullopt;
          double gap = distance(*p, point);
          if (gap < best_gap) best_gap = gap, a = std::max(s - step, lower), b = std::min(s + step, upper);
        }
      }
      return refine_nearest({a, b}, point);
    }
    return std::nullopt;
  }

  std::optional<V> center(double s) {
    auto m = members_at(s);
    if (!m) return std::nullopt;
    return center_with(s, *m);
  }

  std::optional<V> center_with(double s, std::array<size_t, 2> members) {
    if (auto spine = exact_spine(members))
      if (auto p = spine->point(s)) return p;
    const Side* a = candidate(0, members[0]);
    const Side* b = candidate(1, members[1]);
    if (!a || !b) return std::nullopt;
    return solve_center(s, {a, b});
  }

  const Side* candidate(size_t side, size_t index) const {
    return index < sides_[side].size() ? &sides_[side][index] : nullptr;
  }

  std::shared_ptr<ExactSpine> exact_spine(std::array<size_t, 2> members) {
    const NurbsCurveGeometry* n = nurbs();
    if (!n || n->degree != 1) return nullptr;
    std::pair<size_t, size_t> key{members[0], members[1]};
    for (const auto& [k, spine] : exact_spines_)
      if (k == key) return spine;
    std::shared_ptr<ExactSpine> solved;
    const Side* a = candidate(0, members[0]);
    const Side* b = candidate(1, members[1]);
    if (a && b)
      if (auto s = solve_spine(*n, {a, b})) solved = std::make_shared<ExactSpine>(std::move(*s));
    exact_spines_.emplace_back(key, solved);
    return solved;
  }

  std::optional<ExactSpine> solve_spine(const NurbsCurveGeometry& n, std::array<const Side*, 2> members) {
    auto d = nurbs_domain(n);
    if (!d) return std::nullopt;
    auto [lower, upper] = *d;
    std::vector<double> knots = domain_knots(n, *d);
    auto node = [&](double s) -> std::optional<SpineNode> {
      double step = TRACK_DIFFERENCE_STEP;
      double before = std::max(s - step, lower), after = std::min(s + step, upper);
      auto ca = solve_center(after, members);
      if (!ca) return std::nullopt;
      auto cb = solve_center(before, members);
      if (!cb) return std::nullopt;
      V difference = sub(*ca, *cb);
      double span = after - before;
      auto c = solve_center(s, members);
      if (!c) return std::nullopt;
      return SpineNode{s, *c, V{difference[0] / span, difference[1] / span, difference[2] / span}};
    };
    std::vector<std::vector<SpineNode>> runs;
    std::vector<SpineNode> run;
    for (double knot : knots) {
      auto end = node(knot);
      if (!end) {
        if (run.size() > 1) runs.push_back(std::move(run));
        run.clear();
        continue;
      }
      if (run.empty() || !extend_spine(run, *end, members, node)) {
        if (run.size() > 1) runs.push_back(std::move(run));
        run = {*end};
      }
    }
    if (run.size() > 1) runs.push_back(std::move(run));
    if (runs.empty()) return std::nullopt;
    return ExactSpine{std::move(runs)};
  }

  template <typename N>
  bool extend_spine(std::vector<SpineNode>& run, SpineNode end, std::array<const Side*, 2> members,
                    N& node) {
    std::vector<std::pair<SpineNode, size_t>> stack{{end, 0}};
    while (!stack.empty()) {
      auto [e, depth] = stack.back();
      stack.pop_back();
      if (run.empty()) return false;
      SpineNode start = run.back();
      double middle = midpoint(std::get<0>(start), std::get<0>(e));
      auto exact = solve_center(middle, members);
      if (!exact) return false;
      auto h = hermite_spine_point({start, e}, middle);
      if (!h) return false;
      if (distance(*h, *exact) <= SPINE_CACHE_TOLERANCE_MM) {
        run.push_back(e);
      } else if (depth >= MAX_SPINE_CACHE_DEPTH) {
        return false;
      } else {
        stack.emplace_back(e, depth + 1);
        auto m = node(middle);
        if (!m) return false;
        stack.emplace_back(*m, depth + 1);
      }
    }
    return true;
  }

  std::optional<V> solve_center(double s, std::array<const Side*, 2> members) {
    if (++center_solves_ > MAX_CENTER_SOLVES) return std::nullopt;
    auto start = spine_point(s);
    if (!start) return std::nullopt;
    V current = *start;
    for (size_t i = 0; i < MAX_CENTER_STEPS; ++i) {
      std::array<std::pair<V, V>, 2> planes{};
      for (size_t slot = 0; slot < 2; ++slot) {
        auto f = side_normal(*members[slot], current);
        if (!f) return std::nullopt;
        auto [ft, normal] = *f;
        double side = dot(sub(current, ft), normal) < 0.0 ? -1.0 : 1.0;
        planes[slot] = {add(ft, scale(normal, side * radius_)), normal};
      }
      auto [a, na] = planes[0];
      auto [b, nb] = planes[1];
      V c = cross(na, nb);
      double aa = dot(na, na), ab = dot(na, nb), bb = dot(nb, nb);
      double ra = dot(na, sub(a, current)), rb = dot(nb, sub(b, current));
      V next;
      if (dot(c, c) < MIN_CENTER_TRANSVERSALITY) {
        double ua = ra / std::sqrt(aa), ub = rb / std::sqrt(bb);
        double flip = ab < 0.0 ? -1.0 : 1.0;
        double gap = ua - flip * ub;
        if (!(std::fabs(gap) <= COINCIDENT_OFFSETS_MM && std::isfinite(gap))) return std::nullopt;
        if (std::max(std::fabs(ua), std::fabs(ub)) <= COINCIDENT_OFFSETS_MM) return current;
        double along = midpoint(ua, flip * ub) / std::sqrt(aa);
        next = add(current, scale(na, along));
      } else {
        double det = aa * bb - ab * ab;
        double la = (ra * bb - rb * ab) / det, lb = (rb * aa - ra * ab) / det;
        next = add(current, add(scale(na, la), scale(nb, lb)));
      }
      if (!finite3(next)) return std::nullopt;
      double step = distance(next, current);
      current = next;
      if (step <= std::max(CENTER_STEP_TOLERANCE_MM, rounding_scale(current))) return current;
    }
    return std::nullopt;
  }

  std::optional<V> spine_point(double s) const {
    if (polyline_) return polyline_->point(s);
    return curve_point(spine_, s);
  }

  std::optional<Sec> section(double s) {
    auto c = center(s);
    if (!c) return std::nullopt;
    auto first = contact(0, *c);
    if (!first) return std::nullopt;
    auto second = contact(1, *c);
    if (!second) return std::nullopt;
    V a = sub(*first, *c), b = sub(*second, *c);
    double cosine = std::clamp(dot(a, b) / (radius_ * radius_), -1.0, 1.0);
    double angle = std::acos(cosine);
    if (kPi - angle < MIN_HALF_TURN_GAP) return std::nullopt;
    V middle = add(*c, scale(add(a, b), 1.0 / (1.0 + cosine)));
    double weight = std::cos(angle / 2.0);
    return Sec{H{(*first)[0], (*first)[1], (*first)[2], 1.0},
               H{middle[0] * weight, middle[1] * weight, middle[2] * weight, weight},
               H{(*second)[0], (*second)[1], (*second)[2], 1.0}};
  }

  std::optional<V> contact(size_t side, V center) {
    auto f = side_foot(side, center);
    if (!f) return std::nullopt;
    if (std::fabs(distance(f->first, center) - radius_) <= CONTACT_RADIUS_RELATIVE * radius_)
      return f->first;
    return std::nullopt;
  }

  std::optional<Sec> derivative(double s, std::array<double, 2> d) {
    double step = (d[1] - d[0]) * DIFFERENCE_STEP_FRACTION;
    double before = std::max(s - step, d[0]), after = std::min(s + step, d[1]);
    auto a = section(before);
    if (!a) return std::nullopt;
    auto b = section(after);
    if (!b) return std::nullopt;
    double span = after - before;
    Sec out{};
    for (size_t r = 0; r < 3; ++r)
      for (size_t k = 0; k < 4; ++k) out[r][k] = ((*b)[r][k] - (*a)[r][k]) / span;
    return out;
  }

  static Sec hermite(const Node& start, const Node& end, double f) {
    double span = std::get<0>(end) - std::get<0>(start);
    double h00 = 2 * f * f * f - 3 * f * f + 1, h10 = f * f * f - 2 * f * f + f,
           h01 = -2 * f * f * f + 3 * f * f, h11 = f * f * f - f * f;
    Sec out{};
    for (size_t r = 0; r < 3; ++r)
      for (size_t k = 0; k < 4; ++k)
        out[r][k] = h00 * std::get<1>(start)[r][k] + h10 * span * std::get<2>(start)[r][k] +
                    h01 * std::get<1>(end)[r][k] + h11 * span * std::get<2>(end)[r][k];
    return out;
  }

  static V arc_point(const Sec& s, double t) {
    double basis[3] = {(1 - t) * (1 - t), 2 * t * (1 - t), t * t};
    H sum{0, 0, 0, 0};
    for (size_t r = 0; r < 3; ++r)
      for (size_t k = 0; k < 4; ++k) sum[k] += s[r][k] * basis[r];
    return V{sum[0] / sum[3], sum[1] / sum[3], sum[2] / sum[3]};
  }

  bool split(const Node& start, const Node& end, std::array<double, 2> d, size_t depth,
             std::vector<Node>& out, double& largest) {
    double s = midpoint(std::get<0>(start), std::get<0>(end));
    auto exact = section(s);
    if (!exact) return false;
    Sec fitted = hermite(start, end, 0.5);
    double departure = 0.0;
    for (double t : CHECK_FRACTIONS)
      departure = std::fmax(departure, distance(arc_point(fitted, t), arc_point(*exact, t)));
    if (departure <= BLEND_FIT_TOLERANCE_MM ||
        (depth >= MAX_SPLIT_DEPTH && departure <= BLEND_FIT_LIMIT_MM)) {
      largest = std::fmax(largest, departure);
      out.push_back(end);
      return true;
    }
    if (depth >= MAX_SPLIT_DEPTH || out.size() >= MAX_SPANS) return false;
    auto der = derivative(s, d);
    if (!der) return false;
    Node middle{s, *exact, *der};
    if (!split(start, middle, d, depth + 1, out, largest)) return false;
    return split(middle, end, d, depth + 1, out, largest);
  }

  static std::optional<bool> normal_toward_center(const std::vector<Node>& nodes) {
    auto point = [](const H& row) { return V{row[0] / row[3], row[1] / row[3], row[2] / row[3]}; };
    auto width = [&](const Sec& s) { return distance(point(s[2]), point(s[0])); };
    if (nodes.empty()) return std::nullopt;
    // Rust `max_by` keeps the last of equal maxima.
    const Node* widest = &nodes[0];
    for (const Node& n : nodes)
      if (!(width(std::get<1>(n)) < width(std::get<1>(*widest)))) widest = &n;
    const Sec& section = std::get<1>(*widest);
    const Sec& der = std::get<2>(*widest);
    V first = point(section[0]), middle = point(section[1]), last = point(section[2]);
    double weight = section[0][3];
    V motion{(der[0][0] - first[0] * der[0][3]) / weight, (der[0][1] - first[1] * der[0][3]) / weight,
             (der[0][2] - first[2] * der[0][3]) / weight};
    V chord{midpoint(first[0], last[0]), midpoint(first[1], last[1]), midpoint(first[2], last[2])};
    double side = dot(cross(motion, sub(middle, first)), sub(middle, chord));
    if (!(std::isfinite(side) && side != 0.0)) return std::nullopt;
    return side < 0.0;
  }

  static std::optional<NurbsSurfaceGeometry> build(const std::vector<Node>& nodes) {
    if (!normal_toward_center(nodes)) return std::nullopt;
    if (nodes.empty()) return std::nullopt;
    const Node& first = nodes.front();
    const Node& last = nodes.back();
    auto mid = [](const Sec& s) {
      return V{midpoint(s[0][0], s[2][0]), midpoint(s[0][1], s[2][1]), midpoint(s[0][2], s[2][2])};
    };
    double length = 0.0;
    for (size_t i = 0; i + 1 < nodes.size(); ++i)
      length += distance(mid(std::get<1>(nodes[i + 1])), mid(std::get<1>(nodes[i])));
    double extent = std::get<0>(last) - std::get<0>(first);
    double speed = length / extent;
    if (!(speed > 0.0 && std::isfinite(speed))) speed = 1.0;
    auto knot = [&](double s) { return (s - std::get<0>(first)) * speed; };
    NurbsSurfaceGeometry out;
    out.u_degree = 3;
    out.v_degree = 2;
    out.u_knots.assign(4, knot(std::get<0>(first)));
    std::vector<Sec> rows{std::get<1>(first)};
    for (size_t i = 0; i + 1 < nodes.size(); ++i) {
      const Node& p0 = nodes[i];
      const Node& p1 = nodes[i + 1];
      double span = std::get<0>(p1) - std::get<0>(p0);
      if (span == 0.0) continue;
      Sec ahead = std::get<1>(p0), behind = std::get<1>(p1);
      for (size_t r = 0; r < 3; ++r)
        for (size_t k = 0; k < 4; ++k) {
          ahead[r][k] += std::get<2>(p0)[r][k] * span / 3.0;
          behind[r][k] -= std::get<2>(p1)[r][k] * span / 3.0;
        }
      rows.push_back(ahead);
      rows.push_back(behind);
      rows.push_back(std::get<1>(p1));
      if (std::get<0>(p1) < std::get<0>(last)) out.u_knots.insert(out.u_knots.end(), 3, knot(std::get<0>(p1)));
    }
    out.u_knots.insert(out.u_knots.end(), 4, knot(std::get<0>(last)));
    for (const Sec& row : rows) {
      std::vector<Vec3> poles;
      std::vector<double> weights;
      for (const H& c : row) {
        double w = c[3];
        if (!(w > 0.0 && std::isfinite(w))) return std::nullopt;
        poles.push_back(Vec3{c[0] / w, c[1] / w, c[2] / w});
        weights.push_back(w);
      }
      out.poles.push_back(std::move(poles));
      out.weights.push_back(std::move(weights));
    }
    out.v_knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    if (out.u_knots.size() != out.poles.size() + 4) return std::nullopt;
    for (size_t i = 0; i + 1 < out.u_knots.size(); ++i)
      if (!(out.u_knots[i] <= out.u_knots[i + 1])) return std::nullopt;
    return out;
  }
};

/// Stored-spine points over `interval` (cadmpeg `spine_samples`).
std::vector<V> spine_samples(const CurveGeometry& spine, std::optional<std::array<double, 2>> interval) {
  std::optional<std::vector<double>> knots;
  if (spine.kind == "nurbs" && spine.nurbs) {
    auto d = nurbs_domain(*spine.nurbs);
    if (!d) return {};
    knots = domain_knots(*spine.nurbs, *d);
  }
  std::vector<double> parameters;
  if (interval) {
    auto [low, high] = *interval;
    parameters = {low, midpoint(low, high), high};
    if (knots)
      for (double k : *knots)
        if (k >= low && k <= high) parameters.push_back(k);
  } else if (knots) {
    parameters = *knots;
    for (size_t i = 0; i + 1 < knots->size(); ++i) parameters.push_back(midpoint((*knots)[i], (*knots)[i + 1]));
  } else {
    parameters = {0.0};
  }
  std::vector<V> points;
  for (double p : parameters)
    if (auto q = curve_point(spine, p)) points.push_back(*q);
  return points;
}

/// cadmpeg `Wedge::new`.
std::optional<Wedge> make_wedge(const CurveGeometry& spine, std::array<const SurfaceGeometry*, 2> members,
                                const std::vector<Vec3>& vertices) {
  if (!members[0] || !members[1] || !analytic(*members[0]) || !analytic(*members[1])) return std::nullopt;
  std::optional<std::array<double, 2>> interval;
  if (!vertices.empty()) {
    Geometry fitter(spine, {}, 0.0);
    interval = fitter.range(vertices);
  }
  auto samples = spine_samples(spine, interval);
  Wedge w;
  w.members = members;
  for (size_t slot = 0; slot < 2; ++slot) {
    double farthest = 0.0;
    for (const V& sample : samples) {
      auto f = analytic_foot(*members[slot], sample);
      if (!f) continue;
      double d = dot(sub(sample, f->first), f->second);
      if (std::fabs(d) > std::fabs(farthest)) farthest = d;
    }
    if (!(std::isfinite(farthest) && farthest != 0.0)) return std::nullopt;
    w.outside[slot] = farthest > 0 ? 1.0 : -1.0;
  }
  return w;
}

using Memo = std::map<std::pair<uint32_t, size_t>, std::shared_ptr<Geometry>>;

std::shared_ptr<Geometry> nested_geometry(BlendCarriers& c, uint32_t index, size_t depth,
                                          const std::vector<Vec3>& vertices, Memo& memo);

std::optional<Side> nested_side(BlendCarriers& c, uint32_t index, size_t depth,
                                const std::vector<Vec3>& vertices, Memo& memo) {
  if (const SurfaceGeometry* s = c.surface(index)) return Side{Side::Surface, s, nullptr, {}};
  auto g = nested_geometry(c, index, depth + 1, vertices, memo);
  if (!g) return std::nullopt;
  return Side{Side::Blend, nullptr, g, {}};
}

std::shared_ptr<Geometry> solve_geometry(BlendCarriers& c, uint32_t index, size_t depth,
                                         const std::vector<Vec3>& vertices, Memo& memo) {
  auto blend = c.blend(index);
  if (!blend) return nullptr;
  const CurveGeometry* spine = c.curve(blend->spine);
  if (!spine) return nullptr;
  std::array<std::vector<Side>, 2> sides;
  for (size_t slot = 0; slot < 2; ++slot) {
    uint32_t reference = blend->supports[slot];
    std::vector<uint32_t> members;
    if (auto pair = c.pair(reference)) {
      const SurfaceGeometry* a = c.surface((*pair)[0]);
      const SurfaceGeometry* b = c.surface((*pair)[1]);
      if (a && b) {
        auto w = make_wedge(*spine, {a, b}, vertices);
        if (!w) return nullptr;
        sides[slot].push_back(Side{Side::WedgeSide, nullptr, nullptr, *w});
        continue;
      }
      members = {(*pair)[0], (*pair)[1]};
    } else {
      members = {reference};
    }
    for (uint32_t m : members) {
      auto side = nested_side(c, m, depth, vertices, memo);
      if (!side) return nullptr;
      sides[slot].push_back(std::move(*side));
    }
  }
  return Geometry::make(*spine, std::move(sides), blend->signed_radius);
}

std::shared_ptr<Geometry> nested_geometry(BlendCarriers& c, uint32_t index, size_t depth,
                                          const std::vector<Vec3>& vertices, Memo& memo) {
  if (depth > MAX_BLEND_NESTING) return nullptr;
  if (auto b = c.bound(index)) index = *b;
  auto key = std::make_pair(index, depth);
  if (auto it = memo.find(key); it != memo.end()) return it->second;
  auto g = solve_geometry(c, index, depth, vertices, memo);
  memo[key] = g;
  return g;
}

}  // namespace

std::optional<size_t> blend_pair_member(const CurveGeometry& spine,
                                        std::array<const SurfaceGeometry*, 2> members,
                                        double signed_radius, const std::vector<Vec3>& vertices) {
  if (!members[0] || !members[1]) return std::nullopt;
  std::array<std::vector<Side>, 2> sides;
  sides[0].push_back(Side{Side::Surface, members[0], nullptr, {}});
  sides[1].push_back(Side{Side::Surface, members[1], nullptr, {}});
  Geometry fitter(spine, std::move(sides), std::fabs(signed_radius));
  return fitter.pair_member(vertices);
}

std::optional<BlendCache> fit_blend(BlendCarriers& carriers, const CurveGeometry& spine,
                                    std::array<uint32_t, 2> references,
                                    std::array<uint32_t, 2> resolved, double signed_radius,
                                    const std::vector<Vec3>& vertices) {
  Memo memo;
  std::array<std::optional<Side>, 2> sides;
  for (size_t slot = 0; slot < 2; ++slot) {
    if (auto pair = carriers.pair(references[slot])) {
      const SurfaceGeometry* a = carriers.surface((*pair)[0]);
      const SurfaceGeometry* b = carriers.surface((*pair)[1]);
      if (a && b)
        if (auto w = make_wedge(spine, {a, b}, vertices)) {
          sides[slot] = Side{Side::WedgeSide, nullptr, nullptr, *w};
          continue;
        }
    }
    // cadmpeg `blend_side`: forget the seeds of the memo's blends first.
    for (auto& [_, g] : memo)
      if (g) g->reset_seed();
    sides[slot] = nested_side(carriers, resolved[slot], 0, vertices, memo);
  }
  if (!sides[0] || !sides[1]) return std::nullopt;
  std::array<std::vector<Side>, 2> both;
  both[0].push_back(std::move(*sides[0]));
  both[1].push_back(std::move(*sides[1]));
  Geometry fitter(spine, std::move(both), std::fabs(signed_radius));
  auto fitted = fitter.fit(vertices);
  if (!fitted) return std::nullopt;
  return BlendCache{std::move(fitted->surface), fitted->fit_tolerance};
}

}  // namespace swglb::brep
