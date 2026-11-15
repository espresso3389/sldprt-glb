// SPDX-License-Identifier: Apache-2.0
// Geometry predicates of the tessellation owner stage. Ported from
// cadmpeg-codec-sldprt tessellation.rs (polygon tests, analytic surface
// residuals and normals, pole box, surface measures) and cadmpeg-ir
// (`math::sum`, `math::planar`, `math::solve::least_squares_step`,
// `eval::nurbs_surface_parameter_near_point`, `eval::nurbs_surface_partials`)
// at cadmpeg 5af95421 (Apache-2.0).
//
// cadmpeg evaluates some sums exactly (`ExactSignedSum`); this port uses an
// expansion sum (exact signs, sums rounded from the expansion) and the
// platform's hypot/atan2/trig instead of cadmpeg's portable ones, so results
// may differ in the last bits.
#include <algorithm>
#include <bit>
#include <cfloat>
#include <limits>

#include "ownership_internal.h"

namespace swglb::ownership {

bool charge_product(WorkBudget& work, size_t left, size_t right) {
  size_t units = 0;
  if (__builtin_mul_overflow(left, right, &units)) {
    work.exhaust();
    return false;
  }
  return work.charge_by(units);
}

bool total_less(double a, double b) {
  auto key = [](double value) {
    int64_t bits = std::bit_cast<int64_t>(value);
    bits ^= static_cast<int64_t>(static_cast<uint64_t>(bits >> 63) >> 1);
    return bits;
  };
  return key(a) < key(b);
}

namespace {

inline void two_sum(double a, double b, double& sum, double& error) {
  sum = a + b;
  double bv = sum - a;
  error = (a - (sum - bv)) + (b - bv);
}

/// cadmpeg-ir `math::sum::ExactSignedSum`: an exact sum of products, kept as a
/// nonoverlapping expansion (Shewchuk). `finish` is none for an exact zero.
class ExactSum {
 public:
  void add(double x) {
    if (!std::isfinite(x)) {
      nonfinite_ = true;
      plain_ += x;
      return;
    }
    plain_ += x;
    if (x == 0.0) return;
    double q = x;
    size_t out = 0;
    for (size_t i = 0; i < count_; ++i) {
      double s, e;
      two_sum(q, parts_[i], s, e);
      q = s;
      if (e != 0.0) parts_[out++] = e;
    }
    if (q != 0.0) parts_[out++] = q;
    count_ = out;
    if (count_ >= parts_.size() - 1) compress();
  }
  void add_product(double a, double b) {
    double p = a * b;
    if (!std::isfinite(p)) {
      add(p);
      return;
    }
    add(p);
    add(std::fma(a, b, -p));
  }
  void add_factors(double a, double b, double c) {
    double p = a * b;
    double e = std::isfinite(p) ? std::fma(a, b, -p) : 0.0;
    add_product(p, c);
    add_product(e, c);
  }
  /// The rounded sum, or none for an exact zero.
  std::optional<double> finish() const {
    if (nonfinite_) return plain_;
    if (count_ == 0) return std::nullopt;
    double sum = 0;
    for (size_t i = 0; i < count_; ++i) sum += parts_[i];
    return sum;
  }
  /// -1, 0 or 1 (exact when every term was finite).
  int sign() const {
    if (nonfinite_) return plain_ > 0 ? 1 : plain_ < 0 ? -1 : 0;
    if (count_ == 0) return 0;
    return parts_[count_ - 1] > 0 ? 1 : -1;
  }

 private:
  void compress() {
    // Renormalise: sum from the smallest part; keep the result and its error.
    double sum = 0, error = 0;
    for (size_t i = 0; i < count_; ++i) {
      double s, e;
      two_sum(sum, parts_[i], s, e);
      sum = s;
      error += e;
    }
    count_ = 0;
    double s, e;
    two_sum(sum, error, s, e);
    if (e != 0.0) parts_[count_++] = e;
    if (s != 0.0) parts_[count_++] = s;
  }
  std::array<double, 64> parts_{};
  size_t count_ = 0;
  bool nonfinite_ = false;
  double plain_ = 0;
};

/// cadmpeg-ir `math::sum::fast_dot`: the plain sum, or none where it may
/// have cancelled or rounded below the normal range.
template <size_t N>
std::optional<double> fast_dot(const std::array<double, N>& coefficients,
                               const std::array<double, N>& components,
                               const std::array<double, N>& products) {
  for (double product : products)
    if (!std::isfinite(product)) return std::nullopt;
  for (size_t i = 0; i < N; ++i)
    if (coefficients[i] != 0.0 && components[i] != 0.0 && std::fabs(products[i]) < DBL_MIN)
      return std::nullopt;
  double scale_value = 0;
  for (double product : products) scale_value = std::max(scale_value, std::fabs(product));
  if (scale_value == 0.0) return 0.0;
  double sum = 0;
  for (double product : products) sum += product;
  if (!std::isfinite(sum)) return std::nullopt;
  if (std::fabs(sum) / scale_value < F64_EPSILON) return std::nullopt;
  return sum;
}

/// cadmpeg-ir `math::sum::finite_dot`.
template <size_t N>
double finite_dot(const std::array<double, N>& coefficients,
                  const std::array<double, N>& components) {
  std::array<double, N> products;
  for (size_t i = 0; i < N; ++i) products[i] = coefficients[i] * components[i];
  auto plain = [&] {
    double sum = products[0];
    for (size_t i = 1; i < N; ++i) sum += products[i];
    return sum;
  };
  for (size_t i = 0; i < N; ++i)
    if (!std::isfinite(coefficients[i]) || !std::isfinite(components[i])) return plain();
  if (auto value = fast_dot(coefficients, components, products)) return *value;
  ExactSum sum;
  for (size_t i = 0; i < N; ++i) sum.add_product(coefficients[i], components[i]);
  auto value = sum.finish();
  if (!value) return 0.0;
  return std::isfinite(*value) ? *value : plain();
}

}  // namespace

double dot(Vec3 a, Vec3 b) {
  return finite_dot<3>({a.x, a.y, a.z}, {b.x, b.y, b.z});
}

std::optional<Vec3> unit(Vec3 a) {
  double length = norm(a);
  if (!(length > F64_EPSILON)) return std::nullopt;
  if (!is_finite(a) || !std::isfinite(length)) return std::nullopt;
  Vec3 result{a.x / length, a.y / length, a.z / length};
  if (!is_finite(result)) return std::nullopt;
  return result;
}

// ---------------------------------------------------------------------------
// cadmpeg-ir math::planar
// ---------------------------------------------------------------------------
namespace {

/// `orientation`: -1, 0, 1, or 2 for nonfinite input.
int orientation(Point2 a, Point2 b, Point2 p) {
  if (!a.finite() || !b.finite() || !p.finite()) return 2;
  double left = (b.u - a.u) * (p.v - a.v);
  double right = (b.v - a.v) * (p.u - a.u);
  double determinant = left - right;
  if (std::isfinite(determinant) &&
      std::fabs(determinant) > 4.0 * F64_EPSILON * (std::fabs(left) + std::fabs(right)))
    return determinant > 0 ? 1 : determinant < 0 ? -1 : 0;
  ExactSum sum;
  sum.add_product(b.u, p.v);
  sum.add_product(-b.u, a.v);
  sum.add_product(-a.u, p.v);
  sum.add_product(-b.v, p.u);
  sum.add_product(b.v, a.u);
  sum.add_product(a.v, p.u);
  return sum.sign();
}

std::pair<Point2, double> scaled_displacement(Point2 start, Point2 end, double minimum_scale) {
  Point2 delta{end.u - start.u, end.v - start.v};
  if (delta.finite()) {
    double s = std::max({std::fabs(delta.u), std::fabs(delta.v), minimum_scale});
    if (s == 0.0) return {delta, s};
    return {Point2{delta.u / s, delta.v / s}, s};
  }
  double s = std::max({std::fabs(start.u), std::fabs(start.v), std::fabs(end.u), std::fabs(end.v),
                       minimum_scale});
  return {Point2{end.u / s - start.u / s, end.v / s - start.v / s}, s};
}

ExactSum line_offset_determinant(Point2 start, Point2 end, Point2 point) {
  ExactSum d;
  d.add_product(end.u, start.v);
  d.add_product(-end.u, point.v);
  d.add_product(start.u, point.v);
  d.add_product(-end.v, start.u);
  d.add_product(end.v, point.u);
  d.add_product(-start.v, point.u);
  return d;
}

std::optional<double> finite_line_projection_parameter(Point2 start, Point2 end, Point2 point) {
  Point2 delta{end.u - start.u, end.v - start.v};
  Point2 relative{point.u - start.u, point.v - start.v};
  auto denominator = fast_dot<2>({delta.u, delta.v}, {delta.u, delta.v},
                                 {delta.u * delta.u, delta.v * delta.v});
  auto numerator = fast_dot<2>({relative.u, relative.v}, {delta.u, delta.v},
                               {relative.u * delta.u, relative.v * delta.v});
  if (denominator && numerator && *denominator > 0.0) {
    double parameter = *numerator / *denominator;
    if (std::isfinite(parameter)) return parameter;
  }
  ExactSum num, den;
  for (auto [s, e, p] : {std::array<double, 3>{start.u, end.u, point.u},
                         std::array<double, 3>{start.v, end.v, point.v}}) {
    num.add_product(p, e);
    num.add_product(-p, s);
    num.add_product(-s, e);
    num.add_product(s, s);
    den.add_product(s, s);
    den.add_product(e, e);
    den.add_factors(-2.0, s, e);
  }
  auto d = den.finish();
  if (!d) return std::nullopt;
  auto n = num.finish();
  if (!n) return 0.0;
  double q = *n / *d;
  if (!std::isfinite(q)) return std::nullopt;
  return q;
}

}  // namespace

std::optional<double> polygon_area_twice(std::span<const Point2> points) {
  for (const Point2& p : points)
    if (!p.finite()) return std::nullopt;
  ExactSum area;
  for (size_t i = 0; i < points.size(); ++i) {
    const Point2& first = points[i];
    const Point2& second = points[(i + 1) % points.size()];
    area.add_product(first.u, second.v);
    area.add_product(-first.v, second.u);
  }
  auto value = area.finish();
  if (!value) return 0.0;
  if (!std::isfinite(*value)) return std::nullopt;
  return *value;
}

double point_segment_distance(Point2 point, Point2 start, Point2 end) {
  auto dist = [](Point2 a, Point2 b) { return std::hypot(a.u - b.u, a.v - b.v); };
  double first_distance = dist(point, start);
  double last_distance = dist(point, end);
  if (last_distance < first_distance) std::swap(start, end);
  auto parameter = finite_line_projection_parameter(start, end, point);
  if (!parameter) return std::min(first_distance, last_distance);
  if (*parameter <= 0.0) return dist(point, start);
  if (*parameter >= 1.0) return dist(point, end);
  auto [direction, s] = scaled_displacement(start, end, 0.0);
  ExactSum length_sum;
  length_sum.add_product(std::hypot(direction.u, direction.v), s);
  auto length = length_sum.finish();
  if (!length) return std::numeric_limits<double>::quiet_NaN();
  auto determinant = line_offset_determinant(start, end, point).finish();
  if (!determinant) return 0.0;
  double distance_value = *determinant / *length;
  if (!std::isfinite(distance_value)) return std::numeric_limits<double>::quiet_NaN();
  return std::fabs(distance_value);
}

bool segments_intersect(Point2 a, Point2 b, Point2 c, Point2 d, double tolerance) {
  if (!std::isfinite(tolerance) || tolerance < 0.0) return false;
  if (!a.finite() || !b.finite() || !c.finite() || !d.finite()) return false;
  auto bounds_overlap = [tolerance](double a1, double b1, double c1, double d1) {
    return std::min(a1, b1) <= std::max(c1, d1) + tolerance &&
           std::min(c1, d1) <= std::max(a1, b1) + tolerance;
  };
  if (!bounds_overlap(a.u, b.u, c.u, d.u) || !bounds_overlap(a.v, b.v, c.v, d.v)) return false;
  if (point_segment_distance(a, c, d) <= tolerance || point_segment_distance(b, c, d) <= tolerance ||
      point_segment_distance(c, a, b) <= tolerance || point_segment_distance(d, a, b) <= tolerance)
    return true;
  auto opposite = [](int first, int second) {
    return (first == -1 && second == 1) || (first == 1 && second == -1);
  };
  return opposite(orientation(a, b, c), orientation(a, b, d)) &&
         opposite(orientation(c, d, a), orientation(c, d, b));
}

// ---------------------------------------------------------------------------
// Surfaces
// ---------------------------------------------------------------------------
PoleBox pole_box(const NurbsSurfaceGeometry& nurbs) {
  PoleBox box;
  box.low.fill(std::numeric_limits<double>::infinity());
  box.high.fill(-std::numeric_limits<double>::infinity());
  bool rational = !nurbs.weights.empty();
  for (size_t i = 0; i < nurbs.poles.size(); ++i) {
    for (size_t j = 0; j < nurbs.poles[i].size(); ++j) {
      if (rational) {
        if (i >= nurbs.weights.size() || j >= nurbs.weights[i].size() ||
            !(nurbs.weights[i][j] > 0.0))
          return PoleBox{};
      }
      const Vec3& p = nurbs.poles[i][j];
      std::array<double, 3> values{p.x, p.y, p.z};
      for (size_t axis = 0; axis < 3; ++axis) {
        box.low[axis] = std::min(box.low[axis], values[axis]);
        box.high[axis] = std::max(box.high[axis], values[axis]);
      }
    }
  }
  box.valid = true;
  return box;
}

std::optional<SolvedSurface> solved_surface(const SurfaceGeometry& geometry) {
  SolvedSurface s;
  const std::string& kind = geometry.kind;
  if (kind.empty() || kind == "procedural") return std::nullopt;
  if (kind == "plane") s.kind = SurfaceKind::Plane;
  else if (kind == "cylinder") s.kind = SurfaceKind::Cylinder;
  else if (kind == "cone") s.kind = SurfaceKind::Cone;
  else if (kind == "sphere") s.kind = SurfaceKind::Sphere;
  else if (kind == "torus") s.kind = SurfaceKind::Torus;
  else if (kind == "nurbs") s.kind = SurfaceKind::Nurbs;
  else s.kind = SurfaceKind::Unknown;
  s.origin = geometry.origin;
  s.axis = geometry.normal;
  s.reference = geometry.u_axis;
  s.radius = geometry.radius;
  s.minor_radius = geometry.minor_radius;
  s.half_angle = geometry.half_angle;
  s.ratio = geometry.ratio;
  if (s.kind == SurfaceKind::Nurbs && geometry.nurbs) {
    s.nurbs = &*geometry.nurbs;
    s.box = pole_box(*geometry.nurbs);
  }
  return s;
}

std::pair<Vec3, Vec3> perpendicular_basis(Vec3 axis, Vec3 reference) {
  Vec3 r = divided(sub(reference, scale(axis, dot(reference, axis))));
  return {r, divided(cross(axis, r))};
}

std::optional<Vec3> analytic_surface_normal(const SolvedSurface& surface, Vec3 point) {
  switch (surface.kind) {
    case SurfaceKind::Plane:
      return surface.axis;
    case SurfaceKind::Cylinder: {
      Vec3 delta = sub(point, surface.origin);
      Vec3 radial = sub(delta, scale(surface.axis, dot(delta, surface.axis)));
      return unit(radial);
    }
    case SurfaceKind::Sphere:
      return unit(sub(point, surface.origin));
    case SurfaceKind::Torus: {
      Vec3 delta = sub(point, surface.origin);
      double axial = dot(delta, surface.axis);
      Vec3 radial = sub(delta, scale(surface.axis, axial));
      auto radial_unit = unit(radial);
      if (!radial_unit) return std::nullopt;
      return unit(add(scale(*radial_unit, norm(radial) - surface.radius),
                      scale(surface.axis, axial)));
    }
    case SurfaceKind::Cone: {
      auto [reference, transverse] = perpendicular_basis(surface.axis, surface.reference);
      double ratio = surface.ratio;
      double slope = std::tan(surface.half_angle);
      if (!std::isfinite(slope)) return std::nullopt;
      Vec3 delta = sub(point, surface.origin);
      double axial = dot(delta, surface.axis);
      double major = dot(delta, reference);
      double minor = dot(delta, transverse);
      double elliptical_radius = std::hypot(major, minor / ratio);
      if (elliptical_radius <= F64_EPSILON) return std::nullopt;
      double local_radius = surface.radius + axial * slope;
      double signum = std::signbit(local_radius) ? -1.0 : 1.0;
      if (std::isnan(local_radius)) signum = local_radius;
      return unit(sub(add(scale(reference, major / elliptical_radius),
                          scale(transverse, minor / (ratio * ratio * elliptical_radius))),
                      scale(surface.axis, slope * signum)));
    }
    default:
      return std::nullopt;
  }
}

std::optional<double> analytic_surface_residual(const SolvedSurface& surface, Vec3 point) {
  std::optional<double> result;
  switch (surface.kind) {
    case SurfaceKind::Plane:
      result = std::fabs(dot(sub(point, surface.origin), surface.axis));
      break;
    case SurfaceKind::Cylinder: {
      Vec3 delta = sub(point, surface.origin);
      Vec3 radial = sub(delta, scale(surface.axis, dot(delta, surface.axis)));
      result = std::fabs(norm(radial) - surface.radius);
      break;
    }
    case SurfaceKind::Sphere:
      result = std::fabs(norm(sub(point, surface.origin)) - surface.radius);
      break;
    case SurfaceKind::Torus: {
      Vec3 delta = sub(point, surface.origin);
      double axial = dot(delta, surface.axis);
      Vec3 radial = sub(delta, scale(surface.axis, axial));
      double r = norm(radial) - surface.radius;
      result = std::fabs(std::sqrt(r * r + axial * axial) - surface.minor_radius);
      break;
    }
    case SurfaceKind::Cone: {
      auto [reference, transverse] = perpendicular_basis(surface.axis, surface.reference);
      double slope = std::tan(surface.half_angle);
      if (!std::isfinite(slope)) return std::nullopt;
      Vec3 delta = sub(point, surface.origin);
      double axial = dot(delta, surface.axis);
      double major = dot(delta, reference);
      double minor = dot(delta, transverse);
      double local_radius = surface.radius + axial * slope;
      double elliptical_radius = std::hypot(major, minor / surface.ratio);
      result = std::fabs(elliptical_radius - std::fabs(local_radius));
      break;
    }
    default:
      return std::nullopt;
  }
  if (!std::isfinite(*result)) return std::nullopt;
  return result;
}

// ---------------------------------------------------------------------------
// NURBS (cadmpeg-ir eval)
// ---------------------------------------------------------------------------
namespace {

struct NurbsShape {
  size_t u_degree, v_degree, u_count, v_count;
  bool rational;
};

std::optional<NurbsShape> nurbs_shape(const NurbsSurfaceGeometry& s) {
  NurbsShape shape{s.u_degree, s.v_degree, s.poles.size(),
                   s.poles.empty() ? 0 : s.poles[0].size(), !s.weights.empty()};
  for (const auto& row : s.poles)
    if (row.size() != shape.v_count) return std::nullopt;
  if (shape.rational) {
    if (s.weights.size() != shape.u_count) return std::nullopt;
    for (const auto& row : s.weights)
      if (row.size() != shape.v_count) return std::nullopt;
  }
  return shape;
}

/// cadmpeg-ir `eval::basis::bspline_span`.
std::optional<size_t> bspline_span(const std::vector<double>& knots, size_t degree, size_t count,
                                   double t) {
  if (count <= degree || knots.size() < count + degree + 1) return std::nullopt;
  if (t >= knots[count]) return count - 1;
  if (t <= knots[degree]) return degree;
  if (count - degree == 1) return degree;
  size_t lo = degree, hi = count;
  while (lo < hi) {
    size_t mid = lo + (hi - lo) / 2;
    if (t < knots[mid]) hi = mid;
    else if (t >= knots[mid + 1]) lo = mid + 1;
    else return mid;
  }
  return lo;
}

/// cadmpeg-ir `math::wrap_parameter` / `eval::periodic_parameter`.
std::optional<double> periodic_parameter(const std::vector<double>& knots, size_t degree,
                                         size_t count, bool periodic, double t) {
  if (!std::isfinite(t)) return std::nullopt;
  if (degree >= knots.size() || count >= knots.size()) return std::nullopt;
  double start = knots[degree], end = knots[count];
  if (!periodic || (t >= start && t <= end)) return t;
  if (start >= end) return std::nullopt;
  if (t >= start && t < end) return t;
  if (t == end) return start;
  double period = end - start;
  double wrapped = start + rem_euclid(t - start, period);
  if (wrapped >= end) wrapped = start;
  return wrapped;
}

/// Cox-de Boor basis values of `degree` at span (Piegl-Tiller A2.2).
void basis_values(const std::vector<double>& knots, size_t degree, size_t span, double t,
                  double* n, double* left, double* right) {
  n[0] = 1.0;
  for (size_t j = 1; j <= degree; ++j) {
    left[j] = t - knots[span + 1 - j];
    right[j] = knots[span + j] - t;
    double saved = 0.0;
    for (size_t r = 0; r < j; ++r) {
      double temp = n[r] / (right[r + 1] + left[j - r]);
      n[r] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    n[j] = saved;
  }
}

/// Basis values and first derivatives at `span`.
void basis_with_derivatives(const std::vector<double>& knots, size_t degree, size_t span, double t,
                            double* n, double* dn, double* lower, double* left, double* right) {
  basis_values(knots, degree, span, t, n, left, right);
  if (!dn) return;
  if (degree == 0) {
    dn[0] = 0.0;
    return;
  }
  basis_values(knots, degree - 1, span, t, lower, left, right);
  double p = static_cast<double>(degree);
  for (size_t k = 0; k <= degree; ++k) {
    size_t i = span - degree + k;
    double value = 0.0;
    if (k >= 1) {
      double width = knots[i + degree] - knots[i];
      if (width != 0.0) value += lower[k - 1] / width;
    }
    if (k + 1 <= degree) {
      double width = knots[i + degree + 1] - knots[i + 1];
      if (width != 0.0) value -= lower[k] / width;
    }
    dn[k] = p * value;
  }
}

struct NurbsScratch {
  std::vector<double> nu, dnu, nv, dnv, lower, left, right;
  void reserve(size_t degree) {
    size_t n = degree + 2;
    if (nu.size() < n) {
      for (auto* v : {&nu, &dnu, &nv, &dnv, &lower, &left, &right}) v->assign(n, 0.0);
    }
  }
};

/// The point and (when `partials`) first partials of `s` at (u, v).
bool evaluate_nurbs(const NurbsSurfaceGeometry& s, double u_at, double v_at, bool partials,
                    SurfacePartials& out) {
  auto shape = nurbs_shape(s);
  if (!shape) return false;
  auto u = periodic_parameter(s.u_knots, shape->u_degree, shape->u_count, s.u_periodic, u_at);
  auto v = periodic_parameter(s.v_knots, shape->v_degree, shape->v_count, s.v_periodic, v_at);
  if (!u || !v) return false;
  auto u_span = bspline_span(s.u_knots, shape->u_degree, shape->u_count, *u);
  auto v_span = bspline_span(s.v_knots, shape->v_degree, shape->v_count, *v);
  if (!u_span || !v_span) return false;
  thread_local NurbsScratch scratch;
  scratch.reserve(std::max(shape->u_degree, shape->v_degree));
  basis_with_derivatives(s.u_knots, shape->u_degree, *u_span, *u, scratch.nu.data(),
                         partials ? scratch.dnu.data() : nullptr, scratch.lower.data(),
                         scratch.left.data(), scratch.right.data());
  basis_with_derivatives(s.v_knots, shape->v_degree, *v_span, *v, scratch.nv.data(),
                         partials ? scratch.dnv.data() : nullptr, scratch.lower.data(),
                         scratch.left.data(), scratch.right.data());
  size_t p = shape->u_degree, q = shape->v_degree;
  for (size_t k = 0; k <= p; ++k)
    if (!std::isfinite(scratch.nu[k])) return false;
  for (size_t l = 0; l <= q; ++l)
    if (!std::isfinite(scratch.nv[l])) return false;
  Vec3 a{}, au{}, av{};
  double w = 0, wu = 0, wv = 0;
  for (size_t k = 0; k <= p; ++k) {
    size_t i = *u_span - p + k;
    for (size_t l = 0; l <= q; ++l) {
      size_t j = *v_span - q + l;
      double weight = shape->rational ? s.weights[i][j] : 1.0;
      const Vec3& pole = s.poles[i][j];
      double b = scratch.nu[k] * scratch.nv[l] * weight;
      a = add(a, scale(pole, b));
      w += b;
      if (partials) {
        double bu = scratch.dnu[k] * scratch.nv[l] * weight;
        double bv = scratch.nu[k] * scratch.dnv[l] * weight;
        au = add(au, scale(pole, bu));
        av = add(av, scale(pole, bv));
        wu += bu;
        wv += bv;
      }
    }
  }
  if (shape->rational) {
    if (w == 0.0) return false;
    out.point = scale(a, 1.0 / w);
    out.point = Vec3{a.x / w, a.y / w, a.z / w};
    if (partials) {
      out.du = Vec3{(au.x - wu * out.point.x) / w, (au.y - wu * out.point.y) / w,
                    (au.z - wu * out.point.z) / w};
      out.dv = Vec3{(av.x - wv * out.point.x) / w, (av.y - wv * out.point.y) / w,
                    (av.z - wv * out.point.z) / w};
    }
  } else {
    out.point = a;
    out.du = au;
    out.dv = av;
  }
  if (!is_finite(out.point)) return false;
  if (partials && (!is_finite(out.du) || !is_finite(out.dv))) return false;
  return true;
}

/// cadmpeg-ir `math::solve::least_squares_step`.
std::optional<std::pair<double, double>> least_squares_step(Vec3 du, Vec3 dv, Vec3 residual) {
  if (!is_finite(residual)) return std::nullopt;
  double du_scale = std::max({std::fabs(du.x), std::fabs(du.y), std::fabs(du.z)});
  double dv_scale = std::max({std::fabs(dv.x), std::fabs(dv.y), std::fabs(dv.z)});
  if (du_scale == 0.0 || dv_scale == 0.0) return std::nullopt;
  du = Vec3{du.x / du_scale, du.y / du_scale, du.z / du_scale};
  dv = Vec3{dv.x / dv_scale, dv.y / dv_scale, dv.z / dv_scale};
  double du_squared = dot(du, du);
  double mixed = dot(du, dv);
  double dv_squared = dot(dv, dv);
  double determinant = std::fma(du_squared, dv_squared, -mixed * mixed);
  if (determinant <= F64_EPSILON * du_squared * dv_squared) return std::nullopt;
  if (residual.x == 0.0 && residual.y == 0.0 && residual.z == 0.0) return std::pair{0.0, 0.0};
  auto fast = [&]() -> std::optional<std::pair<double, double>> {
    std::array<double, 3> r{residual.x, residual.y, residual.z};
    std::array<double, 3> u{du.x, du.y, du.z};
    std::array<double, 3> v{dv.x, dv.y, dv.z};
    auto du_residual = fast_dot<3>(u, r, {u[0] * r[0], u[1] * r[1], u[2] * r[2]});
    if (!du_residual) return std::nullopt;
    auto dv_residual = fast_dot<3>(v, r, {v[0] * r[0], v[1] * r[1], v[2] * r[2]});
    if (!dv_residual) return std::nullopt;
    auto u_numerator = fast_dot<2>({dv_squared, -mixed}, {*du_residual, *dv_residual},
                                   {dv_squared * *du_residual, -mixed * *dv_residual});
    if (!u_numerator) return std::nullopt;
    auto v_numerator = fast_dot<2>({du_squared, -mixed}, {*dv_residual, *du_residual},
                                   {du_squared * *dv_residual, -mixed * *du_residual});
    if (!v_numerator) return std::nullopt;
    double u_denominator = determinant * du_scale;
    double v_denominator = determinant * dv_scale;
    if (!std::isnormal(u_denominator) || !std::isnormal(v_denominator)) return std::nullopt;
    auto admitted = [](double value, double numerator) -> std::optional<double> {
      if (!(std::isnormal(value) || value == 0.0)) return std::nullopt;
      if (value == 0.0 && numerator != 0.0) return std::nullopt;
      return value;
    };
    auto su = admitted(*u_numerator / u_denominator, *u_numerator);
    auto sv = admitted(*v_numerator / v_denominator, *v_numerator);
    if (!su || !sv) return std::nullopt;
    return std::pair{*su, *sv};
  };
  if (auto step = fast()) return step;
  ExactSum u_numerator, v_numerator;
  for (auto [u, v, r] : {std::array<double, 3>{du.x, dv.x, residual.x},
                         std::array<double, 3>{du.y, dv.y, residual.y},
                         std::array<double, 3>{du.z, dv.z, residual.z}}) {
    u_numerator.add_factors(dv_squared, u, r);
    u_numerator.add_factors(-mixed, v, r);
    v_numerator.add_factors(du_squared, v, r);
    v_numerator.add_factors(-mixed, u, r);
  }
  ExactSum ud, vd;
  ud.add_product(determinant, du_scale);
  vd.add_product(determinant, dv_scale);
  auto u_den = ud.finish();
  auto v_den = vd.finish();
  if (!u_den || !v_den) return std::nullopt;
  auto quotient = [](std::optional<double> n, double d) -> std::optional<double> {
    if (!n) return 0.0;
    double q = *n / d;
    if (!std::isfinite(q)) return std::nullopt;
    return q;
  };
  auto su = quotient(u_numerator.finish(), *u_den);
  auto sv = quotient(v_numerator.finish(), *v_den);
  if (!su || !sv) return std::nullopt;
  return std::pair{*su, *sv};
}

/// cadmpeg-ir `math::interpolate`.
std::optional<double> interpolate(double start, double end, double fraction) {
  if (!(fraction >= 0.0 && fraction <= 1.0)) return std::nullopt;
  double value = finite_dot<2>({1.0 - fraction, fraction}, {start, end});
  if (!std::isfinite(value)) return std::nullopt;
  return value;
}

}  // namespace

std::optional<Vec3> nurbs_surface_point(const NurbsSurfaceGeometry& surface, double u, double v) {
  SurfacePartials out;
  if (!evaluate_nurbs(surface, u, v, false, out)) return std::nullopt;
  return out.point;
}

std::optional<SurfacePartials> nurbs_surface_partials(const NurbsSurfaceGeometry& surface,
                                                      double u, double v) {
  SurfacePartials out;
  if (!evaluate_nurbs(surface, u, v, true, out)) return std::nullopt;
  return out;
}

// cadmpeg-ir eval.rs `nurbs_surface_parameter_near_point` without a seed.
std::optional<Point2> nurbs_surface_parameter_near_point(const NurbsSurfaceGeometry& surface,
                                                         Vec3 point) {
  constexpr size_t COARSE_GRID = 8;
  constexpr double COARSE_GRID_F64 = 8.0;
  constexpr size_t MAX_ITERATIONS = 24;
  constexpr size_t MAX_LINE_SEARCH_STEPS = 12;
  if (!is_finite(point)) return std::nullopt;
  size_t u_degree = surface.u_degree, v_degree = surface.v_degree;
  size_t u_count = surface.poles.size();
  size_t v_count = surface.poles.empty() ? 0 : surface.poles[0].size();
  if (u_degree > u_count || u_count >= surface.u_knots.size()) return std::nullopt;
  if (v_degree > v_count || v_count >= surface.v_knots.size()) return std::nullopt;
  double u_start = surface.u_knots[u_degree], u_end = surface.u_knots[u_count];
  double v_start = surface.v_knots[v_degree], v_end = surface.v_knots[v_count];
  if (!(u_start < u_end) || !(v_start < v_end)) return std::nullopt;
  auto project = [](double value, double lo, double hi) { return std::clamp(value, lo, hi); };

  std::optional<std::pair<Point2, double>> best;
  for (size_t ui = 0; ui <= COARSE_GRID; ++ui) {
    auto u = interpolate(u_start, u_end, static_cast<double>(ui) / COARSE_GRID_F64);
    if (!u) return std::nullopt;
    for (size_t vi = 0; vi <= COARSE_GRID; ++vi) {
      auto v = interpolate(v_start, v_end, static_cast<double>(vi) / COARSE_GRID_F64);
      if (!v) return std::nullopt;
      auto candidate = nurbs_surface_point(surface, *u, *v);
      if (!candidate) return std::nullopt;
      double d = distance(*candidate, point);
      if (!best || d < best->second) best = std::pair{Point2{*u, *v}, d};
    }
  }
  if (!best) return std::nullopt;
  Point2 parameters = best->first;
  for (size_t iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
    auto partials = nurbs_surface_partials(surface, parameters.u, parameters.v);
    if (!partials) return std::nullopt;
    double current_distance = distance(partials->point, point);
    if (current_distance == 0.0) return parameters;
    Vec3 residual = sub(partials->point, point);
    auto step = least_squares_step(partials->du, partials->dv, residual);
    if (!step) break;
    double s = 1.0;
    bool accepted = false;
    for (size_t search = 0; search < MAX_LINE_SEARCH_STEPS; ++search) {
      Point2 candidate{project(parameters.u - s * step->first, u_start, u_end),
                       project(parameters.v - s * step->second, v_start, v_end)};
      auto candidate_point = nurbs_surface_point(surface, candidate.u, candidate.v);
      if (!candidate_point) return std::nullopt;
      if (distance(*candidate_point, point) < current_distance) {
        parameters = candidate;
        accepted = true;
        break;
      }
      s *= 0.5;
    }
    if (!accepted) break;
  }
  return parameters;
}

// tessellation.rs `outside_pole_box`.
bool outside_pole_box(const PoleBox& box, Vec3 point, double tolerance) {
  if (!box.valid) return false;
  std::array<double, 3> coordinates{point.x, point.y, point.z};
  double s = 1.0;
  for (double value : box.low)
    if (std::isfinite(value)) s = std::max(s, std::fabs(value));
  for (double value : box.high)
    if (std::isfinite(value)) s = std::max(s, std::fabs(value));
  for (double value : coordinates)
    if (std::isfinite(value)) s = std::max(s, std::fabs(value));
  double margin = tolerance + EPS_POLE_BOX_ROUNDING * s;
  for (size_t axis = 0; axis < 3; ++axis)
    if (coordinates[axis] < box.low[axis] - margin || coordinates[axis] > box.high[axis] + margin)
      return true;
  return false;
}

bool outside_pole_box(const NurbsSurfaceGeometry& nurbs, Vec3 point, double tolerance) {
  return outside_pole_box(pole_box(nurbs), point, tolerance);
}

// tessellation.rs `surface_measure_with`.
std::optional<SurfaceMeasure> surface_measure_with(const SolvedSurface& surface, Vec3 point,
                                                   std::optional<double> fit_tolerance,
                                                   bool with_normal) {
  if (surface.kind == SurfaceKind::Nurbs) {
    if (!fit_tolerance || !surface.nurbs) return std::nullopt;
    double tolerance = *fit_tolerance;
    if (outside_pole_box(surface.box, point, tolerance)) return std::nullopt;
    auto parameters = nurbs_surface_parameter_near_point(*surface.nurbs, point);
    if (!parameters) return std::nullopt;
    auto partials = nurbs_surface_partials(*surface.nurbs, parameters->u, parameters->v);
    if (!partials) return std::nullopt;
    double residual = distance(point, partials->point);
    if (residual > tolerance) return std::nullopt;
    if (!std::isfinite(residual)) return std::nullopt;
    SurfaceMeasure measure{residual, std::nullopt};
    if (with_normal) measure.normal = unit(cross(partials->du, partials->dv));
    return measure;
  }
  auto residual = analytic_surface_residual(surface, point);
  if (!residual) return std::nullopt;
  SurfaceMeasure measure{*residual, std::nullopt};
  if (with_normal) measure.normal = analytic_surface_normal(surface, point);
  return measure;
}

std::optional<double> surface_residual(const SolvedSurface& surface, Vec3 point,
                                       double fit_tolerance) {
  auto measure = surface_measure_with(surface, point, fit_tolerance, false);
  if (!measure) return std::nullopt;
  return measure->residual;
}

// ---------------------------------------------------------------------------
// Polygons (tessellation.rs)
// ---------------------------------------------------------------------------
std::optional<double> simple_polygon_area_twice(std::span<const Point2> polygon,
                                                double tolerance) {
  if (polygon.size() < 3) return std::nullopt;
  auto area = polygon_area_twice(polygon);
  if (!area) return std::nullopt;
  if (std::fabs(*area) <= tolerance * tolerance) return std::nullopt;
  size_t n = polygon.size();
  for (size_t left = 0; left < n; ++left) {
    for (size_t right = left + 1; right < n; ++right) {
      if (right == left + 1 || (left == 0 && right + 1 == n)) continue;
      if (segments_intersect(polygon[left], polygon[(left + 1) % n], polygon[right],
                             polygon[(right + 1) % n], tolerance))
        return std::nullopt;
    }
  }
  return area;
}

bool is_simple_polygon(std::span<const Point2> polygon, double tolerance) {
  return simple_polygon_area_twice(polygon, tolerance).has_value();
}

std::optional<std::vector<Triangle2>> triangulate_polygon(std::span<const Point2> polygon,
                                                          double tolerance, WorkBudget& work) {
  if (!charge_product(work, polygon.size(), polygon.size())) return std::nullopt;
  auto area = simple_polygon_area_twice(polygon, tolerance);
  if (!area) return std::nullopt;
  double orientation_sign = *area > 0 ? 1.0 : (*area < 0 ? -1.0 : 0.0);
  if (*area == 0.0) orientation_sign = std::signbit(*area) ? -1.0 : 1.0;
  std::vector<size_t> remaining(polygon.size());
  for (size_t i = 0; i < remaining.size(); ++i) remaining[i] = i;
  std::vector<Triangle2> triangles;
  while (remaining.size() > 3) {
    std::optional<size_t> ear_position;
    size_t count = remaining.size();
    for (size_t position = 0; position < count; ++position) {
      if (!work.charge()) return std::nullopt;
      size_t previous_position = (position + count - 1) % count;
      size_t next_position = (position + 1) % count;
      Point2 previous = polygon[remaining[previous_position]];
      Point2 current = polygon[remaining[position]];
      Point2 next = polygon[remaining[next_position]];
      double s = std::max(point_distance(previous, current), point_distance(current, next));
      double cross_value = signed_area_twice(previous, current, next);
      if (!std::isfinite(cross_value) || orientation_sign * cross_value <= tolerance * s) continue;
      if (!charge_product(work, 2, count)) return std::nullopt;
      bool blocked = false;
      for (size_t edge_position = 0; edge_position < count && !blocked; ++edge_position) {
        size_t edge_next_position = (edge_position + 1) % count;
        if (edge_position == previous_position || edge_position == position ||
            edge_position == next_position || edge_next_position == previous_position ||
            edge_next_position == position || edge_next_position == next_position)
          continue;
        if (segments_intersect(previous, next, polygon[remaining[edge_position]],
                               polygon[remaining[edge_next_position]], tolerance))
          blocked = true;
      }
      if (blocked) continue;
      Triangle2 triangle{previous, current, next};
      auto triangle_box = PolygonBox::make(triangle, tolerance);
      for (size_t other_position = 0; other_position < count && !blocked; ++other_position) {
        Point2 point = polygon[remaining[other_position]];
        if (other_position != previous_position && other_position != position &&
            other_position != next_position && !(triangle_box && triangle_box->excludes(point)) &&
            polygon_strictly_contains(triangle, point, tolerance))
          blocked = true;
      }
      if (blocked) continue;
      ear_position = position;
      break;
    }
    if (!ear_position) return std::nullopt;
    size_t position = *ear_position;
    size_t previous_position = (position + count - 1) % count;
    size_t next_position = (position + 1) % count;
    triangles.push_back({polygon[remaining[previous_position]], polygon[remaining[position]],
                         polygon[remaining[next_position]]});
    remaining.erase(remaining.begin() + static_cast<ptrdiff_t>(position));
  }
  if (remaining.size() != 3) return std::nullopt;
  Point2 first = polygon[remaining[0]], second = polygon[remaining[1]],
         third = polygon[remaining[2]];
  double s = std::max({point_distance(first, second), point_distance(second, third),
                       point_distance(third, first)});
  if (!(std::fabs(signed_area_twice(first, second, third)) > tolerance * s)) return std::nullopt;
  triangles.push_back({first, second, third});
  return triangles;
}

namespace {

template <typename Edges>
bool edges_within(std::span<const Point2> polygon, const Edges& edges, Point2 point,
                  double tolerance) {
  for (size_t index : edges) {
    Point2 start = polygon[index];
    Point2 end = polygon[(index + 1) % polygon.size()];
    if (point.finite() && start.finite() && end.finite() &&
        !segment_box_excludes(point, start, end, tolerance) &&
        point_segment_distance(point, start, end) <= tolerance)
      return true;
  }
  return false;
}

struct IndexRange {
  size_t first, last;
  struct iterator {
    size_t value;
    size_t operator*() const { return value; }
    iterator& operator++() {
      ++value;
      return *this;
    }
    bool operator!=(const iterator& other) const { return value != other.value; }
  };
  iterator begin() const { return {first}; }
  iterator end() const { return {last}; }
};

template <typename Edges>
bool crossings_odd(std::span<const Point2> polygon, const Edges& edges, Point2 point) {
  bool inside = false;
  for (size_t index : edges) {
    Point2 start = polygon[index];
    Point2 end = polygon[(index + 1) % polygon.size()];
    if ((start.v > point.v) == (end.v > point.v)) continue;
    double crossing = start.u + (point.v - start.v) * (end.u - start.u) / (end.v - start.v);
    if (crossing > point.u) inside = !inside;
  }
  return inside;
}

}  // namespace

bool segment_box_excludes(Point2 point, Point2 start, Point2 end, double tolerance) {
  double s = std::fabs(tolerance);
  for (double value : {point.u, point.v, start.u, start.v, end.u, end.v})
    s = std::max(s, std::fabs(value));
  double reach = tolerance + SEGMENT_BOX_MARGIN * s;
  return point.u - std::max(start.u, end.u) > reach || std::min(start.u, end.u) - point.u > reach ||
         point.v - std::max(start.v, end.v) > reach || std::min(start.v, end.v) - point.v > reach;
}

bool polygon_boundary_within(std::span<const Point2> polygon, Point2 point, double tolerance) {
  return edges_within(polygon, IndexRange{0, polygon.size()}, point, tolerance);
}

bool polygon_contains(std::span<const Point2> polygon, Point2 point, double tolerance) {
  return polygon_boundary_within(polygon, point, tolerance) ||
         crossings_odd(polygon, IndexRange{0, polygon.size()}, point);
}

bool polygon_strictly_contains(std::span<const Point2> polygon, Point2 point, double tolerance) {
  return !polygon_boundary_within(polygon, point, tolerance) &&
         crossings_odd(polygon, IndexRange{0, polygon.size()}, point);
}

std::optional<PolygonBox> PolygonBox::make(std::span<const Point2> polygon, double tolerance) {
  auto bounds = polygon_bounds(polygon);
  if (!bounds) return std::nullopt;
  auto [minimum, maximum] = *bounds;
  double s = std::fabs(tolerance);
  for (double value : {minimum.u, minimum.v, maximum.u, maximum.v})
    s = std::max(s, std::fabs(value));
  if (!std::isfinite(tolerance) || !(s >= MIN_INDEXED_POLYGON_SCALE && s < MAX_BOXED_POLYGON_SCALE))
    return std::nullopt;
  return PolygonBox{minimum, maximum, s, tolerance};
}

bool PolygonBox::excludes(Point2 point) const {
  double s = std::max({scale, std::fabs(point.u), std::fabs(point.v)});
  if (!point.finite() || s >= MAX_BOXED_POLYGON_SCALE) return false;
  double reach = std::fabs(tolerance) + 2.0 * SEGMENT_BOX_MARGIN * s;
  return point.u > maximum.u + reach || point.u < minimum.u - reach ||
         point.v > maximum.v + reach || point.v < minimum.v - reach;
}

std::optional<EdgeBands> EdgeBands::make(std::span<const Point2> polygon) {
  size_t count = polygon.size();
  if (count < MIN_BANDED_POLYGON_EDGES) return std::nullopt;
  auto bounds = polygon_bounds(polygon);
  if (!bounds) return std::nullopt;
  auto [minimum, maximum] = *bounds;
  double s = 0.0;
  for (double value : {minimum.u, minimum.v, maximum.u, maximum.v})
    s = std::max(s, std::fabs(value));
  if (s < MIN_INDEXED_POLYGON_SCALE) return std::nullopt;
  size_t listing_limit = 0;
  if (__builtin_mul_overflow(count, MAX_BAND_LISTINGS_PER_EDGE, &listing_limit))
    return std::nullopt;
  size_t band_count = count;
  std::vector<std::pair<size_t, size_t>> listings;
  while (band_count >= MIN_EDGE_BANDS) {
    double band_height = (maximum.v - minimum.v) / static_cast<double>(band_count);
    if (!std::isnormal(band_height)) return std::nullopt;
    EdgeBands bands;
    bands.v_min = minimum.v;
    bands.band_height = band_height;
    bands.scale = s;
    bands.last_band = band_count - 1;
    listings.clear();
    bool fits = true;
    for (size_t index = 0; index < count; ++index) {
      double start = polygon[index].v;
      double end = polygon[(index + 1) % count].v;
      size_t low = bands.band(std::min(start, end));
      size_t high = bands.band(std::max(start, end));
      if (listings.size() + (high - low + 1) > listing_limit) {
        fits = false;
        break;
      }
      for (size_t band_index = low; band_index <= high; ++band_index)
        listings.emplace_back(band_index, index);
    }
    if (fits) {
      std::sort(listings.begin(), listings.end());
      bands.starts.resize(bands.last_band + 2);
      size_t cursor = 0;
      for (size_t band_index = 0; band_index <= bands.last_band + 1; ++band_index) {
        while (cursor < listings.size() && listings[cursor].first < band_index) ++cursor;
        bands.starts[band_index] = cursor;
      }
      bands.edges.reserve(listings.size());
      for (auto& listing : listings) bands.edges.push_back(listing.second);
      return bands;
    }
    band_count /= 4;
  }
  return std::nullopt;
}

size_t EdgeBands::band(double v) const {
  double position = std::floor((v - v_min) / band_height);
  if (std::isnan(position) || position <= 0.0) return 0;
  if (!(position < 18446744073709551616.0)) return last_band;
  return std::min(static_cast<size_t>(position), last_band);
}

std::optional<std::span<const size_t>> EdgeBands::at(Point2 point) const {
  if (!point.finite()) return std::nullopt;
  size_t b = band(point.v);
  return std::span<const size_t>(edges.data() + starts[b], starts[b + 1] - starts[b]);
}

std::optional<std::span<const size_t>> EdgeBands::near(Point2 point, double tolerance) const {
  if (!point.finite() || !std::isfinite(tolerance)) return std::nullopt;
  double s = std::max(scale, std::fabs(tolerance));
  s = std::max({s, std::fabs(point.u), std::fabs(point.v)});
  double reach = std::fabs(tolerance) + 2.0 * SEGMENT_BOX_MARGIN * s;
  size_t first = starts[band(point.v - reach)];
  size_t last = starts[band(point.v + reach) + 1];
  return std::span<const size_t>(edges.data() + first, last - first);
}

TrimPolygon::TrimPolygon(std::vector<Point2> points)
    : points_(std::move(points)), bands_(EdgeBands::make(points_)) {}

bool TrimPolygon::is_simple(double tolerance) const {
  uint64_t key = std::bit_cast<uint64_t>(tolerance);
  if (simple_at_ && simple_at_->first == key) return simple_at_->second;
  bool simple = is_simple_polygon(points_, tolerance);
  simple_at_ = std::pair{key, simple};
  return simple;
}

bool TrimPolygon::boundary_within(Point2 point, double tolerance) const {
  if (bands_) {
    if (auto edges = bands_->near(point, tolerance))
      return edges_within(points_, *edges, point, tolerance);
  }
  return polygon_boundary_within(points_, point, tolerance);
}

bool TrimPolygon::contains(Point2 point, double tolerance) const {
  return boundary_within(point, tolerance) || crossings_odd(point);
}

bool TrimPolygon::strictly_contains(Point2 point, double tolerance) const {
  return !boundary_within(point, tolerance) && crossings_odd(point);
}

bool TrimPolygon::crossings_odd(Point2 point) const {
  if (bands_) {
    if (auto edges = bands_->at(point)) return ownership::crossings_odd(points_, *edges, point);
  }
  return ownership::crossings_odd(points_, IndexRange{0, points_.size()}, point);
}

std::optional<CheckedPlanarPolygon> CheckedPlanarPolygon::make(const TrimPolygon& boundary,
                                                               double tolerance,
                                                               WorkBudget& work) {
  size_t length = boundary.points().size();
  if (!charge_product(work, length, length)) return std::nullopt;
  if (!boundary.is_simple(tolerance)) return std::nullopt;
  return CheckedPlanarPolygon(boundary, tolerance, polygon_is_convex(boundary.points()), work);
}

bool CheckedPlanarPolygon::contains_triangle(const Triangle2& triangle) const {
  for (const Point2& point : triangle)
    if (!point.finite()) return false;
  const auto& points = boundary_->points();
  WorkBudget& work = *work_;
  if (convex_) {
    if (!charge_product(work, 3, points.size())) return false;
    for (const Point2& point : triangle)
      if (!boundary_->contains(point, tolerance_)) return false;
    return true;
  }
  std::vector<double> cuts;
  for (size_t edge = 0; edge < 3; ++edge) {
    Point2 start = triangle[edge];
    Point2 end = triangle[(edge + 1) % 3];
    double du = end.u - start.u;
    double dv = end.v - start.v;
    double length_squared = du * du + dv * dv;
    if (!std::isfinite(length_squared)) return false;
    if (length_squared == 0.0) continue;
    if (!work.charge_by(points.size())) return false;
    cuts.assign({0.0, 1.0});
    for (size_t index = 0; index < points.size(); ++index) {
      Point2 point = points[index];
      Point2 next = points[(index + 1) % points.size()];
      double pu = point.u - start.u;
      double pv = point.v - start.v;
      double projected = (pu * du + pv * dv) / length_squared;
      if (!std::isfinite(projected)) return false;
      if (projected >= 0.0 && projected <= 1.0) cuts.push_back(projected);
      double eu = next.u - point.u;
      double ev = next.v - point.v;
      double denominator = du * ev - dv * eu;
      if (!std::isfinite(denominator)) return false;
      if (denominator == 0.0) continue;
      double along_triangle = (pu * ev - pv * eu) / denominator;
      double along_boundary = (pu * dv - pv * du) / denominator;
      if (!std::isfinite(along_triangle) || !std::isfinite(along_boundary)) return false;
      if (along_triangle >= 0.0 && along_triangle <= 1.0 && along_boundary >= 0.0 &&
          along_boundary <= 1.0)
        cuts.push_back(along_triangle);
    }
    std::sort(cuts.begin(), cuts.end(), total_less);
    if (!charge_product(work, cuts.size(), points.size())) return false;
    for (size_t i = 0; i + 1 < cuts.size(); ++i) {
      double parameter = cuts[i] + (cuts[i + 1] - cuts[i]) / 2.0;
      if (!boundary_->contains(Point2{start.u + parameter * du, start.v + parameter * dv},
                               tolerance_))
        return false;
    }
  }
  return true;
}

bool polygon_contains_triangle(std::span<const Point2> boundary, const Triangle2& triangle,
                               double tolerance) {
  WorkBudget work(std::numeric_limits<size_t>::max());
  TrimPolygon polygon(std::vector<Point2>(boundary.begin(), boundary.end()));
  auto checked = CheckedPlanarPolygon::make(polygon, tolerance, work);
  return checked && checked->contains_triangle(triangle);
}

bool polygon_is_convex(std::span<const Point2> boundary) {
  double orientation_value = 0.0;
  size_t n = boundary.size();
  for (size_t index = 0; index < n; ++index) {
    double turn =
        signed_area_twice(boundary[index], boundary[(index + 1) % n], boundary[(index + 2) % n]);
    if (!std::isfinite(turn)) return false;
    if (turn != 0.0) {
      if (orientation_value != 0.0 && orientation_value * turn < 0.0) return false;
      orientation_value = turn > 0 ? 1.0 : -1.0;
    }
  }
  return orientation_value != 0.0;
}

std::optional<std::pair<Point2, Point2>> polygon_bounds(std::span<const Point2> polygon) {
  if (polygon.empty()) return std::nullopt;
  Point2 minimum = polygon[0], maximum = minimum;
  for (const Point2& point : polygon) {
    if (!point.finite()) return std::nullopt;
    minimum.u = std::min(minimum.u, point.u);
    minimum.v = std::min(minimum.v, point.v);
    maximum.u = std::max(maximum.u, point.u);
    maximum.v = std::max(maximum.v, point.v);
  }
  return std::pair{minimum, maximum};
}

bool polygon_inside_polygon(std::span<const Point2> inner, std::span<const Point2> outer,
                            double tolerance) {
  if (inner.empty()) return true;
  if (!polygon_contains(outer, inner[0], tolerance)) return false;
  if (std::isfinite(tolerance) && tolerance >= 0.0 && polygon_is_convex(outer)) {
    if (auto bounds = polygon_bounds(inner)) {
      auto [minimum, maximum] = *bounds;
      std::array<Point2, 4> corners{minimum, Point2{minimum.u, maximum.v}, maximum,
                                    Point2{maximum.u, minimum.v}};
      bool contained = true;
      for (const Point2& corner : corners) {
        if (!polygon_strictly_contains(outer, corner, tolerance)) {
          contained = false;
          break;
        }
      }
      if (contained) return true;
    }
  }
  for (size_t i = 1; i < inner.size(); ++i)
    if (!polygon_contains(outer, inner[i], tolerance)) return false;
  for (size_t li = 0; li < inner.size(); ++li)
    for (size_t ri = 0; ri < outer.size(); ++ri)
      if (segments_intersect(inner[li], inner[(li + 1) % inner.size()], outer[ri],
                             outer[(ri + 1) % outer.size()], tolerance))
        return false;
  return true;
}

bool polygons_overlap(std::span<const Point2> first, std::span<const Point2> second,
                      double tolerance) {
  if (!first.empty() && !second.empty()) {
    if (segments_intersect(first[0], first[1 % first.size()], second[0], second[1 % second.size()],
                           tolerance))
      return true;
  }
  if (std::isfinite(tolerance) && tolerance >= 0.0) {
    auto a = polygon_bounds(first);
    auto b = polygon_bounds(second);
    if (a && b) {
      auto [first_min, first_max] = *a;
      auto [second_min, second_max] = *b;
      if (first_max.u + tolerance < second_min.u || second_max.u + tolerance < first_min.u ||
          first_max.v + tolerance < second_min.v || second_max.v + tolerance < first_min.v)
        return false;
    }
  }
  for (size_t fi = 0; fi < first.size(); ++fi) {
    for (size_t si = 0; si < second.size(); ++si) {
      if (fi == 0 && si == 0) continue;
      if (segments_intersect(first[fi], first[(fi + 1) % first.size()], second[si],
                             second[(si + 1) % second.size()], tolerance))
        return true;
    }
  }
  for (const Point2& point : first)
    if (polygon_strictly_contains(second, point, tolerance)) return true;
  for (const Point2& point : second)
    if (polygon_strictly_contains(first, point, tolerance)) return true;
  return false;
}

namespace {
std::pair<double, double> triangle_projection(const Triangle2& triangle, Point2 axis) {
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const Point2& point : triangle) {
    double value = point.u * axis.u + point.v * axis.v;
    minimum = std::fmin(minimum, value);
    maximum = std::fmax(maximum, value);
  }
  return {minimum, maximum};
}
}  // namespace

bool triangles_have_positive_overlap(const Triangle2& first, const Triangle2& second,
                                     double tolerance) {
  for (const Triangle2* triangle : {&first, &second}) {
    for (size_t index = 0; index < 3; ++index) {
      Point2 start = (*triangle)[index];
      Point2 end = (*triangle)[(index + 1) % 3];
      double length = point_distance(start, end);
      if (!std::isfinite(length) || length <= F64_EPSILON) return false;
      Point2 axis{(start.v - end.v) / length, (end.u - start.u) / length};
      auto a = triangle_projection(first, axis);
      auto b = triangle_projection(second, axis);
      double overlap = std::fmin(a.second, b.second) - std::fmax(a.first, b.first);
      if (!std::isfinite(overlap) || overlap <= tolerance) return false;
    }
  }
  return true;
}

bool convex_polygon_contains(std::span<const Point2> polygon, Point2 point, double tolerance) {
  double sign = 0.0;
  for (size_t index = 0; index < polygon.size(); ++index) {
    Point2 start = polygon[index];
    Point2 end = polygon[(index + 1) % polygon.size()];
    double cross_value = signed_area_twice(start, end, point);
    if (point.finite() && start.finite() && end.finite() &&
        point_segment_distance(point, start, end) <= tolerance)
      continue;
    double signum = std::isnan(cross_value) ? cross_value : (std::signbit(cross_value) ? -1.0 : 1.0);
    if (sign == 0.0) sign = signum;
    else if (signum != sign) return false;
  }
  return true;
}

bool circle_inside_polygon(std::span<const Point2> polygon, CircularHole hole, double tolerance) {
  if (!polygon_contains(polygon, hole.center, tolerance)) return false;
  for (size_t index = 0; index < polygon.size(); ++index) {
    Point2 start = polygon[index], end = polygon[(index + 1) % polygon.size()];
    if (!(hole.center.finite() && start.finite() && end.finite())) return false;
    if (!(point_segment_distance(hole.center, start, end) >= hole.radius - tolerance)) return false;
  }
  return true;
}

bool circle_inside_circle(CircularHole outer, CircularHole inner, double tolerance) {
  return point_distance(outer.center, inner.center) + inner.radius <= outer.radius + tolerance;
}

std::optional<std::pair<CircularHole, std::vector<CircularHole>>> circular_outer_and_holes(
    std::span<const CircularHole> circles, double tolerance) {
  std::optional<size_t> outer_index;
  for (size_t index = 0; index < circles.size(); ++index) {
    bool all = true;
    for (size_t inner = 0; inner < circles.size() && all; ++inner)
      if (index != inner && !circle_inside_circle(circles[index], circles[inner], tolerance))
        all = false;
    if (all) {
      if (outer_index) return std::nullopt;
      outer_index = index;
    }
  }
  if (!outer_index) return std::nullopt;
  std::vector<CircularHole> holes;
  for (size_t index = 0; index < circles.size(); ++index)
    if (index != *outer_index) holes.push_back(circles[index]);
  for (size_t i = 0; i < holes.size(); ++i)
    for (size_t j = i + 1; j < holes.size(); ++j)
      if (point_distance(holes[i].center, holes[j].center) <
          holes[i].radius + holes[j].radius - tolerance)
        return std::nullopt;
  return std::pair{circles[*outer_index], std::move(holes)};
}

std::optional<std::pair<CircularHole, CircularHole>> chordal_hole_constraint(
    CircularHole hole, std::span<const Point2> points, double tolerance) {
  std::optional<double> minimum;
  for (const Point2& point : points) {
    double d = point_distance(point, hole.center);
    minimum = minimum ? std::fmin(*minimum, d) : d;
  }
  if (!minimum) return std::nullopt;
  if (*minimum >= hole.radius - tolerance) return std::pair{hole, hole};
  std::vector<double> boundary_angles;
  for (const Point2& point : points) {
    double d = point_distance(point, hole.center);
    if (std::fabs(d - hole.radius) <= tolerance)
      boundary_angles.push_back(std::atan2(point.v - hole.center.v, point.u - hole.center.u));
  }
  std::sort(boundary_angles.begin(), boundary_angles.end(), total_less);
  // Vec::dedup_by: drop each angle within tolerance of the last kept one.
  {
    double limit = tolerance / hole.radius;
    std::vector<double> kept;
    for (double angle : boundary_angles)
      if (kept.empty() || !(std::fabs(angle - kept.back()) <= limit)) kept.push_back(angle);
    boundary_angles.swap(kept);
  }
  if (boundary_angles.size() < 3) return std::nullopt;
  double wrap_gap = boundary_angles[0] + TAU - boundary_angles.back();
  double maximum_gap = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < boundary_angles.size(); ++i)
    maximum_gap = std::fmax(maximum_gap, boundary_angles[i + 1] - boundary_angles[i]);
  maximum_gap = std::fmax(maximum_gap, wrap_gap);
  if (maximum_gap > PI) return std::nullopt;
  double maximum_sagitta = hole.radius * (1.0 - std::cos(maximum_gap / 2.0));
  double inward_deflection = hole.radius - *minimum;
  if (inward_deflection > maximum_sagitta + tolerance) return std::nullopt;
  return std::pair{CircularHole{hole.center, hole.radius - maximum_sagitta}, hole};
}

bool triangle_crosses_hole(const Triangle2& triangle, CircularHole exclusion,
                           CircularHole boundary, double tolerance) {
  if (convex_polygon_contains(triangle, exclusion.center, tolerance)) return true;
  for (size_t index = 0; index < 3; ++index) {
    Point2 start = triangle[index];
    Point2 end = triangle[(index + 1) % 3];
    bool near = exclusion.center.finite() && start.finite() && end.finite() &&
                point_segment_distance(exclusion.center, start, end) < exclusion.radius - tolerance;
    bool both_on_boundary =
        std::fabs(point_distance(start, boundary.center) - boundary.radius) <= tolerance &&
        std::fabs(point_distance(end, boundary.center) - boundary.radius) <= tolerance;
    if (near && !both_on_boundary) return true;
  }
  return false;
}

bool circle_overlaps_polygon(CircularHole circle, std::span<const Point2> polygon,
                             double tolerance) {
  if (polygon_strictly_contains(polygon, circle.center, tolerance)) return true;
  for (const Point2& point : polygon)
    if (point_distance(point, circle.center) < circle.radius + tolerance) return true;
  for (size_t index = 0; index < polygon.size(); ++index) {
    Point2 start = polygon[index], end = polygon[(index + 1) % polygon.size()];
    if (circle.center.finite() && start.finite() && end.finite() &&
        point_segment_distance(circle.center, start, end) < circle.radius + tolerance)
      return true;
  }
  return false;
}

}  // namespace swglb::ownership
