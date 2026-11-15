// SPDX-License-Identifier: Apache-2.0
// Internal interface of the tessellation owner stage (ownership.cc,
// ownership_geometry.cc). Ported from cadmpeg-codec-sldprt tessellation.rs
// (`assign_face_key_owners`, `assign_persistent_owners`,
// `assign_unique_surface_owners` and their trims and predicates), with the
// helpers they use from cadmpeg-ir (`math::planar`, `math::solve`,
// `eval::nurbs_surface_parameter_near_point`, `eval::nurbs_surface_partials`)
// at cadmpeg 5af95421 (Apache-2.0). Exposed for tests/ownership_test.cc.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "model.h"
#include "stages.h"

namespace swglb::ownership {

// ---------------------------------------------------------------------------
// Constants (tessellation.rs)
// ---------------------------------------------------------------------------
inline constexpr double EPS_DISPLAY_QUANTIZATION = 1.0e-9;
inline constexpr double EPS_AXIS_ALIGNMENT = 1.0e-9;
inline constexpr double EPS_CYLINDER_ANGLE = 1.0e-12;
inline constexpr double EPS_POLE_BOX_ROUNDING = 1.0e-9;
inline constexpr double DISPLAY_QUANTIZATION_ULPS = 8.0;
inline constexpr size_t MAX_PLANAR_TRIM_ARC_SEGMENTS = 4096;
inline constexpr double MIN_TESSELLATION_NORMAL_ALIGNMENT = 1.0 - 1.0e-4;
/// Floor of the work one surface-owner pass may spend.
inline constexpr size_t TESSELLATION_OWNER_WORK = size_t{1} << 30;
/// Work per model row above the floor.
inline constexpr size_t TESSELLATION_OWNER_WORK_PER_ROW = size_t{1} << 17;
inline constexpr double SEGMENT_BOX_MARGIN = 1.0e-9;
inline constexpr size_t MIN_BANDED_POLYGON_EDGES = 16;
inline constexpr size_t MAX_BAND_LISTINGS_PER_EDGE = 8;
inline constexpr size_t MIN_EDGE_BANDS = 4;
inline constexpr double MIN_INDEXED_POLYGON_SCALE = 1.0e-200;
inline constexpr double MAX_BOXED_POLYGON_SCALE = 1.0e150;
inline constexpr double F32_EPSILON = 1.1920928955078125e-7;  ///< f32::EPSILON
inline constexpr double F64_EPSILON = 2.220446049250313e-16;  ///< f64::EPSILON
inline constexpr double PI = 3.141592653589793;
inline constexpr double TAU = 6.283185307179586;

// ---------------------------------------------------------------------------
// Work budget (cadmpeg-core decode::WorkBudget, without parents)
// ---------------------------------------------------------------------------
class WorkBudget {
 public:
  explicit WorkBudget(size_t limit) : remaining_(limit) {}
  bool charge() { return charge_by(1); }
  /// Charges `work` units; a refused charge exhausts the budget (sticky).
  bool charge_by(size_t work) {
    if (exhausted_) return false;
    if (work > remaining_) {
      exhausted_ = true;
      return false;
    }
    remaining_ -= work;
    return true;
  }
  void exhaust() { exhausted_ = true; }
  bool exhausted() const { return exhausted_; }

 private:
  size_t remaining_;
  bool exhausted_ = false;
};

/// tessellation.rs `charge_product`: a product past size_t exhausts.
bool charge_product(WorkBudget& work, size_t left, size_t right);

// ---------------------------------------------------------------------------
// Vector arithmetic (cadmpeg-ir math)
// ---------------------------------------------------------------------------
inline Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 scale(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline bool is_finite(Vec3 a) {
  return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}
/// Compensated dot product (cadmpeg `Vector3::dot` sums the products exactly).
double dot(Vec3 a, Vec3 b);
/// `hypot(hypot(x, y), z)` (cadmpeg `Vector3::norm`).
inline double norm(Vec3 a) { return std::hypot(std::hypot(a.x, a.y), a.z); }
inline double distance(Vec3 a, Vec3 b) { return norm(sub(a, b)); }
/// cadmpeg `Vector3::unit`: none at or below f64::EPSILON length.
std::optional<Vec3> unit(Vec3 a);
/// Divides by the length without a check (cadmpeg `perpendicular_basis` `divided`).
inline Vec3 divided(Vec3 a) {
  double length = norm(a);
  return {a.x / length, a.y / length, a.z / length};
}
/// f64::rem_euclid.
inline double rem_euclid(double value, double modulus) {
  double r = std::fmod(value, modulus);
  return r < 0.0 ? r + std::fabs(modulus) : r;
}
/// f64::total_cmp(a, b) == Less.
bool total_less(double a, double b);

struct Point2 {
  double u = 0, v = 0;
  bool operator==(const Point2&) const = default;
  bool finite() const { return std::isfinite(u) && std::isfinite(v); }
};
using Triangle2 = std::array<Point2, 3>;

// ---------------------------------------------------------------------------
// cadmpeg-ir math::planar
// ---------------------------------------------------------------------------
std::optional<double> polygon_area_twice(std::span<const Point2> points);
/// Inputs must be finite.
double point_segment_distance(Point2 point, Point2 start, Point2 end);
bool segments_intersect(Point2 a, Point2 b, Point2 c, Point2 d, double tolerance);

// ---------------------------------------------------------------------------
// Surfaces
// ---------------------------------------------------------------------------
enum class SurfaceKind { Plane, Cylinder, Cone, Sphere, Torus, Nurbs, Unknown };

/// Bounds of a NURBS pole grid (tessellation.rs `outside_pole_box`),
/// computed once per surface.
struct PoleBox {
  bool valid = false;  ///< false: a weight <= 0 gives no bound
  std::array<double, 3> low{}, high{};
};
PoleBox pole_box(const NurbsSurfaceGeometry& nurbs);

/// A solved carrier (cadmpeg `SolvedSurfaceGeometry`). Directions are the
/// admitted frame (axis = model `normal`, reference = model `u_axis`).
struct SolvedSurface {
  SurfaceKind kind = SurfaceKind::Unknown;
  Vec3 origin, axis, reference;
  double radius = 0, minor_radius = 0, half_angle = 0, ratio = 1;
  const NurbsSurfaceGeometry* nurbs = nullptr;  ///< Nurbs only; may be null.
  PoleBox box;
};
/// `SurfaceGeometry::solved()`: none for a procedural carrier (which never
/// carries a cache in SLDPRT). Unrecognised kinds are `Unknown`.
std::optional<SolvedSurface> solved_surface(const SurfaceGeometry& geometry);

std::pair<Vec3, Vec3> perpendicular_basis(Vec3 axis, Vec3 reference);
std::optional<Vec3> analytic_surface_normal(const SolvedSurface& surface, Vec3 point);
std::optional<double> analytic_surface_residual(const SolvedSurface& surface, Vec3 point);

struct SurfacePartials {
  Vec3 point, du, dv;
};
std::optional<Vec3> nurbs_surface_point(const NurbsSurfaceGeometry& surface, double u, double v);
std::optional<SurfacePartials> nurbs_surface_partials(const NurbsSurfaceGeometry& surface,
                                                      double u, double v);
std::optional<Point2> nurbs_surface_parameter_near_point(const NurbsSurfaceGeometry& surface,
                                                         Vec3 point);
bool outside_pole_box(const PoleBox& box, Vec3 point, double tolerance);
bool outside_pole_box(const NurbsSurfaceGeometry& nurbs, Vec3 point, double tolerance);

struct SurfaceMeasure {
  double residual = 0;
  std::optional<Vec3> normal;
};
std::optional<SurfaceMeasure> surface_measure_with(const SolvedSurface& surface, Vec3 point,
                                                   std::optional<double> fit_tolerance,
                                                   bool with_normal);
inline std::optional<SurfaceMeasure> surface_measure(const SolvedSurface& surface, Vec3 point,
                                                     std::optional<double> fit_tolerance) {
  return surface_measure_with(surface, point, fit_tolerance, true);
}
std::optional<double> surface_residual(const SolvedSurface& surface, Vec3 point,
                                       double fit_tolerance);

// ---------------------------------------------------------------------------
// Polygons (tessellation.rs)
// ---------------------------------------------------------------------------
struct CircularHole {
  Point2 center;
  double radius = 0;
};

inline double point_distance(Point2 a, Point2 b) { return std::hypot(a.u - b.u, a.v - b.v); }
inline double signed_area_twice(Point2 l, Point2 m, Point2 r) {
  return (m.u - l.u) * (r.v - m.v) - (m.v - l.v) * (r.u - m.u);
}
bool is_simple_polygon(std::span<const Point2> polygon, double tolerance);
std::optional<double> simple_polygon_area_twice(std::span<const Point2> polygon, double tolerance);
std::optional<std::vector<Triangle2>> triangulate_polygon(std::span<const Point2> polygon,
                                                          double tolerance, WorkBudget& work);
bool polygon_boundary_within(std::span<const Point2> polygon, Point2 point, double tolerance);
bool segment_box_excludes(Point2 point, Point2 start, Point2 end, double tolerance);
bool polygon_contains(std::span<const Point2> polygon, Point2 point, double tolerance);
bool polygon_strictly_contains(std::span<const Point2> polygon, Point2 point, double tolerance);
bool polygon_is_convex(std::span<const Point2> boundary);
std::optional<std::pair<Point2, Point2>> polygon_bounds(std::span<const Point2> polygon);
bool polygon_inside_polygon(std::span<const Point2> inner, std::span<const Point2> outer,
                            double tolerance);
bool polygons_overlap(std::span<const Point2> first, std::span<const Point2> second,
                      double tolerance);
bool triangles_have_positive_overlap(const Triangle2& first, const Triangle2& second,
                                     double tolerance);
bool convex_polygon_contains(std::span<const Point2> polygon, Point2 point, double tolerance);
bool circle_inside_polygon(std::span<const Point2> polygon, CircularHole hole, double tolerance);
bool circle_inside_circle(CircularHole outer, CircularHole inner, double tolerance);
std::optional<std::pair<CircularHole, std::vector<CircularHole>>> circular_outer_and_holes(
    std::span<const CircularHole> circles, double tolerance);
std::optional<std::pair<CircularHole, CircularHole>> chordal_hole_constraint(
    CircularHole hole, std::span<const Point2> points, double tolerance);
bool triangle_crosses_hole(const Triangle2& triangle, CircularHole exclusion,
                           CircularHole boundary, double tolerance);
bool circle_overlaps_polygon(CircularHole circle, std::span<const Point2> polygon,
                             double tolerance);

/// tessellation.rs `PolygonBox`.
struct PolygonBox {
  Point2 minimum, maximum;
  double scale = 0, tolerance = 0;
  static std::optional<PolygonBox> make(std::span<const Point2> polygon, double tolerance);
  bool excludes(Point2 point) const;
};

/// tessellation.rs `EdgeBands`.
struct EdgeBands {
  double v_min = 0, band_height = 0, scale = 0;
  size_t last_band = 0;
  std::vector<size_t> starts, edges;
  static std::optional<EdgeBands> make(std::span<const Point2> polygon);
  size_t band(double v) const;
  std::optional<std::span<const size_t>> at(Point2 point) const;
  std::optional<std::span<const size_t>> near(Point2 point, double tolerance) const;
};

/// tessellation.rs `TrimPolygon`.
class TrimPolygon {
 public:
  explicit TrimPolygon(std::vector<Point2> points);
  const std::vector<Point2>& points() const { return points_; }
  const std::optional<EdgeBands>& bands() const { return bands_; }
  bool is_simple(double tolerance) const;
  bool boundary_within(Point2 point, double tolerance) const;
  bool contains(Point2 point, double tolerance) const;
  bool strictly_contains(Point2 point, double tolerance) const;
  bool crossings_odd(Point2 point) const;

 private:
  std::vector<Point2> points_;
  std::optional<EdgeBands> bands_;
  mutable std::optional<std::pair<uint64_t, bool>> simple_at_;
};

/// tessellation.rs `CheckedPlanarPolygon`.
class CheckedPlanarPolygon {
 public:
  static std::optional<CheckedPlanarPolygon> make(const TrimPolygon& boundary, double tolerance,
                                                  WorkBudget& work);
  bool contains_triangle(const Triangle2& triangle) const;

 private:
  CheckedPlanarPolygon(const TrimPolygon& b, double t, bool c, WorkBudget& w)
      : boundary_(&b), tolerance_(t), convex_(c), work_(&w) {}
  const TrimPolygon* boundary_;
  double tolerance_;
  bool convex_;
  WorkBudget* work_;
};
/// Test helper (tessellation.rs `#[cfg(test)] polygon_contains_triangle`).
bool polygon_contains_triangle(std::span<const Point2> boundary, const Triangle2& triangle,
                               double tolerance);

// ---------------------------------------------------------------------------
// Mesh samples and trims
// ---------------------------------------------------------------------------
/// A mesh's positions, shading normals and strip triangles.
struct MeshSamples {
  std::vector<Vec3> vertices;
  std::vector<Vec3> normals;  ///< Empty for an unshaded table.
  std::vector<std::array<uint32_t, 3>> triangles;
  static MeshSamples of(const Tessellation& mesh);
};
size_t vertex_count(const Tessellation& mesh);
size_t triangle_count(const Tessellation& mesh);

struct PlaneFrame {
  Vec3 origin, normal, u_axis;
  static std::optional<PlaneFrame> make(Vec3 origin, Vec3 normal, Vec3 u_axis);
  Vec3 v_axis() const;
  Point2 project(Vec3 point) const;
};

struct PlanarHole {
  /// A circle when `boundary` is empty.
  std::optional<CircularHole> circle;
  std::optional<TrimPolygon> boundary;
  std::vector<Triangle2> triangles;
  static PlanarHole make_circle(CircularHole hole) { return PlanarHole{hole, std::nullopt, {}}; }
  static std::optional<PlanarHole> polygon(std::vector<Point2> boundary, double tolerance,
                                           WorkBudget& work);
};

struct PlanarTrim {
  PlaneFrame frame;
  /// Outer: a polygon, a circle, or none.
  std::optional<std::variant<TrimPolygon, CircularHole>> outer;
  std::vector<PlanarHole> holes;
  double boundary_tolerance = 0;
  bool contains_mesh(const MeshSamples& mesh, double tolerance, WorkBudget& work) const;
};

struct CylindricalTrim {
  Vec3 origin, axis, reference;
  double radius = 0, min_axial = 0, max_axial = 0, angular_start = 0, angular_span = 0;
  bool contains_mesh(const MeshSamples& mesh, double tolerance) const;
};

struct ConicalTrim {
  Vec3 origin, axis, reference;
  double radius = 0, ratio = 1, slope = 0, min_axial = 0, max_axial = 0, angular_start = 0,
         angular_span = 0;
  bool contains_mesh(const MeshSamples& mesh, double tolerance) const;
};

struct AnalyticTrim {
  std::variant<PlanarTrim, CylindricalTrim, ConicalTrim> trim;
  bool contains_mesh(const MeshSamples& mesh, double tolerance, WorkBudget& work) const;
};

std::optional<double> cylinder_angle(Vec3 point, Vec3 origin, Vec3 axis, Vec3 reference);
std::optional<double> cone_angle(Vec3 point, Vec3 origin, Vec3 axis, Vec3 reference,
                                 double ratio);
std::optional<std::pair<double, double>> circular_interval(std::vector<double>& angles);
bool circular_interval_contains(double start, double span, double angle, double tolerance);
std::optional<double> shortest_arc_span(double start, double end);
std::optional<std::pair<size_t, double>> planar_arc_segments(double span, double radius,
                                                             double tolerance);
std::optional<PlaneFrame> plane_frame(const SolvedSurface& surface);

struct BoundaryTolerance {
  double tolerance = 0, sampling_tolerance = 0;
};
std::optional<std::pair<std::vector<Point2>, double>> planar_boundary_samples(
    const CurveGeometry& curve, Vec3 start, Vec3 end, const SolvedSurface& surface,
    const PlaneFrame& frame, BoundaryTolerance limits);

/// Indexed topology the trim grammars read (tessellation.rs `TrimTopology`).
/// Holds pointers into `model`, which must outlive it unchanged (only
/// tessellations may change).
struct TrimTopology {
  std::unordered_map<std::string_view, const Loop*> loops;
  std::unordered_map<std::string_view, const Coedge*> coedges;
  std::unordered_map<std::string_view, const Edge*> edges;
  std::unordered_map<std::string_view, const Vertex*> vertices;
  std::unordered_map<std::string_view, Vec3> points;
  std::unordered_map<std::string_view, const CurveGeometry*> curves;
  /// Largest absolute coordinate over `points`, at least one.
  double coordinate_scale = 1;
  explicit TrimTopology(const Model& model);
};
std::optional<PlanarTrim> planar_trim(const Face& face, const SolvedSurface& surface,
                                      const TrimTopology& topology, WorkBudget& work);
std::optional<AnalyticTrim> analytic_trim(const Face& face, const SolvedSurface& surface,
                                          const TrimTopology& topology, WorkBudget& work);

// ---------------------------------------------------------------------------
// Owner assignment
// ---------------------------------------------------------------------------
std::vector<std::string> assign_face_key_owners(
    Model& model, const std::vector<std::pair<std::string, uint32_t>>& face_keys,
    const std::vector<FaceKeyBinding>& bindings);
std::vector<std::string> assign_persistent_owners(
    Model& model, const std::vector<std::pair<std::string, PersistentFaceIdentity>>& identities,
    const std::vector<PersistentFaceBinding>& bindings);

struct SurfaceOwnerResult {
  std::vector<std::string> assigned;
  size_t capped = 0;  ///< Tables left unassigned by the work cap.
  /// Chordal deflection of each table assigned through an approximate fit
  /// (tessellation id, deflection); not part of the CADIR subset.
  std::vector<std::pair<std::string, double>> deflections;
};
std::optional<size_t> tessellation_owner_rows(const Model& model);
size_t tessellation_owner_work_limit(std::optional<size_t> rows);
SurfaceOwnerResult assign_unique_surface_owners(Model& model);
SurfaceOwnerResult assign_unique_surface_owners_within(Model& model, size_t work_limit);

}  // namespace swglb::ownership
