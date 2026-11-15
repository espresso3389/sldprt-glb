// SPDX-License-Identifier: Apache-2.0
// Tessellation owner assignment: face keys, persistent identities, then
// unique surface owners. Ported from cadmpeg-codec-sldprt tessellation.rs
// (`assign_face_key_owners`, `assign_persistent_owners`,
// `assign_unique_surface_owners_within`, `approximate_surface_owner`,
// `approximate_trimmed_surface_owner`, the planar / cylindrical / conical
// trims) at cadmpeg 5af95421 (Apache-2.0), as called by decode.rs for both
// the B-rep and the metadata-only path.
//
// Differences from cadmpeg, none of which occur on SLDPRT input:
// - bodies carry no placement transform in the SLDPRT B-rep, so the body
//   inverse is always the identity (cadmpeg `identity_inverse`);
// - faces state no evaluation tolerance (cadmpeg `Face::tolerance` is always
//   None for SLDPRT), so the below-quantization refusal is not ported;
// - loss notes are not produced (the capped count is returned instead).
#include <algorithm>
#include <map>
#include <unordered_map>

#include "ownership_internal.h"

namespace swglb {
namespace ownership {

namespace {

/// The quantization tolerance at `coordinate_scale` (tessellation.rs).
double quantization_tolerance_at(double coordinate_scale) {
  return coordinate_scale * F32_EPSILON * DISPLAY_QUANTIZATION_ULPS + EPS_DISPLAY_QUANTIZATION;
}

/// cadmpeg `identity_point`: adding zero makes each negative zero positive.
Vec3 identity_point(Vec3 point) { return {point.x + 0.0, point.y + 0.0, point.z + 0.0}; }

/// cadmpeg `FaceEvaluationTolerance::of` for a face without a stated tolerance.
constexpr double FACE_EVALUATION_TOLERANCE = EPS_DISPLAY_QUANTIZATION;

template <typename Map, typename Key>
auto find_ptr(const Map& map, const Key& key) -> const typename Map::mapped_type* {
  auto it = map.find(key);
  return it == map.end() ? nullptr : &it->second;
}

}  // namespace

// ---------------------------------------------------------------------------
// Mesh samples
// ---------------------------------------------------------------------------
size_t vertex_count(const Tessellation& mesh) {
  size_t count = 0;
  for (const auto& strip : mesh.strips) count += strip.size();
  return count;
}

size_t triangle_count(const Tessellation& mesh) {
  size_t count = 0;
  for (const auto& strip : mesh.strips)
    if (strip.size() >= 3) count += strip.size() - 2;
  return count;
}

MeshSamples MeshSamples::of(const Tessellation& mesh) {
  MeshSamples samples;
  size_t vertices = vertex_count(mesh);
  samples.vertices.reserve(vertices);
  if (mesh.has_normals) samples.normals.reserve(vertices);
  samples.triangles.reserve(triangle_count(mesh));
  uint32_t base = 0;
  for (const auto& strip : mesh.strips) {
    for (const MeshVertex& vertex : strip) {
      samples.vertices.push_back(identity_point(vertex.position));
      if (mesh.has_normals) samples.normals.push_back(identity_point(vertex.normal));
    }
    // cadmpeg-ir tessellation.rs `strip_triangles`.
    if (strip.size() >= 3) {
      for (uint32_t index = 0; index + 2 < strip.size(); ++index) {
        uint32_t a = base + index;
        if (index % 2 == 0) samples.triangles.push_back({a, a + 1, a + 2});
        else samples.triangles.push_back({a, a + 2, a + 1});
      }
    }
    base += static_cast<uint32_t>(strip.size());
  }
  return samples;
}

// ---------------------------------------------------------------------------
// Planes, arcs and angles
// ---------------------------------------------------------------------------
std::optional<PlaneFrame> PlaneFrame::make(Vec3 origin, Vec3 normal, Vec3 u_axis) {
  auto n = unit(normal);
  if (!n) return std::nullopt;
  auto u = unit(sub(u_axis, scale(*n, dot(u_axis, *n))));
  if (!u) return std::nullopt;
  return PlaneFrame{origin, *n, *u};
}

Vec3 PlaneFrame::v_axis() const { return divided(cross(normal, u_axis)); }

Point2 PlaneFrame::project(Vec3 point) const {
  Vec3 delta = sub(point, origin);
  return Point2{dot(delta, u_axis), dot(delta, v_axis())};
}

std::optional<PlaneFrame> plane_frame(const SolvedSurface& surface) {
  if (surface.kind != SurfaceKind::Plane) return std::nullopt;
  return PlaneFrame::make(surface.origin, surface.axis, surface.reference);
}

std::optional<double> cylinder_angle(Vec3 point, Vec3 origin, Vec3 axis, Vec3 reference) {
  auto [r, t] = perpendicular_basis(axis, reference);
  Vec3 delta = sub(point, origin);
  double angle = std::atan2(dot(delta, t), dot(delta, r));
  if (!std::isfinite(angle)) return std::nullopt;
  return rem_euclid(angle, TAU);
}

std::optional<double> cone_angle(Vec3 point, Vec3 origin, Vec3 axis, Vec3 reference,
                                 double ratio) {
  auto [r, t] = perpendicular_basis(axis, reference);
  Vec3 delta = sub(point, origin);
  double angle = std::atan2(dot(delta, t) / ratio, dot(delta, r));
  if (!std::isfinite(angle)) return std::nullopt;
  return rem_euclid(angle, TAU);
}

std::optional<std::pair<double, double>> circular_interval(std::vector<double>& angles) {
  std::erase_if(angles, [](double angle) { return !std::isfinite(angle); });
  if (angles.empty()) return std::nullopt;
  std::sort(angles.begin(), angles.end(), total_less);
  {
    // Vec::dedup_by: drop each angle within EPS of the last kept one.
    size_t kept = 1;
    for (size_t i = 1; i < angles.size(); ++i)
      if (!(std::fabs(angles[i] - angles[kept - 1]) <= EPS_CYLINDER_ANGLE))
        angles[kept++] = angles[i];
    angles.resize(kept);
  }
  if (angles.size() == 1) return std::pair{angles[0], TAU};
  size_t n = angles.size();
  size_t largest_gap_index = 0;
  double largest_gap = 0;
  for (size_t index = 0; index < n; ++index) {
    double next = angles[(index + 1) % n];
    double gap = index + 1 == n ? next + TAU - angles[index] : next - angles[index];
    // Iterator::max_by keeps the last of equal maxima.
    if (index == 0 || !total_less(gap, largest_gap)) {
      largest_gap_index = index;
      largest_gap = gap;
    }
  }
  return std::pair{angles[(largest_gap_index + 1) % n], TAU - largest_gap};
}

bool circular_interval_contains(double start, double span, double angle, double tolerance) {
  double d = rem_euclid(angle - start, TAU);
  return d <= span + tolerance || d + tolerance >= TAU;
}

std::optional<double> shortest_arc_span(double start, double end) {
  double forward = rem_euclid(end - start, TAU);
  double span = forward <= PI ? forward : forward - TAU;
  if (std::fabs(span) > EPS_CYLINDER_ANGLE && std::fabs(span) < PI - EPS_CYLINDER_ANGLE)
    return span;
  return std::nullopt;
}

std::optional<std::pair<size_t, double>> planar_arc_segments(double span, double radius,
                                                             double tolerance) {
  if (!std::isfinite(span)) return std::nullopt;
  double half_chord = std::sqrt(std::fmin(0.5 * (tolerance / radius), 1.0));
  double maximum_span = 4.0 * std::asin(half_chord);
  size_t requested = MAX_PLANAR_TRIM_ARC_SEGMENTS;
  if (std::isfinite(maximum_span) && maximum_span > EPS_CYLINDER_ANGLE) {
    double count = std::ceil(std::fabs(span) / maximum_span);
    // cadmpeg `truncate_f64_to_usize`: none when not representable.
    if (count >= 0.0 && count < 18446744073709551616.0) requested = static_cast<size_t>(count);
  }
  size_t segments = std::clamp<size_t>(requested, 1, MAX_PLANAR_TRIM_ARC_SEGMENTS);
  double actual_span = std::fabs(span) / static_cast<double>(segments);
  double sine = std::sin(actual_span / 4.0);
  return std::pair{segments, (radius * sine) * (2.0 * sine)};
}

namespace {

std::optional<double> ellipse_parameter(Vec3 point, Vec3 center, Vec3 major_direction,
                                        Vec3 minor_direction, double major_radius,
                                        double minor_radius, double tolerance) {
  Vec3 delta = sub(point, center);
  double cosine = dot(delta, major_direction) / major_radius;
  double sine = dot(delta, minor_direction) / minor_radius;
  double normalization = std::hypot(cosine, sine);
  if (std::isfinite(normalization) &&
      std::fabs(normalization - 1.0) <= tolerance / std::fmin(major_radius, minor_radius))
    return rem_euclid(std::atan2(sine, cosine), TAU);
  return std::nullopt;
}

struct PlanarArc {
  Vec3 center, first_direction, second_direction;
  double first_radius, second_radius;

  std::optional<std::pair<std::vector<Point2>, double>> samples(double start_parameter,
                                                                double span,
                                                                const SolvedSurface& surface,
                                                                const PlaneFrame& frame,
                                                                BoundaryTolerance limits) const {
    double radius = std::fmax(first_radius, second_radius);
    auto segments = planar_arc_segments(span, radius, limits.sampling_tolerance);
    if (!segments) return std::nullopt;
    std::vector<Point2> points;
    points.reserve(segments->first);
    for (size_t index = 0; index < segments->first; ++index) {
      double parameter = start_parameter + span * static_cast<double>(index) /
                                               static_cast<double>(segments->first);
      Vec3 point = add(add(center, scale(first_direction, first_radius * std::cos(parameter))),
                       scale(second_direction, second_radius * std::sin(parameter)));
      auto residual = analytic_surface_residual(surface, point);
      if (!residual || *residual > limits.tolerance) return std::nullopt;
      points.push_back(frame.project(point));
    }
    return std::pair{std::move(points), segments->second};
  }
};

}  // namespace

std::optional<std::pair<std::vector<Point2>, double>> planar_boundary_samples(
    const CurveGeometry& curve, Vec3 start, Vec3 end, const SolvedSurface& surface,
    const PlaneFrame& frame, BoundaryTolerance limits) {
  double tolerance = limits.tolerance;
  if (curve.kind == "line") return std::pair{std::vector<Point2>{frame.project(start)}, 0.0};
  bool circle = curve.kind == "circle";
  if (!circle && curve.kind != "ellipse") return std::nullopt;
  Vec3 center = curve.center;
  Vec3 axis = curve.axis;
  Vec3 first_direction, second_direction;
  double first_radius = curve.radius;
  double second_radius = circle ? curve.radius : curve.minor_radius;
  if (circle) {
    std::tie(first_direction, second_direction) = perpendicular_basis(axis, curve.ref_direction);
  } else {
    // EllipseCurve frame: reference = major direction, binormal = axis x reference.
    first_direction = curve.ref_direction;
    second_direction = divided(cross(axis, curve.ref_direction));
  }
  if (std::fabs(dot(axis, frame.normal)) < 1.0 - EPS_AXIS_ALIGNMENT ||
      first_radius <= tolerance || (!circle && second_radius <= tolerance))
    return std::nullopt;
  auto center_residual = analytic_surface_residual(surface, center);
  if (!center_residual || *center_residual > tolerance) return std::nullopt;
  double endpoint_tolerance = std::fmax(tolerance, limits.sampling_tolerance);
  auto start_parameter = ellipse_parameter(start, center, first_direction, second_direction,
                                           first_radius, second_radius, endpoint_tolerance);
  if (!start_parameter) return std::nullopt;
  auto end_parameter = ellipse_parameter(end, center, first_direction, second_direction,
                                         first_radius, second_radius, endpoint_tolerance);
  if (!end_parameter) return std::nullopt;
  auto span = shortest_arc_span(*start_parameter, *end_parameter);
  if (!span) return std::nullopt;
  return PlanarArc{center, first_direction, second_direction, first_radius, second_radius}
      .samples(*start_parameter, *span, surface, frame, limits);
}

// ---------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------
TrimTopology::TrimTopology(const Model& model) {
  // HashMap collection keeps the last entry of a repeated id.
  for (const auto& entry : model.loops) loops[entry.id] = &entry;
  for (const auto& entry : model.coedges) coedges[entry.id] = &entry;
  for (const auto& entry : model.edges) edges[entry.id] = &entry;
  for (const auto& entry : model.vertices) vertices[entry.id] = &entry;
  for (const auto& entry : model.points) points[entry.id] = entry.position;
  for (const auto& entry : model.curves) curves[entry.id] = &entry.geometry;
  coordinate_scale = 1.0;
  for (const auto& [id, point] : points)
    for (double value : {point.x, point.y, point.z})
      coordinate_scale = std::fmax(coordinate_scale, std::fabs(value));
}

namespace {

std::optional<Vec3> vertex_point(const TrimTopology& topology,
                                 const std::optional<std::string>& vertex_id) {
  if (!vertex_id) return std::nullopt;
  auto vertex = find_ptr(topology.vertices, std::string_view(*vertex_id));
  if (!vertex) return std::nullopt;
  auto point = find_ptr(topology.points, std::string_view((*vertex)->point));
  if (!point) return std::nullopt;
  return *point;
}

const CurveGeometry* edge_curve(const TrimTopology& topology, const Edge& edge) {
  if (!edge.curve) return nullptr;
  auto curve = find_ptr(topology.curves, std::string_view(*edge.curve));
  return curve ? *curve : nullptr;
}

/// Residual `> tolerance`, or none when the residual is absent (`?`).
std::optional<bool> residual_exceeds(const SolvedSurface& surface, Vec3 point, double tolerance) {
  auto residual = analytic_surface_residual(surface, point);
  if (!residual) return std::nullopt;
  return *residual > tolerance;
}

std::optional<CircularHole> closed_planar_circle(const Loop& loop, const SolvedSurface& surface,
                                                 const PlaneFrame& frame, double tolerance,
                                                 const TrimTopology& topology) {
  if (loop.coedges.empty()) return std::nullopt;
  auto coedge = find_ptr(topology.coedges, std::string_view(loop.coedges[0]));
  if (!coedge) return std::nullopt;
  auto edge = find_ptr(topology.edges, std::string_view((*coedge)->edge));
  if (!edge) return std::nullopt;
  if ((*coedge)->owner_loop != loop.id || loop.coedges.size() != 1) return std::nullopt;
  const CurveGeometry* curve = edge_curve(topology, **edge);
  if (!curve || curve->kind != "circle") return std::nullopt;
  Vec3 center = curve->center;
  Vec3 axis = curve->axis;
  double radius = curve->radius;
  auto boundary_point = vertex_point(topology, (*edge)->start);
  if (!boundary_point) return std::nullopt;
  auto end_point = vertex_point(topology, (*edge)->end);
  if (!end_point) return std::nullopt;
  if (radius <= tolerance) return std::nullopt;
  if (std::fabs(dot(axis, frame.normal)) < 1.0 - EPS_AXIS_ALIGNMENT) return std::nullopt;
  auto center_exceeds = residual_exceeds(surface, center, tolerance);
  if (!center_exceeds || *center_exceeds) return std::nullopt;
  auto boundary_exceeds = residual_exceeds(surface, *boundary_point, tolerance);
  if (!boundary_exceeds || *boundary_exceeds) return std::nullopt;
  if (distance(*boundary_point, *end_point) > tolerance ||
      std::fabs(distance(*boundary_point, center) - radius) > tolerance)
    return std::nullopt;
  return CircularHole{frame.project(center), radius};
}

}  // namespace

// ---------------------------------------------------------------------------
// Planar trims
// ---------------------------------------------------------------------------
std::optional<PlanarHole> PlanarHole::polygon(std::vector<Point2> boundary, double tolerance,
                                              WorkBudget& work) {
  auto triangles = triangulate_polygon(boundary, tolerance, work);
  if (!triangles) return std::nullopt;
  return PlanarHole{std::nullopt, TrimPolygon(std::move(boundary)), std::move(*triangles)};
}

std::optional<PlanarTrim> planar_trim(const Face& face, const SolvedSurface& surface,
                                      const TrimTopology& topology, WorkBudget& work) {
  auto frame = plane_frame(surface);
  if (!frame) return std::nullopt;
  double tolerance = FACE_EVALUATION_TOLERANCE;
  double sampling_tolerance =
      std::fmax(tolerance, quantization_tolerance_at(topology.coordinate_scale));
  std::vector<std::vector<Point2>> polygons;
  std::vector<CircularHole> circles;
  double boundary_tolerance = 0.0;
  for (const std::string& loop_id : face.loops) {
    auto loop_ptr = find_ptr(topology.loops, std::string_view(loop_id));
    if (!loop_ptr) return std::nullopt;
    const Loop& loop = **loop_ptr;
    if (loop.face != face.id || loop.coedges.empty()) return std::nullopt;
    if (loop.coedges.size() == 1) {
      auto circle = closed_planar_circle(loop, surface, *frame, tolerance, topology);
      if (!circle) return std::nullopt;
      circles.push_back(*circle);
      continue;
    }
    std::vector<Point2> polygon;
    std::optional<Vec3> first_start, previous_end;
    for (const std::string& coedge_id : loop.coedges) {
      auto coedge = find_ptr(topology.coedges, std::string_view(coedge_id));
      if (!coedge) return std::nullopt;
      auto edge = find_ptr(topology.edges, std::string_view((*coedge)->edge));
      if (!edge) return std::nullopt;
      if ((*coedge)->owner_loop != loop.id) return std::nullopt;
      const auto& start_id = (*coedge)->reversed ? (*edge)->end : (*edge)->start;
      const auto& end_id = (*coedge)->reversed ? (*edge)->start : (*edge)->end;
      auto start = vertex_point(topology, start_id);
      if (!start) return std::nullopt;
      auto end = vertex_point(topology, end_id);
      if (!end) return std::nullopt;
      auto start_exceeds = residual_exceeds(surface, *start, tolerance);
      if (!start_exceeds || *start_exceeds) return std::nullopt;
      auto end_exceeds = residual_exceeds(surface, *end, tolerance);
      if (!end_exceeds || *end_exceeds) return std::nullopt;
      if (previous_end && distance(*previous_end, *start) > tolerance) return std::nullopt;
      const CurveGeometry* curve = edge_curve(topology, **edge);
      if (!curve) return std::nullopt;
      auto samples = planar_boundary_samples(*curve, *start, *end, surface, *frame,
                                             BoundaryTolerance{tolerance, sampling_tolerance});
      if (!samples) return std::nullopt;
      polygon.insert(polygon.end(), samples->first.begin(), samples->first.end());
      boundary_tolerance = std::fmax(boundary_tolerance, samples->second);
      if (!first_start) first_start = *start;
      previous_end = *end;
    }
    if (!previous_end || !first_start) return std::nullopt;
    if (distance(*previous_end, *first_start) > tolerance) return std::nullopt;
    polygons.push_back(std::move(polygon));
  }

  PlanarTrim trim;
  trim.frame = *frame;
  trim.boundary_tolerance = boundary_tolerance;
  if (polygons.empty()) {
    if (circles.empty()) return std::nullopt;
    if (!charge_product(work, circles.size(), circles.size())) return std::nullopt;
    auto split = circular_outer_and_holes(circles, tolerance);
    if (!split) return std::nullopt;
    trim.outer = split->first;
    for (const CircularHole& hole : split->second)
      trim.holes.push_back(PlanarHole::make_circle(hole));
    return trim;
  }
  // Every polygon participates in a containment pair when more than one is
  // present. Establish simplicity once before testing pairs.
  if (polygons.size() > 1) {
    for (const auto& polygon : polygons) {
      if (!charge_product(work, polygon.size(), polygon.size()) ||
          !is_simple_polygon(polygon, sampling_tolerance))
        return std::nullopt;
    }
  }
  std::optional<size_t> outer_index;
  for (size_t index = 0; index < polygons.size(); ++index) {
    const auto& outer = polygons[index];
    bool contains_all = true;
    for (size_t inner_index = 0; inner_index < polygons.size(); ++inner_index) {
      if (index == inner_index) continue;
      const auto& inner = polygons[inner_index];
      if (!charge_product(work, inner.size(), outer.size())) return std::nullopt;
      if (!polygon_inside_polygon(inner, outer, sampling_tolerance)) {
        contains_all = false;
        break;
      }
    }
    if (contains_all) {
      for (const CircularHole& circle : circles) {
        if (!work.charge_by(outer.size())) return std::nullopt;
        if (!circle_inside_polygon(outer, circle, sampling_tolerance)) {
          contains_all = false;
          break;
        }
      }
    }
    if (contains_all) {
      if (outer_index) return std::nullopt;
      outer_index = index;
    }
  }
  if (!outer_index) return std::nullopt;
  std::vector<const std::vector<Point2>*> polygon_holes;
  for (size_t index = 0; index < polygons.size(); ++index)
    if (index != *outer_index) polygon_holes.push_back(&polygons[index]);
  for (size_t index = 0; index < polygon_holes.size(); ++index) {
    const auto& left = *polygon_holes[index];
    for (size_t right_index = index + 1; right_index < polygon_holes.size(); ++right_index) {
      const auto& right = *polygon_holes[right_index];
      if (!charge_product(work, left.size(), right.size()) ||
          polygons_overlap(left, right, sampling_tolerance))
        return std::nullopt;
    }
    for (const CircularHole& circle : circles) {
      if (!work.charge_by(left.size()) || circle_overlaps_polygon(circle, left, sampling_tolerance))
        return std::nullopt;
    }
  }
  for (size_t index = 0; index < circles.size(); ++index) {
    if (!work.charge_by(circles.size())) return std::nullopt;
    for (size_t right = index + 1; right < circles.size(); ++right) {
      if (point_distance(circles[index].center, circles[right].center) <
          circles[index].radius + circles[right].radius - sampling_tolerance)
        return std::nullopt;
    }
  }
  for (const CircularHole& circle : circles) trim.holes.push_back(PlanarHole::make_circle(circle));
  for (const auto* polygon : polygon_holes) {
    auto hole = PlanarHole::polygon(*polygon, sampling_tolerance, work);
    if (!hole) return std::nullopt;
    trim.holes.push_back(std::move(*hole));
  }
  trim.outer = TrimPolygon(polygons[*outer_index]);
  return trim;
}

namespace {

std::optional<PlanarTrim> planar_hole_trim(const Face& face, const SolvedSurface& surface,
                                           const TrimTopology& topology) {
  auto frame = plane_frame(surface);
  if (!frame) return std::nullopt;
  double tolerance = FACE_EVALUATION_TOLERANCE;
  bool has_polygon_loop = false;
  for (const std::string& loop_id : face.loops) {
    auto loop = find_ptr(topology.loops, std::string_view(loop_id));
    if (!loop) return std::nullopt;
    has_polygon_loop |= (*loop)->coedges.size() > 1;
  }
  if (!has_polygon_loop) return std::nullopt;
  PlanarTrim trim;
  trim.frame = *frame;
  for (const std::string& loop_id : face.loops) {
    auto loop = find_ptr(topology.loops, std::string_view(loop_id));
    if (!loop) return std::nullopt;
    if ((*loop)->face == face.id && (*loop)->coedges.size() == 1) {
      if (auto circle = closed_planar_circle(**loop, surface, *frame, tolerance, topology))
        trim.holes.push_back(PlanarHole::make_circle(*circle));
    }
  }
  if (trim.holes.empty()) return std::nullopt;
  return trim;
}

/// One axial bound update.
void include_axial(std::optional<std::pair<double, double>>& bounds, double axial) {
  if (bounds) bounds = std::pair{std::fmin(bounds->first, axial), std::fmax(bounds->second, axial)};
  else bounds = std::pair{axial, axial};
}

std::optional<CylindricalTrim> cylindrical_trim(const Face& face, const SolvedSurface& surface,
                                                const TrimTopology& topology) {
  if (surface.kind != SurfaceKind::Cylinder) return std::nullopt;
  Vec3 origin = surface.origin, axis = surface.axis;
  double radius = surface.radius;
  if (radius <= EPS_DISPLAY_QUANTIZATION) return std::nullopt;
  if (face.loops.size() != 1) return std::nullopt;
  auto loop_ptr = find_ptr(topology.loops, std::string_view(face.loops[0]));
  if (!loop_ptr) return std::nullopt;
  const Loop& loop = **loop_ptr;
  if (loop.face != face.id || loop.coedges.empty()) return std::nullopt;
  double tolerance = FACE_EVALUATION_TOLERANCE;
  std::optional<std::pair<double, double>> axial_bounds;
  for (const std::string& coedge_id : loop.coedges) {
    auto coedge = find_ptr(topology.coedges, std::string_view(coedge_id));
    if (!coedge) return std::nullopt;
    if ((*coedge)->owner_loop != loop.id) return std::nullopt;
    auto edge = find_ptr(topology.edges, std::string_view((*coedge)->edge));
    if (!edge) return std::nullopt;
    const CurveGeometry* curve = edge_curve(topology, **edge);
    if (!curve) return std::nullopt;
    if (curve->kind == "line") {
      if (std::fabs(dot(curve->direction, axis)) < 1.0 - EPS_AXIS_ALIGNMENT) return std::nullopt;
    } else if (curve->kind == "circle") {
      if (std::fabs(dot(curve->axis, axis)) < 1.0 - EPS_AXIS_ALIGNMENT ||
          std::fabs(curve->radius - radius) > tolerance)
        return std::nullopt;
    } else {
      return std::nullopt;
    }
    for (const auto* vertex_id : {&(*edge)->start, &(*edge)->end}) {
      auto point = vertex_point(topology, *vertex_id);
      if (!point) return std::nullopt;
      auto exceeds = residual_exceeds(surface, *point, tolerance);
      if (!exceeds || *exceeds) return std::nullopt;
      double axial = dot(sub(*point, origin), axis);
      if (!std::isfinite(axial)) return std::nullopt;
      include_axial(axial_bounds, axial);
    }
  }
  if (!axial_bounds) return std::nullopt;
  std::vector<double> angles;
  for (const std::string& coedge_id : loop.coedges) {
    auto coedge = find_ptr(topology.coedges, std::string_view(coedge_id));
    if (!coedge) return std::nullopt;
    auto edge = find_ptr(topology.edges, std::string_view((*coedge)->edge));
    if (!edge) return std::nullopt;
    for (const auto* vertex_id : {&(*edge)->start, &(*edge)->end}) {
      auto point = vertex_point(topology, *vertex_id);
      if (!point) return std::nullopt;
      auto angle = cylinder_angle(*point, origin, axis, surface.reference);
      if (!angle) return std::nullopt;
      angles.push_back(*angle);
    }
  }
  auto interval = circular_interval(angles);
  if (!interval) return std::nullopt;
  if (!(axial_bounds->second - axial_bounds->first > tolerance)) return std::nullopt;
  return CylindricalTrim{origin,
                         axis,
                         surface.reference,
                         radius,
                         axial_bounds->first,
                         axial_bounds->second,
                         interval->first,
                         interval->second};
}

std::optional<ConicalTrim> conical_trim(const Face& face, const SolvedSurface& surface,
                                        const TrimTopology& topology) {
  if (surface.kind != SurfaceKind::Cone) return std::nullopt;
  Vec3 origin = surface.origin, axis = surface.axis, frame_reference = surface.reference;
  double radius = surface.radius;
  double ratio = surface.ratio;
  double slope = std::tan(surface.half_angle);
  if (radius <= EPS_DISPLAY_QUANTIZATION || !std::isfinite(slope)) return std::nullopt;
  if (face.loops.size() != 1) return std::nullopt;
  auto loop_ptr = find_ptr(topology.loops, std::string_view(face.loops[0]));
  if (!loop_ptr) return std::nullopt;
  const Loop& loop = **loop_ptr;
  if (loop.face != face.id || loop.coedges.empty()) return std::nullopt;
  double tolerance = FACE_EVALUATION_TOLERANCE;
  std::optional<std::pair<double, double>> axial_bounds;
  std::vector<double> angles;
  for (const std::string& coedge_id : loop.coedges) {
    auto coedge = find_ptr(topology.coedges, std::string_view(coedge_id));
    if (!coedge) return std::nullopt;
    if ((*coedge)->owner_loop != loop.id) return std::nullopt;
    auto edge = find_ptr(topology.edges, std::string_view((*coedge)->edge));
    if (!edge) return std::nullopt;
    const CurveGeometry* curve = edge_curve(topology, **edge);
    if (!curve) return std::nullopt;
    if (curve->kind == "line") {
      if (std::fabs(dot(curve->direction, axis)) < 1.0 - EPS_AXIS_ALIGNMENT) return std::nullopt;
    } else if (curve->kind == "nurbs") {
      if (!curve->nurbs) return std::nullopt;
      const NurbsCurveGeometry& nurbs = *curve->nurbs;
      if (nurbs.degree != 1 || nurbs.periodic || nurbs.poles.size() != 2) return std::nullopt;
      for (const Vec3& pole : nurbs.poles) {
        auto residual = analytic_surface_residual(surface, pole);
        if (!residual || *residual > tolerance) return std::nullopt;
      }
    } else if (curve->kind == "ellipse") {
      Vec3 center = curve->center;
      Vec3 major_direction = curve->ref_direction;
      double major_radius = curve->radius;
      double minor_radius = curve->minor_radius;
      auto [reference, transverse] = perpendicular_basis(axis, frame_reference);
      Vec3 center_delta = sub(center, origin);
      double center_axial = dot(center_delta, axis);
      Vec3 center_radial = sub(center_delta, scale(axis, center_axial));
      double expected_radius = std::fabs(radius + center_axial * slope);
      double reference_aligned = std::fabs(dot(major_direction, reference));
      double transverse_aligned = std::fabs(dot(major_direction, transverse));
      bool aligned_radii = false;
      if (reference_aligned >= 1.0 - EPS_AXIS_ALIGNMENT) {
        aligned_radii = std::fabs(major_radius - expected_radius) <= tolerance &&
                        std::fabs(minor_radius - expected_radius * ratio) <= tolerance;
      } else if (transverse_aligned >= 1.0 - EPS_AXIS_ALIGNMENT) {
        aligned_radii = std::fabs(major_radius - expected_radius * ratio) <= tolerance &&
                        std::fabs(minor_radius - expected_radius) <= tolerance;
      }
      if (std::fabs(dot(curve->axis, axis)) < 1.0 - EPS_AXIS_ALIGNMENT ||
          major_radius <= tolerance || minor_radius <= tolerance ||
          norm(center_radial) > tolerance || !aligned_radii)
        return std::nullopt;
    } else if (curve->kind == "circle") {
      if (std::fabs(ratio - 1.0) > EPS_AXIS_ALIGNMENT ||
          std::fabs(dot(curve->axis, axis)) < 1.0 - EPS_AXIS_ALIGNMENT)
        return std::nullopt;
      Vec3 center_delta = sub(curve->center, origin);
      double center_axial = dot(center_delta, axis);
      Vec3 center_radial = sub(center_delta, scale(axis, center_axial));
      double expected_radius = std::fabs(radius + center_axial * slope);
      if (norm(center_radial) > tolerance || std::fabs(curve->radius - expected_radius) > tolerance)
        return std::nullopt;
    } else {
      return std::nullopt;
    }
    for (const auto* vertex_id : {&(*edge)->start, &(*edge)->end}) {
      auto point = vertex_point(topology, *vertex_id);
      if (!point) return std::nullopt;
      auto exceeds = residual_exceeds(surface, *point, tolerance);
      if (!exceeds || *exceeds) return std::nullopt;
      double axial = dot(sub(*point, origin), axis);
      auto angle = cone_angle(*point, origin, axis, frame_reference, ratio);
      if (!angle) return std::nullopt;
      if (!std::isfinite(axial) || !std::isfinite(*angle)) return std::nullopt;
      include_axial(axial_bounds, axial);
      angles.push_back(*angle);
    }
  }
  if (!axial_bounds) return std::nullopt;
  auto interval = circular_interval(angles);
  if (!interval) return std::nullopt;
  if (!(axial_bounds->second - axial_bounds->first > tolerance)) return std::nullopt;
  return ConicalTrim{origin,
                     axis,
                     frame_reference,
                     radius,
                     ratio,
                     slope,
                     axial_bounds->first,
                     axial_bounds->second,
                     interval->first,
                     interval->second};
}

}  // namespace

std::optional<AnalyticTrim> analytic_trim(const Face& face, const SolvedSurface& surface,
                                          const TrimTopology& topology, WorkBudget& work) {
  switch (surface.kind) {
    case SurfaceKind::Plane: {
      auto trim = planar_trim(face, surface, topology, work);
      if (!trim) trim = planar_hole_trim(face, surface, topology);
      if (!trim) return std::nullopt;
      return AnalyticTrim{std::move(*trim)};
    }
    case SurfaceKind::Cylinder:
      if (auto trim = cylindrical_trim(face, surface, topology)) return AnalyticTrim{*trim};
      return std::nullopt;
    case SurfaceKind::Cone:
      if (auto trim = conical_trim(face, surface, topology)) return AnalyticTrim{*trim};
      return std::nullopt;
    default:
      return std::nullopt;
  }
}

// ---------------------------------------------------------------------------
// Trim containment
// ---------------------------------------------------------------------------
bool CylindricalTrim::contains_mesh(const MeshSamples& mesh, double tolerance) const {
  for (const Vec3& point : mesh.vertices) {
    double axial = dot(sub(point, origin), axis);
    auto angular = cylinder_angle(point, origin, axis, reference);
    bool inside = std::isfinite(axial) && axial >= min_axial - tolerance &&
                  axial <= max_axial + tolerance && angular &&
                  (angular_span >= TAU - EPS_CYLINDER_ANGLE ||
                   circular_interval_contains(angular_start, angular_span, *angular,
                                              tolerance / radius));
    if (!inside) return false;
  }
  return true;
}

bool ConicalTrim::contains_mesh(const MeshSamples& mesh, double tolerance) const {
  for (const Vec3& point : mesh.vertices) {
    double axial = dot(sub(point, origin), axis);
    double local_radius = radius + axial * slope;
    auto angular = cone_angle(point, origin, axis, reference, ratio);
    bool inside =
        std::isfinite(axial) && std::isfinite(local_radius) && axial >= min_axial - tolerance &&
        axial <= max_axial + tolerance && angular &&
        (angular_span >= TAU - EPS_CYLINDER_ANGLE ||
         circular_interval_contains(
             angular_start, angular_span, *angular,
             tolerance / std::fmax(std::fabs(local_radius) * std::fmin(ratio, 1.0),
                                   EPS_DISPLAY_QUANTIZATION)));
    if (!inside) return false;
  }
  return true;
}

bool AnalyticTrim::contains_mesh(const MeshSamples& mesh, double tolerance,
                                 WorkBudget& work) const {
  if (auto planar = std::get_if<PlanarTrim>(&trim))
    return planar->contains_mesh(mesh, tolerance, work);
  if (!work.charge_by(mesh.vertices.size())) return false;
  if (auto cylindrical = std::get_if<CylindricalTrim>(&trim))
    return cylindrical->contains_mesh(mesh, tolerance);
  return std::get<ConicalTrim>(trim).contains_mesh(mesh, tolerance);
}

namespace {

/// tessellation.rs `HoleConstraint`.
struct HoleConstraint {
  const TrimPolygon* boundary = nullptr;  ///< polygon hole
  const std::vector<Triangle2>* triangles = nullptr;
  CircularHole exclusion, circle_boundary;  ///< circle hole

  bool contains_interior(Point2 point, double tolerance) const {
    if (boundary) return boundary->strictly_contains(point, tolerance);
    return point_distance(point, exclusion.center) < exclusion.radius - tolerance;
  }
  bool crosses_triangle(const Triangle2& triangle, double tolerance) const {
    if (boundary) {
      for (const Triangle2& hole_triangle : *triangles)
        if (triangles_have_positive_overlap(triangle, hole_triangle, tolerance)) return true;
      return false;
    }
    return triangle_crosses_hole(triangle, exclusion, circle_boundary, tolerance);
  }
};

}  // namespace

bool PlanarTrim::contains_mesh(const MeshSamples& mesh, double tolerance, WorkBudget& work) const {
  tolerance += boundary_tolerance;
  const TrimPolygon* outer_polygon_boundary =
      outer ? std::get_if<TrimPolygon>(&*outer) : nullptr;
  const CircularHole* outer_circle = outer ? std::get_if<CircularHole>(&*outer) : nullptr;
  size_t point_edges = outer_polygon_boundary ? outer_polygon_boundary->points().size() : 1;
  for (const PlanarHole& hole : holes) {
    size_t edges = hole.boundary ? hole.boundary->points().size() : 1;
    if (__builtin_add_overflow(point_edges, edges, &point_edges)) {
      work.exhaust();
      return false;
    }
  }
  if (!charge_product(work, mesh.vertices.size(), point_edges)) return false;
  std::vector<Point2> projected;
  projected.reserve(mesh.vertices.size());
  for (const Vec3& point : mesh.vertices) projected.push_back(frame.project(point));
  std::vector<HoleConstraint> constraints;
  constraints.reserve(holes.size());
  for (const PlanarHole& hole : holes) {
    HoleConstraint constraint;
    if (hole.boundary) {
      constraint.boundary = &*hole.boundary;
      constraint.triangles = &hole.triangles;
    } else {
      auto chordal = chordal_hole_constraint(*hole.circle, projected, tolerance);
      if (!chordal) return false;
      constraint.exclusion = chordal->first;
      constraint.circle_boundary = chordal->second;
    }
    constraints.push_back(constraint);
  }
  for (const Point2& point : projected) {
    if (outer_polygon_boundary && !outer_polygon_boundary->contains(point, tolerance)) return false;
    if (outer_circle && !(point_distance(point, outer_circle->center) <=
                          outer_circle->radius + tolerance))
      return false;
    for (const HoleConstraint& hole : constraints)
      if (hole.contains_interior(point, tolerance)) return false;
  }
  std::optional<CheckedPlanarPolygon> outer_polygon;
  if (!mesh.triangles.empty() && outer_polygon_boundary) {
    outer_polygon = CheckedPlanarPolygon::make(*outer_polygon_boundary, tolerance, work);
    if (!outer_polygon) return false;
  }
  size_t hole_triangles = 0;
  for (const PlanarHole& hole : holes) {
    size_t count = hole.boundary ? hole.triangles.size() : 1;
    if (__builtin_add_overflow(hole_triangles, count, &hole_triangles)) {
      work.exhaust();
      return false;
    }
  }
  if (!charge_product(work, mesh.triangles.size(), hole_triangles)) return false;
  for (const auto& triangle : mesh.triangles) {
    if (triangle[0] >= projected.size() || triangle[1] >= projected.size() ||
        triangle[2] >= projected.size())
      return false;
    Triangle2 corners{projected[triangle[0]], projected[triangle[1]], projected[triangle[2]]};
    if (outer_polygon && !outer_polygon->contains_triangle(corners)) return false;
    for (const HoleConstraint& hole : constraints)
      if (hole.crosses_triangle(corners, tolerance)) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Owner assignment
// ---------------------------------------------------------------------------
namespace {

/// tessellation.rs `face_bodies`: the body of every face whose shell and
/// region resolve.
std::unordered_map<std::string_view, const std::string*> face_bodies(const Model& model) {
  std::unordered_map<std::string_view, const std::string*> regions;
  for (const auto& region : model.regions) regions[region.id] = &region.body;
  std::unordered_map<std::string_view, const std::string*> shells;
  for (const auto& shell : model.shells)
    if (auto body = find_ptr(regions, std::string_view(shell.region))) shells[shell.id] = *body;
  std::unordered_map<std::string_view, const std::string*> faces;
  for (const auto& face : model.faces)
    if (auto body = find_ptr(shells, std::string_view(face.shell))) faces[face.id] = *body;
  return faces;
}

/// tessellation.rs `unique_by_key`: each key's value, or none when the key
/// occurs with two different values.
template <typename K, typename V, typename Pairs>
std::map<K, std::optional<V>> unique_by_key(const Pairs& pairs) {
  std::map<K, std::optional<V>> found;
  for (const auto& [key, value] : pairs) {
    auto [it, inserted] = found.try_emplace(key, value);
    if (!inserted && it->second && *it->second != value) it->second.reset();
  }
  return found;
}

bool needs_surface_owner(const Tessellation& mesh) {
  return !mesh.body && mesh.faces.empty() && vertex_count(mesh) != 0;
}

void assign_owner(Tessellation& mesh, const std::string& face, const std::string& body) {
  mesh.faces.push_back(face);
  mesh.body = body;
}

struct SurfaceCandidate {
  const std::string* face;
  const std::string* body;
  const std::optional<SolvedSurface>* surface;
  double tolerance;
  std::optional<AnalyticTrim> trim;
};

bool is_planar_surface(const SolvedSurface& surface) { return surface.kind == SurfaceKind::Plane; }
bool contains_nurbs_surface(const SolvedSurface& surface) {
  return surface.kind == SurfaceKind::Nurbs;
}

/// tessellation.rs `approximate_trimmed_surface_owner`.
std::optional<std::pair<size_t, double>> approximate_trimmed_surface_owner(
    const MeshSamples& mesh, const std::vector<SurfaceCandidate>& candidates,
    double quantization_tolerance, WorkBudget& work) {
  std::vector<std::pair<size_t, double>> fits;
  for (size_t index = 0; index < candidates.size(); ++index) {
    const SurfaceCandidate& candidate = candidates[index];
    if (!candidate.trim) continue;
    double max_residual = 0.0;
    bool ok = true;
    for (const Vec3& point : mesh.vertices) {
      if (!work.charge() || !*candidate.surface) {
        ok = false;
        break;
      }
      auto measure = surface_measure_with(**candidate.surface, point, std::nullopt, false);
      if (!measure) {
        ok = false;
        break;
      }
      max_residual = std::fmax(max_residual, measure->residual);
    }
    if (!ok || !*candidate.surface) continue;
    if (is_planar_surface(**candidate.surface) && max_residual > quantization_tolerance) continue;
    if (candidate.trim->contains_mesh(mesh, quantization_tolerance, work))
      fits.emplace_back(index, max_residual);
  }
  std::stable_sort(fits.begin(), fits.end(),
                   [](const auto& l, const auto& r) { return total_less(l.second, r.second); });
  if (fits.empty()) return std::nullopt;
  double best_deflection = fits[0].second;
  std::erase_if(fits, [&](const auto& fit) {
    return !(fit.second <= best_deflection + quantization_tolerance);
  });
  if (fits.size() != 1) return std::nullopt;
  return fits[0];
}

/// tessellation.rs `approximate_surface_owner`.
std::optional<std::pair<size_t, double>> approximate_surface_owner(
    const MeshSamples& mesh, const std::vector<SurfaceCandidate>& candidates,
    double quantization_tolerance, WorkBudget& work) {
  if (mesh.normals.size() != mesh.vertices.size() || mesh.normals.empty()) return std::nullopt;
  // The identity image of each shading normal, made unit once per mesh.
  std::vector<std::optional<std::optional<Vec3>>> unit_normals(mesh.normals.size());
  std::vector<std::pair<size_t, double>> fits;
  for (size_t index = 0; index < candidates.size(); ++index) {
    const SurfaceCandidate& candidate = candidates[index];
    double max_residual = 0.0;
    bool ok = true;
    for (size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) {
      if (!work.charge() || !*candidate.surface) {
        ok = false;
        break;
      }
      auto measure = surface_measure(**candidate.surface, mesh.vertices[vertex], std::nullopt);
      if (!measure || !measure->normal) {
        ok = false;
        break;
      }
      auto& cached = unit_normals[vertex];
      if (!cached) cached = unit(mesh.normals[vertex]);
      if (!*cached) {
        ok = false;
        break;
      }
      if (std::fabs(dot(*measure->normal, **cached)) < MIN_TESSELLATION_NORMAL_ALIGNMENT) {
        ok = false;
        break;
      }
      max_residual = std::fmax(max_residual, measure->residual);
    }
    if (!ok || !*candidate.surface) continue;
    if (is_planar_surface(**candidate.surface) && max_residual > quantization_tolerance) continue;
    fits.emplace_back(index, max_residual);
  }
  if (fits.empty())
    return approximate_trimmed_surface_owner(mesh, candidates, quantization_tolerance, work);
  std::stable_sort(fits.begin(), fits.end(),
                   [](const auto& l, const auto& r) { return total_less(l.second, r.second); });
  double best_deflection = fits[0].second;
  std::erase_if(fits, [&](const auto& fit) {
    return !(fit.second <= best_deflection + quantization_tolerance);
  });
  std::erase_if(fits, [&](const auto& fit) {
    const SurfaceCandidate& candidate = candidates[fit.first];
    return candidate.trim &&
           !candidate.trim->contains_mesh(mesh, quantization_tolerance, work);
  });
  if (fits.empty()) return std::nullopt;
  if (fits.size() > 1 && fits[1].second <= fits[0].second + quantization_tolerance)
    return std::nullopt;
  return fits[0];
}

}  // namespace

std::vector<std::string> assign_face_key_owners(
    Model& model, const std::vector<std::pair<std::string, uint32_t>>& face_keys,
    const std::vector<FaceKeyBinding>& bindings) {
  std::vector<std::pair<uint32_t, std::string_view>> face_pairs, table_pairs;
  face_pairs.reserve(face_keys.size());
  for (const auto& [face, key] : face_keys) face_pairs.emplace_back(key, face);
  table_pairs.reserve(bindings.size());
  for (const auto& binding : bindings) table_pairs.emplace_back(binding.key, binding.tessellation);
  auto faces_by_key = unique_by_key<uint32_t, std::string_view>(face_pairs);
  auto tables_by_key = unique_by_key<uint32_t, std::string_view>(table_pairs);
  auto bodies = face_bodies(model);
  std::unordered_map<std::string_view, std::pair<std::string_view, const std::string*>> owners;
  for (const auto& [key, table] : tables_by_key) {
    if (!table) continue;
    auto face = find_ptr(faces_by_key, key);
    if (!face || !*face) continue;
    if (auto body = find_ptr(bodies, **face)) owners[*table] = {**face, *body};
  }
  std::vector<std::string> assigned;
  for (auto& mesh : model.tessellations) {
    if (mesh.body || !mesh.faces.empty()) continue;
    auto owner = find_ptr(owners, std::string_view(mesh.id));
    if (!owner) continue;
    assign_owner(mesh, std::string(owner->first), *owner->second);
    assigned.push_back(mesh.id);
  }
  return assigned;
}

std::vector<std::string> assign_persistent_owners(
    Model& model, const std::vector<std::pair<std::string, PersistentFaceIdentity>>& identities,
    const std::vector<PersistentFaceBinding>& bindings) {
  std::map<PersistentFaceIdentity, std::optional<std::string_view>> faces_by_identity;
  for (const auto& [target, identity] : identities) {
    auto [it, inserted] = faces_by_identity.try_emplace(identity, target);
    if (!inserted && it->second && *it->second != target) it->second.reset();
  }
  auto bodies = face_bodies(model);
  std::unordered_map<std::string_view, std::optional<const PersistentFaceIdentity*>> by_mesh;
  for (const auto& binding : bindings) {
    auto [it, inserted] = by_mesh.try_emplace(binding.tessellation, &binding.identity);
    if (!inserted && !(it->second && **it->second == binding.identity)) it->second.reset();
  }
  std::vector<std::string> assigned;
  for (auto& mesh : model.tessellations) {
    if (mesh.body || !mesh.faces.empty()) continue;
    auto identity = find_ptr(by_mesh, std::string_view(mesh.id));
    if (!identity || !*identity) continue;
    auto face = find_ptr(faces_by_identity, ***identity);
    if (!face || !*face) continue;
    auto body = find_ptr(bodies, **face);
    if (!body) continue;
    assign_owner(mesh, std::string(**face), **body);
    assigned.push_back(mesh.id);
  }
  return assigned;
}

std::optional<size_t> tessellation_owner_rows(const Model& model) {
  size_t rows = 0;
  if (__builtin_add_overflow(model.faces.size(), model.coedges.size(), &rows)) return std::nullopt;
  for (const auto& mesh : model.tessellations) {
    if (__builtin_add_overflow(rows, vertex_count(mesh), &rows)) return std::nullopt;
    if (__builtin_add_overflow(rows, triangle_count(mesh), &rows)) return std::nullopt;
  }
  return rows;
}

size_t tessellation_owner_work_limit(std::optional<size_t> rows) {
  size_t work = 0;
  if (!rows || __builtin_mul_overflow(*rows, TESSELLATION_OWNER_WORK_PER_ROW, &work))
    return TESSELLATION_OWNER_WORK;
  return std::max(work, TESSELLATION_OWNER_WORK);
}

SurfaceOwnerResult assign_unique_surface_owners(Model& model) {
  return assign_unique_surface_owners_within(
      model, tessellation_owner_work_limit(tessellation_owner_rows(model)));
}

// tessellation.rs `assign_unique_surface_owners_within`.
SurfaceOwnerResult assign_unique_surface_owners_within(Model& model, size_t work_limit) {
  SurfaceOwnerResult result;
  bool any = false;
  for (const auto& mesh : model.tessellations) any |= needs_surface_owner(mesh);
  if (!any) return result;

  std::vector<std::optional<SolvedSurface>> solved;
  solved.reserve(model.surfaces.size());
  std::unordered_map<std::string_view, size_t> surfaces;
  for (size_t i = 0; i < model.surfaces.size(); ++i) {
    solved.push_back(solved_surface(model.surfaces[i].geometry));
    surfaces[model.surfaces[i].id] = i;
  }
  std::unordered_map<std::string_view, const std::string*> regions;
  for (const auto& region : model.regions) regions[region.id] = &region.body;
  std::unordered_map<std::string_view, const std::string*> shell_bodies;
  for (const auto& shell : model.shells)
    if (auto body = find_ptr(regions, std::string_view(shell.region))) shell_bodies[shell.id] = *body;
  TrimTopology topology(model);
  WorkBudget work(work_limit);

  std::vector<SurfaceCandidate> candidates;
  for (const auto& face : model.faces) {
    auto body = find_ptr(shell_bodies, std::string_view(face.shell));
    if (!body) continue;
    if (!face.surface) continue;
    auto surface_index = find_ptr(surfaces, std::string_view(*face.surface));
    if (!surface_index) continue;
    SurfaceCandidate candidate{&face.id, *body, &solved[*surface_index], 0.0, std::nullopt};
    if (*candidate.surface) candidate.trim = analytic_trim(face, **candidate.surface, topology, work);
    candidates.push_back(std::move(candidate));
  }

  std::vector<const SurfaceCandidate*> owners;
  for (auto& mesh : model.tessellations) {
    if (!needs_surface_owner(mesh)) continue;
    // A trim or containment test cut short by the budget is not an
    // ownership answer, so an exhausted pass assigns nothing further.
    if (work.exhausted()) {
      ++result.capped;
      continue;
    }
    MeshSamples samples = MeshSamples::of(mesh);
    double coordinate_scale = 1.0;
    for (const Vec3& point : samples.vertices)
      for (double value : {point.x, point.y, point.z})
        coordinate_scale = std::fmax(coordinate_scale, std::fabs(value));
    double quantization_tolerance = quantization_tolerance_at(coordinate_scale);
    owners.clear();
    for (const SurfaceCandidate& candidate : candidates) {
      if (!work.charge()) break;
      double tolerance = std::fmax(candidate.tolerance, quantization_tolerance);
      if (!*candidate.surface) continue;
      const SolvedSurface& surface = **candidate.surface;
      bool fits = true;
      for (const Vec3& point : samples.vertices) {
        if (!work.charge()) {
          fits = false;
          break;
        }
        auto residual = surface_residual(surface, point, tolerance);
        if (!residual || *residual > tolerance) {
          fits = false;
          break;
        }
      }
      if (fits && (!candidate.trim || candidate.trim->contains_mesh(samples, tolerance, work)))
        owners.push_back(&candidate);
    }
    if (work.exhausted()) {
      ++result.capped;
      continue;
    }
    const SurfaceCandidate* owner = nullptr;
    std::optional<double> deflection;
    if (owners.size() == 1) {
      owner = owners[0];
    } else {
      bool nurbs = false;
      for (const SurfaceCandidate* candidate : owners)
        nurbs |= *candidate->surface && contains_nurbs_surface(**candidate->surface);
      if (nurbs) continue;
      auto approximate =
          approximate_surface_owner(samples, candidates, quantization_tolerance, work);
      if (work.exhausted()) {
        ++result.capped;
        continue;
      }
      if (!approximate) continue;
      owner = &candidates[approximate->first];
      deflection = approximate->second;
    }
    assign_owner(mesh, *owner->face, *owner->body);
    result.assigned.push_back(mesh.id);
    if (deflection) result.deflections.emplace_back(mesh.id, *deflection);
  }
  return result;
}

}  // namespace ownership

// decode.rs: assign_face_key_owners, assign_persistent_owners, then
// assign_unique_surface_owners. The metadata-only path has no face keys and
// no face identities, so the first two bind nothing there, as in cadmpeg.
void assign_tessellation_owners(Model& model, const BrepResult& brep,
                                const std::vector<FaceKeyBinding>& face_keys,
                                const std::vector<PersistentFaceBinding>& persistent) {
  ownership::assign_face_key_owners(model, brep.face_keys, face_keys);
  ownership::assign_persistent_owners(model, brep.face_identities, persistent);
  ownership::assign_unique_surface_owners(model);
}

}  // namespace swglb
