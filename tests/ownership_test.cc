// SPDX-License-Identifier: Apache-2.0
// Tessellation owner tests, ported from cadmpeg-codec-sldprt
// src/tessellation/tests (mod.rs, arc_trims.rs, cone_supports.rs,
// geometry_predicates.rs, planar_trim.rs, owner_fit.rs, pole_box.rs,
// edge_bands.rs) with synthetic models. Rust triangle-list meshes become
// strips spanning the same triangles.
#include <cstdio>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "ownership_internal.h"

using namespace swglb;
using namespace swglb::ownership;

namespace {

int failures = 0;
#define CHECK(condition)                                                              \
  do {                                                                                \
    if (!(condition)) {                                                               \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
      ++failures;                                                                     \
    }                                                                                 \
  } while (0)

constexpr double FRAC_PI_2 = PI / 2;
constexpr double FRAC_PI_4 = PI / 4;
constexpr double FRAC_PI_6 = PI / 6;
constexpr size_t UNBOUNDED = std::numeric_limits<size_t>::max();
const std::string BODY = "synthetic:test:body#body";
const std::string REGION = "synthetic:test:region#region";
const std::string SHELL = "synthetic:test:shell#shell";

Vec3 P(double x, double y, double z) { return {x, y, z}; }

SurfaceGeometry plane(Vec3 origin, Vec3 normal, Vec3 u_axis) {
  SurfaceGeometry g;
  g.kind = "plane";
  g.origin = origin;
  g.normal = normal;
  g.u_axis = u_axis;
  return g;
}
SurfaceGeometry z_plane() { return plane(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0)); }
SurfaceGeometry cylinder(Vec3 origin, Vec3 axis, Vec3 reference, double radius) {
  SurfaceGeometry g = plane(origin, axis, reference);
  g.kind = "cylinder";
  g.radius = radius;
  return g;
}
SurfaceGeometry sphere(Vec3 center, Vec3 axis, Vec3 reference, double radius) {
  SurfaceGeometry g = cylinder(center, axis, reference, radius);
  g.kind = "sphere";
  return g;
}
SurfaceGeometry torus(Vec3 center, Vec3 axis, Vec3 reference, double major, double minor) {
  SurfaceGeometry g = cylinder(center, axis, reference, major);
  g.kind = "torus";
  g.minor_radius = minor;
  return g;
}
SurfaceGeometry cone(Vec3 origin, Vec3 axis, Vec3 reference, double radius, double ratio,
                     double half_angle) {
  SurfaceGeometry g = cylinder(origin, axis, reference, radius);
  g.kind = "cone";
  g.ratio = ratio;
  g.half_angle = half_angle;
  return g;
}
SolvedSurface solved(const SurfaceGeometry& g) { return *solved_surface(g); }

CurveGeometry line_curve(Vec3 origin, Vec3 direction) {
  CurveGeometry c;
  c.kind = "line";
  c.origin = origin;
  c.direction = direction;
  return c;
}
CurveGeometry circle_curve(Vec3 center, Vec3 axis, Vec3 reference, double radius) {
  CurveGeometry c;
  c.kind = "circle";
  c.center = center;
  c.axis = axis;
  c.ref_direction = reference;
  c.radius = radius;
  return c;
}
CurveGeometry ellipse_curve(Vec3 center, Vec3 axis, Vec3 major, double major_radius,
                            double minor_radius) {
  CurveGeometry c = circle_curve(center, axis, major, major_radius);
  c.kind = "ellipse";
  c.minor_radius = minor_radius;
  return c;
}

Model model_with_body() {
  Model model;
  model.bodies.push_back(Body{BODY, "solid", {REGION}, std::nullopt});
  model.regions.push_back(Region{REGION, BODY, {SHELL}});
  return model;
}

void set_shell_faces(Model& model, std::vector<std::string> faces) {
  model.shells = {Shell{SHELL, REGION, std::move(faces)}};
}

/// mod.rs `add_face` with line edges between `corners`, or the given curves.
std::string add_face(Model& model, const std::string& name, SurfaceGeometry geometry,
                     const std::array<Vec3, 4>& corners,
                     std::optional<std::array<CurveGeometry, 4>> curves = std::nullopt,
                     bool distinct_vertices = true) {
  std::string face_id = "synthetic:test:face#face-" + name;
  std::string loop_id = "synthetic:test:loop#loop-" + name;
  std::string surface_id = "synthetic:test:surface#surface-" + name;
  model.surfaces.push_back(Surface{surface_id, std::move(geometry)});
  std::vector<std::string> coedges;
  for (int index = 0; index < 4; ++index)
    coedges.push_back("synthetic:test:coedge#coedge-" + name + "-" + std::to_string(index));
  auto vertex_id = [&](int index) {
    return "synthetic:test:vertex#vertex-" + name + "-" + std::to_string(index);
  };
  for (int index = 0; index < 4; ++index) {
    std::string point_id = "synthetic:test:point#point-" + name + "-" + std::to_string(index);
    model.points.push_back(Point{point_id, corners[index]});
    model.vertices.push_back(Vertex{vertex_id(index), point_id});
  }
  (void)distinct_vertices;
  for (int index = 0; index < 4; ++index) {
    int next = (index + 1) % 4;
    std::string curve_id = "synthetic:test:curve#curve-" + name + "-" + std::to_string(index);
    std::string edge_id = "synthetic:test:edge#edge-" + name + "-" + std::to_string(index);
    CurveGeometry curve =
        curves ? (*curves)[index]
               : line_curve(corners[index], *unit(sub(corners[next], corners[index])));
    model.curves.push_back(Curve{curve_id, curve});
    Edge edge;
    edge.id = edge_id;
    edge.curve = curve_id;
    edge.start = vertex_id(index);
    edge.end = vertex_id(next);
    model.edges.push_back(edge);
    model.coedges.push_back(Coedge{coedges[index], loop_id, edge_id, false});
  }
  model.loops.push_back(Loop{loop_id, face_id, coedges});
  Face face;
  face.id = face_id;
  face.shell = SHELL;
  face.surface = surface_id;
  face.loops = {loop_id};
  model.faces.push_back(face);
  return face_id;
}

std::string add_square_face(Model& model, const std::string& name, double x) {
  return add_face(model, name, z_plane(),
                  {P(x, -1, 0), P(x + 2, -1, 0), P(x + 2, 1, 0), P(x, 1, 0)});
}

std::string add_cylindrical_patch_face(Model& model, const std::string& name, double min_z,
                                       double max_z) {
  double radius = 5.0;
  auto point_at = [&](double angle, double z) {
    return P(radius * std::cos(angle), radius * std::sin(angle), z);
  };
  std::array<Vec3, 4> corners{point_at(0, min_z), point_at(FRAC_PI_2, min_z),
                              point_at(FRAC_PI_2, max_z), point_at(0, max_z)};
  std::array<CurveGeometry, 4> curves{
      circle_curve(P(0, 0, min_z), P(0, 0, 1), P(1, 0, 0), radius),
      line_curve(corners[1], P(0, 0, 1)),
      circle_curve(P(0, 0, max_z), P(0, 0, 1), P(1, 0, 0), radius),
      line_curve(corners[3], P(0, 0, -1))};
  return add_face(model, name, cylinder(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), radius), corners,
                  curves);
}

/// A mesh of the given strips; `normals` (one per strip vertex) shades it.
Tessellation mesh_strips(const std::string& id, const std::vector<std::vector<Vec3>>& strips,
                         const std::vector<std::vector<Vec3>>& normals = {}) {
  Tessellation mesh;
  mesh.id = id;
  mesh.has_normals = !normals.empty();
  for (size_t s = 0; s < strips.size(); ++s) {
    std::vector<MeshVertex> strip;
    for (size_t v = 0; v < strips[s].size(); ++v)
      strip.push_back({strips[s][v], normals.empty() ? Vec3{} : normals[s][v]});
    mesh.strips.push_back(std::move(strip));
  }
  return mesh;
}
Tessellation triangle_mesh(const std::string& id, std::vector<Vec3> vertices,
                           std::vector<Vec3> normals = {}) {
  return mesh_strips(id, {vertices}, normals.empty() ? std::vector<std::vector<Vec3>>{}
                                                     : std::vector<std::vector<Vec3>>{normals});
}
Tessellation persistent_mesh(const std::string& id) {
  return triangle_mesh(id, {P(0, 0, 0), P(1, 0, 0), P(0, 1, 0)});
}
MeshSamples samples_of(const Tessellation& mesh) { return MeshSamples::of(mesh); }

PersistentFaceIdentity persistent_identity(uint32_t source, uint32_t local,
                                           std::vector<uint32_t> trailing) {
  return PersistentFaceIdentity{source, local, std::move(trailing)};
}

NurbsSurfaceGeometry test_nurbs_surface() {
  double heights[9] = {0.0, 0.25, 0.0, 0.25, 0.9, 0.25, 0.0, 0.25, 0.0};
  NurbsSurfaceGeometry s;
  s.u_degree = s.v_degree = 2;
  s.u_knots = s.v_knots = {0, 0, 0, 1, 1, 1};
  s.poles.resize(3);
  for (int u = 0; u < 3; ++u)
    for (int v = 0; v < 3; ++v) s.poles[u].push_back(P(u, v, heights[u * 3 + v]));
  return s;
}
NurbsSurfaceGeometry flat_test_nurbs_surface() {
  NurbsSurfaceGeometry s = test_nurbs_surface();
  for (auto& row : s.poles)
    for (auto& pole : row) pole.z = 0;
  return s;
}
SurfaceGeometry nurbs_geometry(const NurbsSurfaceGeometry& s) {
  SurfaceGeometry g;
  g.kind = "nurbs";
  g.nurbs = s;
  return g;
}
std::array<Vec3, 4> test_nurbs_corners(const NurbsSurfaceGeometry& s) {
  return {*nurbs_surface_point(s, 0, 0), *nurbs_surface_point(s, 1, 0),
          *nurbs_surface_point(s, 1, 1), *nurbs_surface_point(s, 0, 1)};
}
std::pair<Vec3, Vec3> test_nurbs_point_normal(const NurbsSurfaceGeometry& s, double u, double v) {
  auto partials = *nurbs_surface_partials(s, u, v);
  return {partials.point, *unit(cross(partials.du, partials.dv))};
}
std::vector<Vec3> nurbs_points(const NurbsSurfaceGeometry& s) {
  return {*nurbs_surface_point(s, 0.15, 0.2), *nurbs_surface_point(s, 0.8, 0.2),
          *nurbs_surface_point(s, 0.5, 0.8)};
}

const Tessellation& mesh0(const Model& model) { return model.tessellations[0]; }
bool unowned(const Tessellation& mesh) { return !mesh.body && mesh.faces.empty(); }
bool owned_by(const Tessellation& mesh, const std::string& face) {
  return mesh.faces == std::vector<std::string>{face} && mesh.body == BODY;
}
std::vector<std::string> ids(std::initializer_list<const char*> list) {
  return std::vector<std::string>(list.begin(), list.end());
}

PlaneFrame z_frame() { return *PlaneFrame::make(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0)); }
std::vector<Point2> pts(std::initializer_list<std::array<double, 2>> list) {
  std::vector<Point2> out;
  for (auto p : list) out.push_back({p[0], p[1]});
  return out;
}
std::vector<Point2> circle_polygon(size_t count, double radius, double cu = 0, double cv = 0) {
  std::vector<Point2> out;
  for (size_t i = 0; i < count; ++i) {
    double angle = static_cast<double>(i) * TAU / static_cast<double>(count);
    out.push_back({cu + radius * std::cos(angle), cv + radius * std::sin(angle)});
  }
  return out;
}
bool planar_contains(const PlanarTrim& trim, const Tessellation& mesh, double tolerance) {
  WorkBudget work(UNBOUNDED);
  return trim.contains_mesh(samples_of(mesh), tolerance, work);
}

// ---------------------------------------------------------------------------
// mod.rs
// ---------------------------------------------------------------------------
void analytic_surface_residuals_measure_normal_distance() {
  struct Case {
    SurfaceGeometry surface;
    Vec3 point, displaced;
  };
  std::vector<Case> cases{
      {plane(P(0, 0, 2), P(0, 0, 1), P(1, 0, 0)), P(3, 4, 2), P(3, 4, 2.5)},
      {cylinder(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), 3), P(3, 0, 7), P(3.5, 0, 7)},
      {sphere(P(1, 2, 3), P(0, 0, 1), P(1, 0, 0), 4), P(5, 2, 3), P(5.5, 2, 3)},
      {torus(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), 5, 2), P(7, 0, 0), P(7.5, 0, 0)},
  };
  for (const Case& c : cases) {
    CHECK(analytic_surface_residual(solved(c.surface), c.point) == 0.0);
    auto displaced = analytic_surface_residual(solved(c.surface), c.displaced);
    CHECK(displaced && *displaced > 0.0);
  }
  auto cone_surface = solved(cone(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), 3, 0.5, FRAC_PI_4));
  double local_radius = 3.0 + 2.0 * std::tan(FRAC_PI_4);
  auto on = analytic_surface_residual(cone_surface, P(local_radius, 0, 2));
  CHECK(on && *on <= F64_EPSILON * 128.0);
  auto off = analytic_surface_residual(cone_surface, P(local_radius + 0.5, 0, 2));
  CHECK(off && *off > 0.0);
}

void persistent_surface_identity_binds_one_face_and_body() {
  Model model = model_with_body();
  std::string face = add_square_face(model, "persistent", 0.0);
  set_shell_faces(model, {face});
  model.tessellations.push_back(persistent_mesh("synthetic:test:tessellation#mesh"));
  std::vector<std::pair<std::string, PersistentFaceIdentity>> identities{
      {face, persistent_identity(7, 3, {})}};
  std::vector<PersistentFaceBinding> bindings{
      {"synthetic:test:tessellation#mesh", persistent_identity(7, 3, {})}};
  CHECK(assign_persistent_owners(model, identities, bindings) ==
        ids({"synthetic:test:tessellation#mesh"}));
  CHECK(owned_by(mesh0(model), face));
}

void persistent_surface_identity_rejects_ambiguous_face_or_mesh_keys() {
  {
    Model model = model_with_body();
    std::string first = add_square_face(model, "first-persistent", 0.0);
    std::string second = add_square_face(model, "second-persistent", 3.0);
    set_shell_faces(model, {first, second});
    model.tessellations.push_back(persistent_mesh("synthetic:test:tessellation#mesh"));
    std::vector<std::pair<std::string, PersistentFaceIdentity>> identities{
        {first, persistent_identity(7, 3, {})}, {second, persistent_identity(7, 3, {})}};
    std::vector<PersistentFaceBinding> bindings{
        {"synthetic:test:tessellation#mesh", persistent_identity(7, 3, {})}};
    CHECK(assign_persistent_owners(model, identities, bindings).empty());
    CHECK(mesh0(model).faces.empty());
  }
  {
    Model model = model_with_body();
    std::string first = add_square_face(model, "first-mesh", 0.0);
    std::string second = add_square_face(model, "second-mesh", 3.0);
    set_shell_faces(model, {first, second});
    model.tessellations.push_back(persistent_mesh("synthetic:test:tessellation#mesh"));
    std::vector<std::pair<std::string, PersistentFaceIdentity>> identities{
        {first, persistent_identity(7, 3, {})}, {second, persistent_identity(8, 4, {})}};
    std::vector<PersistentFaceBinding> bindings{
        {"synthetic:test:tessellation#mesh", persistent_identity(7, 3, {})},
        {"synthetic:test:tessellation#mesh", persistent_identity(8, 4, {})}};
    CHECK(assign_persistent_owners(model, identities, bindings).empty());
    CHECK(mesh0(model).faces.empty());
  }
}

void persistent_surface_identity_preserves_empty_zero_length_and_tail_order() {
  Model model = model_with_body();
  std::vector<std::vector<uint32_t>> tails{{}, {0}, {10}, {10, 8}, {8, 10}};
  std::vector<std::pair<std::string, PersistentFaceIdentity>> identities;
  std::vector<PersistentFaceBinding> bindings;
  std::vector<std::string> faces;
  for (size_t index = 0; index < tails.size(); ++index) {
    std::string face =
        add_square_face(model, "tail-" + std::to_string(index), static_cast<double>(index) * 3.0);
    std::string mesh = "synthetic:test:tessellation#tail-" + std::to_string(index);
    model.tessellations.push_back(persistent_mesh(mesh));
    auto identity = persistent_identity(7, 11, tails[index]);
    identities.emplace_back(face, identity);
    bindings.push_back({mesh, identity});
    faces.push_back(face);
  }
  set_shell_faces(model, faces);
  CHECK(assign_persistent_owners(model, identities, bindings).size() == 5);
  for (size_t i = 0; i < faces.size(); ++i)
    CHECK(model.tessellations[i].faces == std::vector<std::string>{faces[i]});
}

void persistent_surface_identity_distinguishes_trailing_path_fields() {
  Model model = model_with_body();
  std::string first = add_square_face(model, "first-tail", 0.0);
  std::string second = add_square_face(model, "second-tail", 3.0);
  set_shell_faces(model, {first, second});
  model.tessellations.push_back(persistent_mesh("synthetic:test:tessellation#mesh"));
  std::vector<std::pair<std::string, PersistentFaceIdentity>> identities{
      {first, persistent_identity(266, 2, {0})}, {second, persistent_identity(266, 2, {1})}};
  std::vector<PersistentFaceBinding> bindings{
      {"synthetic:test:tessellation#mesh", persistent_identity(266, 2, {1})}};
  CHECK(assign_persistent_owners(model, identities, bindings) ==
        ids({"synthetic:test:tessellation#mesh"}));
  CHECK(mesh0(model).faces == std::vector<std::string>{second});
}

void face_key_binds_one_table_to_one_face_and_body() {
  Model model = model_with_body();
  std::string first = add_square_face(model, "first-key", 0.0);
  std::string second = add_square_face(model, "second-key", 0.0);
  set_shell_faces(model, {first, second});
  model.tessellations.push_back(persistent_mesh("synthetic:test:tessellation#mesh"));
  std::vector<std::pair<std::string, uint32_t>> face_keys{{first, 0x417300a6},
                                                          {second, 0x417300a7}};
  CHECK(assign_face_key_owners(model, face_keys,
                               {{"synthetic:test:tessellation#mesh", 0x417300a7}}) ==
        ids({"synthetic:test:tessellation#mesh"}));
  CHECK(owned_by(mesh0(model), second));
}

void repeated_face_key_on_two_faces_or_two_tables_binds_nothing() {
  Model model = model_with_body();
  std::string first = add_square_face(model, "first-repeat", 0.0);
  std::string second = add_square_face(model, "second-repeat", 3.0);
  set_shell_faces(model, {first, second});
  model.tessellations.push_back(persistent_mesh("synthetic:test:tessellation#mesh"));
  CHECK(assign_face_key_owners(model, {{first, 7}, {second, 7}},
                               {{"synthetic:test:tessellation#mesh", 7}})
            .empty());
  CHECK(mesh0(model).faces.empty());
  model.tessellations.push_back(persistent_mesh("synthetic:test:tessellation#other"));
  CHECK(assign_face_key_owners(
            model, {{first, 7}, {second, 8}},
            {{"synthetic:test:tessellation#mesh", 7}, {"synthetic:test:tessellation#other", 7}})
            .empty());
  for (const auto& mesh : model.tessellations) CHECK(mesh.faces.empty());
}

void bounded_planar_trim_selects_between_coincident_supports() {
  Model model = model_with_body();
  std::string first = add_square_face(model, "first", -4.0);
  std::string second = add_square_face(model, "second", 2.0);
  set_shell_faces(model, {first, second});
  model.tessellations.push_back(triangle_mesh(
      "synthetic:test:tessellation#mesh", {P(2.25, -0.75, 0), P(3.75, -0.75, 0), P(3, 0.75, 0)}));
  CHECK(assign_unique_surface_owners(model).assigned == ids({"synthetic:test:tessellation#mesh"}));
  CHECK(owned_by(mesh0(model), second));

  for (auto& face : model.faces)
    if (face.id == first) face.loops.clear();
  model.tessellations[0].body.reset();
  model.tessellations[0].faces.clear();
  CHECK(assign_unique_surface_owners(model).assigned.empty());
  CHECK(mesh0(model).faces.empty());
}

void bounded_cylindrical_trim_selects_between_coincident_supports() {
  Model model = model_with_body();
  std::string lower = add_cylindrical_patch_face(model, "lower", 0.0, 1.0);
  std::string upper = add_cylindrical_patch_face(model, "upper", 2.0, 3.0);
  set_shell_faces(model, {lower, upper});
  model.tessellations.push_back(triangle_mesh("synthetic:test:tessellation#lower-mesh",
                                              {P(5, 0, 0.25), P(0, 5, 0.25), P(5, 0, 0.75)}));
  CHECK(assign_unique_surface_owners(model).assigned ==
        ids({"synthetic:test:tessellation#lower-mesh"}));
  CHECK(owned_by(mesh0(model), lower));
}

void chordal_cylindrical_mesh(bool consistent_normals) {
  Model model = model_with_body();
  std::string face = add_cylindrical_patch_face(model, "chordal", 0.0, 1.0);
  set_shell_faces(model, {face});
  double deflection = 0.1;
  std::vector<Vec3> normals = consistent_normals
                                  ? std::vector<Vec3>{P(1, 0, 0), P(0, 1, 0), P(1, 0, 0)}
                                  : std::vector<Vec3>{P(0, 0, 1), P(0, 0, 1), P(0, 0, 1)};
  model.tessellations.push_back(triangle_mesh(
      "synthetic:test:tessellation#chordal-mesh",
      {P(5 - deflection, 0, 0.25), P(0, 5 - deflection, 0.25), P(5 - deflection, 0, 0.75)},
      normals));
  auto result = assign_unique_surface_owners(model);
  CHECK(result.assigned == ids({"synthetic:test:tessellation#chordal-mesh"}));
  CHECK(owned_by(mesh0(model), face));
  CHECK(result.deflections.size() == 1 &&
        std::fabs(result.deflections[0].second - deflection) <= F64_EPSILON * 128.0);
}

void off_surface_planar_mesh_does_not_become_a_chordal_cache() {
  Model model = model_with_body();
  std::string face = add_square_face(model, "off-surface", 0.0);
  set_shell_faces(model, {face});
  model.tessellations.push_back(
      triangle_mesh("synthetic:test:tessellation#off-surface-mesh",
                    {P(0.25, -0.75, 0.1), P(1.75, -0.75, 0.1), P(1.0, 0.75, 0.1)},
                    {P(0, 0, 1), P(0, 0, 1), P(0, 0, 1)}));
  CHECK(assign_unique_surface_owners(model).assigned.empty());
  CHECK(unowned(mesh0(model)));
}

void cylindrical_trim_intervals() {
  std::vector<double> angles{0.0, FRAC_PI_2};
  auto interval = circular_interval(angles);
  CHECK(interval && interval->first == 0.0 && interval->second == FRAC_PI_2);
  CHECK(circular_interval_contains(0.0, FRAC_PI_2, FRAC_PI_4, 0.0));
  CHECK(!circular_interval_contains(0.0, FRAC_PI_2, PI, 0.0));
  double start = 1.0, span = 0.5, tolerance = F64_EPSILON * 4096.0;
  CHECK(circular_interval_contains(start, span, start - tolerance * 0.5, tolerance));
  CHECK(circular_interval_contains(start, span, start + span + tolerance * 0.5, tolerance));
  CHECK(!circular_interval_contains(start, span, start - tolerance * 2.0, tolerance));
}

void unique_nurbs_support_binds_exact_display_list_face() {
  Model model = model_with_body();
  auto surface = test_nurbs_surface();
  std::string face =
      add_face(model, "nurbs-exact", nurbs_geometry(surface), test_nurbs_corners(surface));
  set_shell_faces(model, {face});
  model.tessellations.push_back(
      triangle_mesh("synthetic:test:tessellation#nurbs-exact-mesh", nurbs_points(surface)));
  auto result = assign_unique_surface_owners(model);
  CHECK(result.assigned == ids({"synthetic:test:tessellation#nurbs-exact-mesh"}));
  CHECK(owned_by(mesh0(model), face));
  CHECK(result.deflections.empty());
}

void non_exact_nurbs_support_does_not_use_an_unbounded_cache_fit() {
  Model model = model_with_body();
  auto surface = test_nurbs_surface();
  std::string face =
      add_face(model, "nurbs-cache", nurbs_geometry(surface), test_nurbs_corners(surface));
  set_shell_faces(model, {face});
  std::vector<Vec3> vertices, normals;
  for (auto [u, v] : {std::pair{0.15, 0.2}, {0.8, 0.2}, {0.5, 0.8}}) {
    auto [point, normal] = test_nurbs_point_normal(surface, u, v);
    vertices.push_back(add(point, scale(normal, 0.02)));
    normals.push_back(normal);
  }
  model.tessellations.push_back(
      triangle_mesh("synthetic:test:tessellation#nurbs-cache-mesh", vertices, normals));
  auto result = assign_unique_surface_owners(model);
  CHECK(result.assigned.empty());
  CHECK(unowned(mesh0(model)));
}

void coincident_nurbs_supports_do_not_choose_a_display_list_face() {
  Model model = model_with_body();
  auto surface = test_nurbs_surface();
  auto corners = test_nurbs_corners(surface);
  std::string first = add_face(model, "nurbs-coincident-first", nurbs_geometry(surface), corners);
  std::string second =
      add_face(model, "nurbs-coincident-second", nurbs_geometry(surface), corners);
  set_shell_faces(model, {first});
  set_shell_faces(model, {second});
  model.tessellations.push_back(
      triangle_mesh("synthetic:test:tessellation#nurbs-ambiguous-mesh", nurbs_points(surface)));
  CHECK(assign_unique_surface_owners(model).assigned.empty());
  CHECK(unowned(mesh0(model)));
}

void coincident_nurbs_and_analytic_supports_do_not_fall_through_to_analytic_fit() {
  Model model = model_with_body();
  auto surface = flat_test_nurbs_surface();
  auto corners = test_nurbs_corners(surface);
  std::string nurbs_face =
      add_face(model, "nurbs-plane-coincident", nurbs_geometry(surface), corners);
  std::string plane_face = add_face(model, "plane-coincident", z_plane(), corners);
  set_shell_faces(model, {nurbs_face});
  set_shell_faces(model, {plane_face});
  model.tessellations.push_back(
      triangle_mesh("synthetic:test:tessellation#nurbs-plane-ambiguous-mesh",
                    nurbs_points(surface), {P(0, 0, 1), P(0, 0, 1), P(0, 0, 1)}));
  CHECK(assign_unique_surface_owners(model).assigned.empty());
  CHECK(unowned(mesh0(model)));
}

void circular_hole_excludes_crossing_triangles_but_allows_boundary_chords() {
  PlanarTrim trim;
  trim.frame = z_frame();
  trim.outer = TrimPolygon(pts({{-3, -3}, {3, -3}, {3, 3}, {-3, 3}}));
  trim.holes.push_back(PlanarHole::make_circle(CircularHole{{0, 0}, 1.0}));
  auto boundary_chord = triangle_mesh("m", {P(1, 0, 0), P(0, 1, 0), P(2, 2, 0)});
  auto crossing = triangle_mesh("m", {P(-2, 0, 0), P(2, 0, 0), P(0, 2, 0)});
  CHECK(planar_contains(trim, boundary_chord, 1.0e-9));
  CHECK(!planar_contains(trim, crossing, 1.0e-9));
}

void polygonal_planar_hole_excludes_inner_face_mesh() {
  WorkBudget work(UNBOUNDED);
  PlanarTrim trim;
  trim.frame = z_frame();
  trim.outer = TrimPolygon(pts({{-4, -4}, {4, -4}, {4, 4}, {-4, 4}}));
  trim.holes.push_back(*PlanarHole::polygon(pts({{-2, -2}, {2, -2}, {2, 2}, {-2, 2}}),
                                            EPS_DISPLAY_QUANTIZATION, work));
  // Triangles [0,1,2] and [0,2,3] of the inner square, as the strip 1,2,0,3.
  auto inner_face = mesh_strips("m", {{P(2, -2, 0), P(2, 2, 0), P(-2, -2, 0), P(-2, 2, 0)}});
  auto outer_face = triangle_mesh("m", {P(-4, -4, 0), P(-3, -4, 0), P(-4, -3, 0)});
  auto exterior_chord = triangle_mesh("m", {P(-2, -2, 0), P(2, -2, 0), P(0, -4, 0)});
  auto interior_chord = triangle_mesh("m", {P(-2, -2, 0), P(2, -2, 0), P(4, 4, 0)});
  CHECK(!planar_contains(trim, inner_face, EPS_DISPLAY_QUANTIZATION));
  CHECK(planar_contains(trim, outer_face, EPS_DISPLAY_QUANTIZATION));
  CHECK(planar_contains(trim, exterior_chord, EPS_DISPLAY_QUANTIZATION));
  CHECK(!planar_contains(trim, interior_chord, EPS_DISPLAY_QUANTIZATION));
}

void mixed_planar_holes_reject_overlap() {
  auto polygon = pts({{-2, -2}, {2, -2}, {2, 2}, {-2, 2}});
  CHECK(circle_overlaps_polygon(CircularHole{{0, 0}, 1.0}, polygon, EPS_DISPLAY_QUANTIZATION));
  CHECK(!circle_overlaps_polygon(CircularHole{{4, 0}, 1.0}, polygon, EPS_DISPLAY_QUANTIZATION));
}

void planar_boundary_accepts_bounded_arcs() {
  constexpr double SAMPLE_TOLERANCE = 1.0e-4;
  auto surface = solved(z_plane());
  auto frame = *plane_frame(surface);
  BoundaryTolerance limits{EPS_DISPLAY_QUANTIZATION, SAMPLE_TOLERANCE};
  auto ellipse = planar_boundary_samples(ellipse_curve(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), 2, 1),
                                         P(2, 0, 0), P(0, 1, 0), surface, frame, limits);
  CHECK(ellipse && ellipse->first.size() > 1 && ellipse->second <= SAMPLE_TOLERANCE);
  CHECK(ellipse && ellipse->first.front() == (Point2{2.0, 0.0}));
  CHECK(!shortest_arc_span(0.0, PI));
  auto circle = planar_boundary_samples(circle_curve(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), 2),
                                        P(2, 0, 0), P(0, 2, 0), surface, frame, limits);
  CHECK(circle && circle->first.size() > 1 && circle->second <= SAMPLE_TOLERANCE);
  CHECK(circle && circle->first.front() == (Point2{2.0, 0.0}));
}

// ---------------------------------------------------------------------------
// arc_trims.rs
// ---------------------------------------------------------------------------
void circular_arc_trim_disambiguates_coincident_planar_supports() {
  Model model = model_with_body();
  std::string target = add_square_face(model, "arc-target", 0.0);
  std::string competitor = add_face(model, "arc-competitor", z_plane(),
                                    {P(2, 0, 0), P(0, 2, 0), P(0, 1, 0), P(1, 0, 0)});
  for (auto [curve_id, radius] :
       {std::pair{"synthetic:test:curve#curve-arc-competitor-0", 2.0},
        std::pair{"synthetic:test:curve#curve-arc-competitor-2", 1.0}}) {
    for (auto& curve : model.curves)
      if (curve.id == curve_id)
        curve.geometry = circle_curve(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), radius);
  }
  set_shell_faces(model, {target, competitor});
  model.tessellations.push_back(triangle_mesh("synthetic:test:tessellation#arc-trim-mesh",
                                              {P(0.25, -0.75, 0), P(1.75, -0.75, 0),
                                               P(1.0, -0.25, 0)}));
  CHECK(assign_unique_surface_owners(model).assigned ==
        ids({"synthetic:test:tessellation#arc-trim-mesh"}));
  CHECK(owned_by(mesh0(model), target));
}

// ---------------------------------------------------------------------------
// cone_supports.rs
// ---------------------------------------------------------------------------
void cone_support_binds_display_list_face() {
  Model model = model_with_body();
  double v = 2.0;
  double r = 3.0 + v * std::tan(FRAC_PI_4);
  std::string face =
      add_face(model, "cone", cone(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), 3, 0.5, FRAC_PI_4),
               {P(r, 0, v), P(0, r * 0.5, v), P(-r, 0, v), P(0, -r * 0.5, v)});
  set_shell_faces(model, {face});
  model.tessellations.push_back(triangle_mesh("synthetic:test:tessellation#cone-mesh",
                                              {P(r, 0, v), P(0, r * 0.5, v), P(-r, 0, v)}));
  CHECK(assign_unique_surface_owners(model).assigned ==
        ids({"synthetic:test:tessellation#cone-mesh"}));
  CHECK(owned_by(mesh0(model), face));
}

void cone_chordal_display_list_uses_analytic_normal_for_ownership() {
  Model model = model_with_body();
  auto geometry = cone(P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), 3, 0.5, FRAC_PI_4);
  double axial = 2.0;
  double r = 3.0 + axial * std::tan(FRAC_PI_4);
  std::string face = add_face(model, "cone-cache", geometry,
                              {P(r, 0, axial), P(0, r * 0.5, axial), P(-r, 0, axial),
                               P(0, -r * 0.5, axial)});
  set_shell_faces(model, {face});
  double cache = r - 0.1;
  std::vector<Vec3> vertices{P(cache, 0, axial), P(0, cache * 0.5, axial), P(-cache, 0, axial)};
  std::vector<Vec3> normals;
  for (const Vec3& point : vertices)
    normals.push_back(*analytic_surface_normal(solved(geometry), point));
  model.tessellations.push_back(
      triangle_mesh("synthetic:test:tessellation#cone-cache-mesh", vertices, normals));
  auto result = assign_unique_surface_owners(model);
  CHECK(result.assigned == ids({"synthetic:test:tessellation#cone-cache-mesh"}));
  CHECK(owned_by(mesh0(model), face));
  CHECK(result.deflections.size() == 1 && result.deflections[0].second > 0.09 &&
        result.deflections[0].second < 0.11);
}

void conical_trim_uses_scaled_angular_coordinate() {
  ConicalTrim trim{P(0, 0, 0), P(0, 0, 1), P(1, 0, 0), 3.0, 0.5, 1.0, 0.0, 2.0, 0.0, FRAC_PI_2};
  auto point_at = [&](double angle) {
    return P(4.0 * std::cos(angle), 4.0 * trim.ratio * std::sin(angle), 1.0);
  };
  MeshSamples inside, outside;
  inside.vertices = {point_at(FRAC_PI_4)};
  outside.vertices = {point_at(3.0 * FRAC_PI_4)};
  CHECK(trim.contains_mesh(inside, 0.0));
  CHECK(!trim.contains_mesh(outside, 0.0));
}

// ---------------------------------------------------------------------------
// geometry_predicates.rs
// ---------------------------------------------------------------------------
void geometry_predicates() {
  constexpr double EPS_DISTANCE = 1e-7;
  for (double d : {1.0, 1e-4}) {
    CHECK(segments_intersect({-d, 0}, {d, 0}, {0, -d}, {0, d}, EPS_DISTANCE));
    std::vector<Point2> p{{0, 0}, {d, 0}, {0, d}};
    CHECK(is_simple_polygon(p, EPS_DISTANCE));
    WorkBudget work(UNBOUNDED);
    auto triangles = triangulate_polygon(p, EPS_DISTANCE, work);
    CHECK(triangles && triangles->size() == 1 && (*triangles)[0][0] == p[0] &&
          (*triangles)[0][1] == p[1] && (*triangles)[0][2] == p[2]);
    std::vector<Point2> square{{0, 0}, {d, 0}, {d, d}, {0, d}};
    auto square_triangles = triangulate_polygon(square, EPS_DISTANCE, work);
    CHECK(square_triangles && square_triangles->size() == 2);
  }
  for (double offset : {0.0, 1e8}) {
    std::vector<Point2> p{{offset, offset}, {offset + 1, offset}, {offset + 1, offset + 1},
                          {offset, offset + 1}};
    CHECK(polygon_area_twice(p) == 2.0);
    CHECK(is_simple_polygon(p, EPS_DISTANCE));
  }
  {
    constexpr double T = 1.0e-9;
    auto concave = pts({{0, 0}, {4, 0}, {4, 4}, {2, 4}, {2, 2}, {0, 2}});
    CHECK(is_simple_polygon(concave, T));
    WorkBudget work(UNBOUNDED);
    CHECK(PlanarHole::polygon(concave, T, work).has_value());
    CHECK(polygon_contains(concave, {1, 1}, T));
    CHECK(polygon_contains(concave, {3, 3}, T));
    CHECK(!polygon_contains(concave, {1, 3}, T));
    CHECK(!is_simple_polygon(pts({{0, 0}, {4, 4}, {0, 4}, {4, 0}}), T));
  }
  {
    auto segments = planar_arc_segments(1e-5, 1e12, 1e-9);
    CHECK(segments && segments->first == MAX_PLANAR_TRIM_ARC_SEGMENTS);
    double expected = 2e12 * std::pow(std::sin(1e-5 / (4.0 * 4096.0)), 2);
    CHECK(segments && segments->second > 1e-9);
    CHECK(segments && std::fabs(segments->second / expected - 1.0) <= 4.0 * F64_EPSILON);
  }
  {
    CircularHole hole{{0, 0}, 1.0};
    std::vector<Point2> boundary = circle_polygon(6, 1.0);
    auto chordal = boundary;
    double angle = PI / 6.0;
    chordal.push_back({0.9 * std::cos(angle), 0.9 * std::sin(angle)});
    auto constraint = chordal_hole_constraint(hole, chordal, EPS_DISPLAY_QUANTIZATION);
    CHECK(constraint && constraint->second.radius == hole.radius &&
          constraint->first.radius < hole.radius && constraint->first.radius > 0.8);
    auto deep = boundary;
    deep.push_back({0.7 * std::cos(angle), 0.7 * std::sin(angle)});
    CHECK(!chordal_hole_constraint(hole, deep, EPS_DISPLAY_QUANTIZATION));
    CHECK(!chordal_hole_constraint(hole, pts({{0.5, 0}, {0, 0.5}}), EPS_DISPLAY_QUANTIZATION));
  }
  {
    std::vector<CircularHole> circles{{{0, 0}, 10.0}, {{6, 0}, 2.0}, {{-6, 0}, 2.0}};
    auto split = circular_outer_and_holes(circles, EPS_DISPLAY_QUANTIZATION);
    CHECK(split && split->first.radius == 10.0 && split->second.size() == 2);
    std::vector<CircularHole> ambiguous{{{0, 0}, 10.0}, {{0, 0}, 10.0}};
    CHECK(!circular_outer_and_holes(ambiguous, EPS_DISPLAY_QUANTIZATION));
  }
}

// ---------------------------------------------------------------------------
// planar_trim.rs
// ---------------------------------------------------------------------------
void sole_surface_owner_respects_its_planar_trim() {
  Model model = model_with_body();
  std::string face = add_square_face(model, "bounded", 0.0);
  set_shell_faces(model, {face});
  model.tessellations.push_back(triangle_mesh("synthetic:test:tessellation#outside",
                                              {P(10, 10, 0), P(11, 10, 0), P(10, 11, 0)}));
  CHECK(assign_unique_surface_owners(model).assigned.empty());
  CHECK(unowned(mesh0(model)));
}

void owner_work_cap_leaves_tables_unassigned_and_names_the_bound() {
  auto make = [] {
    Model model = model_with_body();
    std::string face = add_square_face(model, "bounded", 0.0);
    set_shell_faces(model, {face});
    for (const char* id : {"synthetic:test:tessellation#first", "synthetic:test:tessellation#second"})
      model.tessellations.push_back(
          triangle_mesh(id, {P(0.1, 0.1, 0), P(0.4, 0.1, 0), P(0.1, 0.4, 0)}));
    return model;
  };
  Model unbounded = make();
  auto all = assign_unique_surface_owners(unbounded);
  CHECK(all.assigned.size() == 2 && all.capped == 0);
  Model capped = make();
  auto none = assign_unique_surface_owners_within(capped, 1);
  CHECK(none.assigned.empty());
  CHECK(none.capped == 2);
  for (const auto& mesh : capped.tessellations) CHECK(unowned(mesh));
}

void planar_trim_rejects_a_triangle_spanning_a_concave_outer() {
  PlanarTrim trim;
  trim.frame = z_frame();
  trim.outer = TrimPolygon(pts({{0, 0}, {3, 0}, {3, 3}, {2, 3}, {2, 1}, {1, 1}, {1, 3}, {0, 3}}));
  auto mesh = triangle_mesh("m", {P(0.5, 2.5, 0), P(2.5, 2.5, 0), P(1.5, 0.5, 0)});
  CHECK(!planar_contains(trim, mesh, EPS_DISPLAY_QUANTIZATION));
}

void polygon_triangle_checks() {
  auto boundary = pts({{0, 0}, {1, 0}, {1, 1}, {0, 1}});
  CHECK(polygon_contains_triangle(boundary, {boundary[0], boundary[1], boundary[2]},
                                  EPS_DISPLAY_QUANTIZATION));
  auto crossing = pts({{0, 0}, {2, 0}, {0, 2}, {2, 2}});
  CHECK(!polygon_contains_triangle(crossing, {crossing[0], crossing[1], crossing[2]},
                                   EPS_DISPLAY_QUANTIZATION));
}

void single_planar_polygon_yields_a_planar_trim() {
  Model model = model_with_body();
  std::string face_id = add_square_face(model, "budget", 0.0);
  TrimTopology topology(model);
  WorkBudget work(UNBOUNDED);
  auto surface = solved(model.surfaces[0].geometry);
  CHECK(planar_trim(model.faces[0], surface, topology, work).has_value());
  (void)face_id;
}

void convex_polygon_ear_search_preserves_area() {
  auto polygon = circle_polygon(256, 1.0);
  WorkBudget work(UNBOUNDED);
  auto triangles = triangulate_polygon(polygon, EPS_DISPLAY_QUANTIZATION, work);
  CHECK(triangles && triangles->size() == polygon.size() - 2);
  double area = 0;
  if (triangles)
    for (const auto& t : *triangles) area += signed_area_twice(t[0], t[1], t[2]);
  auto expected = simple_polygon_area_twice(polygon, EPS_DISPLAY_QUANTIZATION);
  CHECK(expected && std::fabs(area - *expected) <= 1e-10);
}

void checked_convex_boundary_is_reused_for_every_triangle() {
  auto polygon = circle_polygon(256, 1.0);
  WorkBudget work(UNBOUNDED);
  TrimPolygon trim_polygon(polygon);
  auto boundary = CheckedPlanarPolygon::make(trim_polygon, EPS_DISPLAY_QUANTIZATION, work);
  CHECK(boundary.has_value());
  if (boundary)
    for (size_t i = 1; i + 1 < polygon.size(); ++i)
      CHECK(boundary->contains_triangle({polygon[0], polygon[i], polygon[i + 1]}));
}

void planar_boundary_pairs() {
  auto polygon = circle_polygon(4096, 1.0, 10.0, 10.0);
  auto outside = pts({{-2, -2}, {2, -2}, {0, 2}});
  std::vector<Point2> shared_edge{polygon[0], polygon[1], {10, 10}};
  CHECK(!polygon_inside_polygon(polygon, outside, EPS_DISPLAY_QUANTIZATION));
  CHECK(polygons_overlap(polygon, shared_edge, EPS_DISPLAY_QUANTIZATION));
  CHECK(!polygons_overlap(circle_polygon(4096, 1.0, 0.0), circle_polygon(4096, 1.0, 4.0),
                          EPS_DISPLAY_QUANTIZATION));
  CHECK(polygon_inside_polygon(circle_polygon(4096, 1.0), circle_polygon(4096, 4.0),
                               EPS_DISPLAY_QUANTIZATION));
}

// ---------------------------------------------------------------------------
// owner_fit.rs and pole_box.rs
// ---------------------------------------------------------------------------
void surface_residual_is_the_measured_residual() {
  Vec3 axis = *unit(P(0, 0.6, 0.8));
  Vec3 reference = P(1, 0, 0);
  Vec3 origin = P(0.25, -0.5, 1.0);
  std::vector<SurfaceGeometry> geometries{
      plane(origin, axis, reference), cylinder(origin, axis, reference, 0.75),
      sphere(origin, axis, reference, 0.75), torus(origin, axis, reference, 1.0, 0.25),
      cone(origin, axis, reference, 0.5, 0.75, FRAC_PI_6), nurbs_geometry(test_nurbs_surface())};
  for (const auto& geometry : geometries) {
    SolvedSurface surface = solved(geometry);
    for (int x = 0; x < 6; ++x)
      for (int y = 0; y < 6; ++y)
        for (int z = 0; z < 6; ++z) {
          Vec3 point = P(-1 + 0.4 * x, -1 + 0.4 * y, -1 + 0.4 * z);
          for (double tolerance : {1.0e-3, 0.5, 4.0}) {
            auto measured = surface_measure(surface, point, tolerance);
            auto residual = surface_residual(surface, point, tolerance);
            CHECK(measured.has_value() == residual.has_value());
            if (measured && residual) CHECK(measured->residual == *residual);
          }
        }
  }
}

void owner_work_scales_with_rows_above_the_floor() {
  size_t past_floor = TESSELLATION_OWNER_WORK / TESSELLATION_OWNER_WORK_PER_ROW * 4;
  CHECK(tessellation_owner_work_limit(10) == TESSELLATION_OWNER_WORK);
  CHECK(tessellation_owner_work_limit(past_floor) == past_floor * TESSELLATION_OWNER_WORK_PER_ROW);
  CHECK(tessellation_owner_work_limit(UNBOUNDED) == TESSELLATION_OWNER_WORK);
  CHECK(tessellation_owner_work_limit(std::nullopt) == TESSELLATION_OWNER_WORK);
}

void owner_rows_count_tables_that_already_have_an_owner() {
  Model model;
  model.tessellations.push_back(persistent_mesh("synthetic:test:tessellation#open"));
  auto open_only = tessellation_owner_rows(model);
  auto owned = persistent_mesh("synthetic:test:tessellation#owned");
  owned.faces.push_back("synthetic:test:face#face-owned");
  model.tessellations.push_back(owned);
  CHECK(open_only == 4u);
  CHECK(tessellation_owner_rows(model) == 8u);
}

NurbsSurfaceGeometry weighted_test_surface(std::array<double, 9> weights) {
  NurbsSurfaceGeometry s = test_nurbs_surface();
  s.weights.resize(3);
  for (int i = 0; i < 3; ++i) s.weights[i] = {weights[i * 3], weights[i * 3 + 1], weights[i * 3 + 2]};
  return s;
}

bool inversion_fits(const NurbsSurfaceGeometry& surface, Vec3 point, double tolerance) {
  auto parameters = nurbs_surface_parameter_near_point(surface, point);
  if (!parameters) return false;
  auto partials = nurbs_surface_partials(surface, parameters->u, parameters->v);
  return partials && distance(point, partials->point) <= tolerance;
}

void a_point_outside_the_pole_box_never_fits_the_surface() {
  constexpr double FIT_TOLERANCE = 1.0e-3;
  for (const auto& surface :
       {test_nurbs_surface(), weighted_test_surface({1.0, 0.5, 1.0, 2.0, 0.25, 2.0, 1.0, 0.5, 1.0})}) {
    size_t outside = 0;
    for (int xi = 0; xi < 8; ++xi)
      for (int yi = 0; yi < 8; ++yi)
        for (int zi = 0; zi < 5; ++zi) {
          Vec3 point = P(-0.75 + 0.5 * xi, -0.75 + 0.5 * yi, -0.5 + 0.5 * zi);
          if (outside_pole_box(surface, point, FIT_TOLERANCE)) {
            ++outside;
            CHECK(!inversion_fits(surface, point, FIT_TOLERANCE));
          }
        }
    CHECK(outside > 0);
    SurfaceGeometry nurbs_surface = nurbs_geometry(surface);  // solved_nurbs points into it
    SolvedSurface solved_nurbs = solved(nurbs_surface);
    for (auto [u, v] : {std::pair{0.0, 0.0}, {0.3, 0.8}, {0.5, 0.5}, {1.0, 0.25}}) {
      Vec3 point = *nurbs_surface_point(surface, u, v);
      CHECK(!outside_pole_box(surface, point, FIT_TOLERANCE));
      auto measure = surface_measure(solved_nurbs, point, FIT_TOLERANCE);
      CHECK(measure && measure->residual <= FIT_TOLERANCE);
    }
    CHECK(outside_pole_box(surface, P(0.5, 0.5, 2.0), FIT_TOLERANCE));
    Vec3 beyond_corner = P(-1.1 * FIT_TOLERANCE, 0, 0);
    CHECK(outside_pole_box(surface, beyond_corner, FIT_TOLERANCE));
    CHECK(!inversion_fits(surface, beyond_corner, FIT_TOLERANCE));
    Vec3 near_corner = P(-0.9 * FIT_TOLERANCE, 0, 0);
    CHECK(!outside_pole_box(surface, near_corner, FIT_TOLERANCE));
    CHECK(inversion_fits(surface, near_corner, FIT_TOLERANCE));
  }
  auto negative = weighted_test_surface({1.0, 0.5, 1.0, 2.0, -0.25, 2.0, 1.0, 0.5, 1.0});
  CHECK(!outside_pole_box(negative, P(100, 100, 100), FIT_TOLERANCE));
}

// ---------------------------------------------------------------------------
// edge_bands.rs
// ---------------------------------------------------------------------------
uint64_t xorshift(uint64_t& state) {
  state ^= state << 13;
  state ^= state >> 7;
  state ^= state << 17;
  return state;
}
double unit_random(uint64_t& state) {
  return static_cast<double>(static_cast<uint32_t>(xorshift(state) >> 40)) / double(1u << 24);
}
size_t below(uint64_t& state, uint64_t bound) { return static_cast<size_t>(xorshift(state) % bound); }

std::vector<Point2> star(uint64_t& state, Point2 center, double s) {
  size_t count = 16 + below(state, 150);
  std::vector<Point2> points;
  for (size_t index = 0; index < count; ++index) {
    double angle = (static_cast<double>(index) + unit_random(state) * 0.9) * TAU /
                   static_cast<double>(count);
    double radius = s * (below(state, 16) == 0 ? 0.05 : 0.3 + unit_random(state) * 0.7);
    points.push_back({center.u + radius * std::cos(angle), center.v + radius * std::sin(angle)});
    switch (below(state, 12)) {
      case 0: points.push_back(points.back()); break;
      case 1: points.push_back({points.back().u + s * 0.01, points.back().v}); break;
      default: break;
    }
  }
  return points;
}

std::vector<Point2> probes(uint64_t& state, const std::vector<Point2>& polygon, double tolerance,
                           double s) {
  std::vector<Point2> points{{std::nan(""), polygon[0].v},
                             {polygon[0].u, std::numeric_limits<double>::infinity()}};
  double offset = std::isfinite(tolerance) ? std::fabs(tolerance) : s * 1.0e-3;
  for (int i = 0; i < 12; ++i) {
    size_t index = below(state, polygon.size());
    Point2 start = polygon[index], end = polygon[(index + 1) % polygon.size()];
    double along = unit_random(state) * 1.2 - 0.1;
    double du = end.u - start.u, dv = end.v - start.v;
    double length = std::hypot(du, dv);
    double nu = length > 0 ? -dv / length : 0.0, nv = length > 0 ? du / length : 1.0;
    double side = offset * (1.0 + (unit_random(state) - 0.5) * 1.0e-6);
    Point2 a = start;
    Point2 b{start.u + s * (unit_random(state) - 0.5), start.v};
    Point2 c{start.u + along * du + side * nu, start.v + along * dv + side * nv};
    Point2 d{start.u + along * du - side * nu, start.v + along * dv - side * nv};
    Point2 e{start.u, start.v - side};
    double fu = (unit_random(state) - 0.5) * 2.5 * s;
    double fv = (unit_random(state) - 0.5) * 2.5 * s;
    Point2 f{polygon[0].u + fu, polygon[0].v + fv};
    points.insert(points.end(), {a, b, c, d, e, f});
  }
  return points;
}

void banded_queries_match_scanning_every_edge() {
  uint64_t state = 0x6a09e667f3bcc908ull;
  size_t polygons = 0, banded = 0, cases = 0, near = 0, inside = 0, excluded = 0;
  for (double s : {1.0e-250, 1.0e-6, 1.0, 3.0e4, 1.0e150, 1.0e300}) {
    for (int k = 0; k < 20; ++k) {
      double cu = (unit_random(state) - 0.5) * 8.0 * s;
      double cv = (unit_random(state) - 0.5) * 8.0 * s;
      auto points = star(state, {cu, cv}, s);
      TrimPolygon polygon(points);
      ++polygons;
      banded += polygon.bands().has_value();
      double random_tolerance = s * F32_EPSILON * unit_random(state);
      for (double tolerance : {0.0, -0.0, s * 1.0e-12, random_tolerance, s * 1.0e-3, -s * 1.0e-6,
                               std::nan(""), std::numeric_limits<double>::infinity()}) {
        for (Point2 point : probes(state, points, tolerance, s)) {
          bool within = polygon_boundary_within(points, point, tolerance);
          bool contains = polygon_contains(points, point, tolerance);
          CHECK(polygon.boundary_within(point, tolerance) == within);
          CHECK(polygon.contains(point, tolerance) == contains);
          bool strictly = contains && !within;
          CHECK(polygon_strictly_contains(points, point, tolerance) == strictly);
          CHECK(polygon.strictly_contains(point, tolerance) == strictly);
          auto box = PolygonBox::make(points, tolerance);
          if (box && box->excludes(point)) {
            CHECK(!contains && !within);
            ++excluded;
          }
          ++cases;
          near += within;
          inside += contains && !within;
        }
      }
    }
  }
  CHECK(banded * 4 > polygons * 3);
  CHECK(near * 10 > cases);
  CHECK(inside * 10 > cases);
  CHECK(excluded * 50 > cases);
}

void long_edges_take_fewer_bands_and_unbandable_polygons_are_scanned_whole() {
  std::vector<Point2> comb;
  for (int index = 0; index < 64; ++index) {
    comb.push_back({double(index), 0.0});
    comb.push_back({index + 0.5, 100.0});
  }
  comb.push_back({64.0, -1.0});
  auto bands = TrimPolygon(comb).bands();
  CHECK(bands && bands->starts.size() - 1 < comb.size() / 4);
  std::vector<Point2> flat;
  for (int index = 0; index < 32; ++index) flat.push_back({double(index), 1.0});
  auto not_finite = comb;
  not_finite[3] = {std::nan(""), 0.0};
  std::vector<Point2> small(comb.begin(), comb.begin() + 12);
  for (const auto* polygon : {&flat, &not_finite, &small}) CHECK(!TrimPolygon(*polygon).bands());
  for (const auto& polygon : {comb, flat, not_finite, small}) {
    TrimPolygon trim(polygon);
    for (Point2 point : {Point2{10.2, 50.0}, Point2{3.0, 0.0}})
      CHECK(trim.contains(point, 1.0e-9) == polygon_contains(polygon, point, 1.0e-9));
  }
}

void remembered_simplicity_is_the_tested_simplicity() {
  auto points = circle_polygon(64, 1.0);
  points[10] = {points[9].u + 0.01, points[9].v};
  TrimPolygon polygon(points);
  for (double tolerance : {1.0e-9, 1.0e-9, 0.2, 1.0e-9, 0.0, 0.2, 0.2})
    CHECK(polygon.is_simple(tolerance) == is_simple_polygon(points, tolerance));
}

// ---------------------------------------------------------------------------
// Engine entry point
// ---------------------------------------------------------------------------
void tessellation_owners_run_keys_then_identities_then_surfaces() {
  Model model = model_with_body();
  std::string keyed = add_square_face(model, "keyed", 0.0);
  std::string persistent = add_square_face(model, "persistent", 3.0);
  std::string geometric = add_square_face(model, "geometric", 6.0);
  set_shell_faces(model, {keyed, persistent, geometric});
  // Every table lies on the geometric face, so only the earlier passes can
  // give the first two another owner.
  for (const char* id : {"t#keyed", "t#persistent", "t#geometric"})
    model.tessellations.push_back(
        triangle_mesh(id, {P(6.25, -0.75, 0), P(7.75, -0.75, 0), P(7, 0.75, 0)}));
  BrepResult brep;
  brep.decoded = true;
  brep.face_keys = {{keyed, 41}};
  brep.face_identities = {{persistent, persistent_identity(5, 6, {})}};
  assign_tessellation_owners(model, brep, {{"t#keyed", 41}},
                             {{"t#persistent", persistent_identity(5, 6, {})}});
  CHECK(owned_by(model.tessellations[0], keyed));
  CHECK(owned_by(model.tessellations[1], persistent));
  CHECK(owned_by(model.tessellations[2], geometric));
}

}  // namespace

int main() {
  analytic_surface_residuals_measure_normal_distance();
  persistent_surface_identity_binds_one_face_and_body();
  persistent_surface_identity_rejects_ambiguous_face_or_mesh_keys();
  persistent_surface_identity_preserves_empty_zero_length_and_tail_order();
  persistent_surface_identity_distinguishes_trailing_path_fields();
  face_key_binds_one_table_to_one_face_and_body();
  repeated_face_key_on_two_faces_or_two_tables_binds_nothing();
  bounded_planar_trim_selects_between_coincident_supports();
  bounded_cylindrical_trim_selects_between_coincident_supports();
  chordal_cylindrical_mesh(true);
  chordal_cylindrical_mesh(false);
  off_surface_planar_mesh_does_not_become_a_chordal_cache();
  cylindrical_trim_intervals();
  unique_nurbs_support_binds_exact_display_list_face();
  non_exact_nurbs_support_does_not_use_an_unbounded_cache_fit();
  coincident_nurbs_supports_do_not_choose_a_display_list_face();
  coincident_nurbs_and_analytic_supports_do_not_fall_through_to_analytic_fit();
  circular_hole_excludes_crossing_triangles_but_allows_boundary_chords();
  polygonal_planar_hole_excludes_inner_face_mesh();
  mixed_planar_holes_reject_overlap();
  planar_boundary_accepts_bounded_arcs();
  circular_arc_trim_disambiguates_coincident_planar_supports();
  cone_support_binds_display_list_face();
  cone_chordal_display_list_uses_analytic_normal_for_ownership();
  conical_trim_uses_scaled_angular_coordinate();
  geometry_predicates();
  sole_surface_owner_respects_its_planar_trim();
  owner_work_cap_leaves_tables_unassigned_and_names_the_bound();
  planar_trim_rejects_a_triangle_spanning_a_concave_outer();
  polygon_triangle_checks();
  single_planar_polygon_yields_a_planar_trim();
  convex_polygon_ear_search_preserves_area();
  checked_convex_boundary_is_reused_for_every_triangle();
  planar_boundary_pairs();
  surface_residual_is_the_measured_residual();
  owner_work_scales_with_rows_above_the_floor();
  owner_rows_count_tables_that_already_have_an_owner();
  a_point_outside_the_pole_box_never_fits_the_surface();
  banded_queries_match_scanning_every_edge();
  long_edges_take_fewer_bands_and_unbandable_polygons_are_scanned_whole();
  remembered_simplicity_is_the_tested_simplicity();
  tessellation_owners_run_keys_then_identities_then_surfaces();
  if (failures) {
    std::fprintf(stderr, "ownership_test: %d failure(s)\n", failures);
    return 1;
  }
  std::printf("ownership_test: ok\n");
  return 0;
}
