// SPDX-License-Identifier: Apache-2.0
// Decoded document model: the subset of cadmpeg's CADIR that the GLB
// writer consumes. Field names follow the CADIR JSON so `dump`
// output can be compared with `cadmpeg convert ... -o model.cadir.json`.
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace swglb {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};

/// CADIR colour: sRGB channels normalised to 0..1, stored as f32 like
/// cadmpeg's `Color` (`Color::from_rgba8(r, g, b, a)` = channel / 255).
struct Color {
  float r = 0, g = 0, b = 0, a = 1;
  static Color from_rgba8(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return Color{r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f};
  }
  bool operator==(const Color&) const = default;
};

/// Byte interval [start, end) inside one section payload.
struct ByteRange {
  size_t start = 0, end = 0;
};

// ---------------------------------------------------------------------------
// Saved display meshes (Contents/DisplayLists)
// ---------------------------------------------------------------------------
struct MeshVertex {
  Vec3 position;  ///< Millimetres (stored f32 metres x 1000).
  Vec3 normal;
};

/// Assembly placement provenance written by the assembly stage
/// (cadmpeg `SourceObjectAssociation`).
struct SourceObject {
  std::string object_id;              ///< Full instance path.
  std::optional<std::string> name;
  std::optional<Color> color;         ///< Never set by the current reader.
  std::optional<bool> visible;
};

struct Tessellation {
  /// "sldprt:displaylist:record#<section ordinal>:<table index>"; copies made
  /// by shared-mesh expansion get their own identity (see assembly stage).
  std::string id;
  /// Triangle strips. `has_normals` is false for an unshaded table (empty
  /// native normal channel); normals are then zero.
  std::vector<std::vector<MeshVertex>> strips;
  bool has_normals = true;
  std::optional<std::string> body;  ///< B-rep body owner, when resolved.
  std::vector<std::string> faces;   ///< B-rep face owners, when resolved.
  std::optional<SourceObject> source_object;

  // Provenance (not in CADIR JSON).
  size_t section_index = 0;  ///< Index into Container::sections.
  size_t table_index = 0;    ///< Index of the table inside its section.
  ByteRange table;           ///< Descriptor table bytes.
  ByteRange metadata;        ///< Bytes after the table up to the next table.
};

// ---------------------------------------------------------------------------
// Appearance
// ---------------------------------------------------------------------------
struct Appearance {
  std::string id;
  std::optional<std::string> name;
  std::optional<std::string> schema;
  std::optional<Color> base_color;
};

struct AppearanceBinding {
  std::string id;
  std::string target_kind;  ///< "tessellation", "face", "body", "source", ...
  std::string target_id;
  std::string appearance;
  std::optional<std::string> source_entity_id;
  std::optional<std::string> object_type;
  std::map<std::string, std::string> channels;
};

// ---------------------------------------------------------------------------
// B-rep (lengths in millimetres)
// ---------------------------------------------------------------------------
/// Tensor-product NURBS surface (cadmpeg `NurbsSurface`; CADIR `u_degree`,
/// `u_knots`, `u_periodic`, `poles.rows`). Needed by tessellation ownership.
struct NurbsSurfaceGeometry {
  uint32_t u_degree = 0, v_degree = 0;
  std::vector<double> u_knots, v_knots;  ///< Full knot vectors (count + degree + 1).
  bool u_periodic = false, v_periodic = false;
  std::vector<std::vector<Vec3>> poles;      ///< poles[i][j]: i along u, j along v.
  std::vector<std::vector<double>> weights;  ///< Same shape as poles; empty = polynomial.
};

/// NURBS curve (cadmpeg `NurbsCurve`; CADIR `degree`, `knots`, `periodic`, `poles`).
struct NurbsCurveGeometry {
  uint32_t degree = 0;
  std::vector<double> knots;
  bool periodic = false;
  std::vector<Vec3> poles;
  std::vector<double> weights;  ///< Empty = polynomial.
};

struct SurfaceGeometry {
  std::string kind;  ///< "plane", "cylinder", "cone", "sphere", "torus", "nurbs", ...
  Vec3 origin;       ///< plane origin / cylinder, cone, sphere, torus centre
  Vec3 normal;       ///< plane normal / axis
  Vec3 u_axis;       ///< reference direction
  double radius = 0;        ///< cylinder, sphere, cone (at origin), torus major
  double minor_radius = 0;  ///< torus
  double half_angle = 0;    ///< cone
  double ratio = 1;         ///< cone: minor / major cross-section radius
  std::optional<NurbsSurfaceGeometry> nurbs;  ///< kind == "nurbs"
};

struct CurveGeometry {
  std::string kind;  ///< "line", "circle", "ellipse", "nurbs", ...
  Vec3 origin;       ///< line origin
  Vec3 direction;    ///< line direction
  Vec3 center;       ///< circle/ellipse centre
  Vec3 axis;         ///< circle/ellipse normal
  Vec3 ref_direction;
  double radius = 0;        ///< circle radius / ellipse major radius
  double minor_radius = 0;  ///< ellipse minor radius (major direction = ref_direction)
  std::optional<NurbsCurveGeometry> nurbs;  ///< kind == "nurbs"
};

struct Surface {
  std::string id;
  SurfaceGeometry geometry;
};
struct Curve {
  std::string id;
  CurveGeometry geometry;
};
struct Point {
  std::string id;
  Vec3 position;
};
struct Vertex {
  std::string id;
  std::string point;
};
struct Edge {
  std::string id;
  std::optional<std::string> curve;                       ///< carrier.curve
  std::optional<std::array<double, 2>> param_range;       ///< carrier.param_range
  std::optional<std::string> start, end;                  ///< vertex ids
};
struct Coedge {
  std::string id;
  std::string owner_loop;
  std::string edge;
  bool reversed = false;
};
struct Loop {
  std::string id;
  std::string face;
  std::vector<std::string> coedges;
};
struct Face {
  std::string id;
  std::string shell;
  std::optional<std::string> surface;
  bool reversed = false;
  std::vector<std::string> loops;
  std::optional<Color> color;
};
struct Shell {
  std::string id;
  std::string region;
  std::vector<std::string> faces;
};
struct Region {
  std::string id;
  std::string body;
  std::vector<std::string> shells;
};
struct Body {
  std::string id;
  std::string kind;  ///< "solid", "sheet", "wire", "general"
  std::vector<std::string> regions;
  std::optional<Color> color;
};
struct Configuration {
  std::string id;
  size_t ordinal = 0;
  bool active = false;
  std::optional<std::string> name;
  std::vector<std::string> bodies;
};

// ---------------------------------------------------------------------------
// Saved presentation (DisplayLists scene and reference-plane commands)
// ---------------------------------------------------------------------------
struct ReferencePlane {
  /// CADIR attribute id "sldprt:metadata:saved_reference_plane#<section
  /// ordinal>:<offset>"; the reference writer reads the planes in this id order.
  std::string id;
  size_t source_offset = 0;  ///< Command offset inside the DisplayLists payload.
  std::string label;
  std::vector<Vec3> corners;  ///< 5 points, mm (closed frame).
  Vec3 origin;                ///< mm
  std::string font;
  int64_t point_size = 0;
  std::vector<double> advances;  ///< mm
  double scale = 1;
  /// Pen selection. Exactly one of: indexed pen (`color_index`), custom pen
  /// (`custom_rgb` set; normalised 0..1 sRGB, or empty when the custom pen was
  /// unreadable), or neither (no pen state).
  std::optional<int64_t> color_index;
  bool custom = false;
  std::optional<std::array<double, 3>> custom_rgb;
};

struct Occurrence {
  std::string id;
  std::string path;
  bool visible = true;
};

struct Model {
  std::vector<Tessellation> tessellations;
  std::vector<Appearance> appearances;
  std::vector<AppearanceBinding> appearance_bindings;

  std::vector<Body> bodies;
  std::vector<Region> regions;
  std::vector<Shell> shells;
  std::vector<Face> faces;
  std::vector<Loop> loops;
  std::vector<Coedge> coedges;
  std::vector<Edge> edges;
  std::vector<Vertex> vertices;
  std::vector<Point> points;
  std::vector<Surface> surfaces;
  std::vector<Curve> curves;
  std::vector<Configuration> configurations;
  std::vector<Occurrence> occurrences;

  std::optional<int64_t> saved_display_mode;
  std::vector<ReferencePlane> reference_planes;
  std::string length_unit = "millimeter";
};

}  // namespace swglb
