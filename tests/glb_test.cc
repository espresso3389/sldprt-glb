// SPDX-License-Identifier: Apache-2.0
// Ports of the reference writer's GLB unit tests. Inputs are synthetic CADIR
// documents built as JSON and read with load_cadir; values that JSON can not
// carry (NaN) are set on the loaded Model.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "cadir_load.h"
#include "errors.h"
#include "glb.h"
#include "glb_internal.h"
#include "json_value.h"
#include "model.h"
#include "regenerate.h"

using namespace swglb;
using namespace swglb::glb_internal;

namespace {

#define CHECK(cond)                                                                 \
  do {                                                                              \
    if (!(cond)) {                                                                  \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      std::abort();                                                                 \
    }                                                                               \
  } while (0)

using J = JValue;

constexpr const char* kKey = "solidworksPresentation";
const double kNaN = std::numeric_limits<double>::quiet_NaN();

// ---------------------------------------------------------------------------
// CADIR construction (mirrors the reference tests' dict helpers)
// ---------------------------------------------------------------------------
J obj(std::initializer_list<std::pair<std::string, J>> members) {
  J out = J::object();
  for (const auto& [k, v] : members) out[k] = v;
  return out;
}
J arr(std::initializer_list<J> items) { return J(J::Array(items)); }
J xyz(double x, double y, double z) { return obj({{"x", x}, {"y", y}, {"z", z}}); }

J vertex(double x, double y, double z = 0) {
  return obj({{"position", xyz(x, y, z)}, {"normal", xyz(0, 0, 1)}});
}
J strip(std::initializer_list<J> vertices) { return arr(vertices); }

J shaded(J strips) { return obj({{"mesh", obj({{"kind", "shaded_strips"}, {"strips", strips}})}}); }

J sample(J strips) {
  return obj({{"units", obj({{"length", "millimeter"}})},
              {"model", obj({{"tessellations", arr({shaded(strips)})}})}});
}
J triangle(double size = 1000) {
  return sample(arr({strip({vertex(0, 0), vertex(size, 0), vertex(0, size)})}));
}

J values(std::initializer_list<J> items) {
  J out = J::array();
  for (const auto& v : items) out.push_back(obj({{"value", v}}));
  return out;
}
J mode_attribute(int mode) { return obj({{"name", "saved_display_mode"}, {"values", values({mode})}}); }
J plane_attribute(J vals) { return obj({{"name", "saved_reference_plane"}, {"values", vals}}); }
J unit_frame() { return arr({0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0}); }
J six(int v) { return arr({v, v, v, v, v, v}); }
/// saved_reference_plane values: label, corners, origin, font, size, advances, scale, extra...
J plane_values(J corners, std::initializer_list<J> extra) {
  J out = values({"Plane1", corners, arr({0, 0, 0}), "Century Gothic", 12, six(2), 1});
  for (const auto& v : extra) out.push_back(obj({{"value", v}}));
  return out;
}

Model load(const J& data) { return load_cadir(data.dump()); }

// ---------------------------------------------------------------------------
// GLB inspection
// ---------------------------------------------------------------------------
uint32_t le32(const Bytes& b, size_t at) {
  return static_cast<uint32_t>(b[at]) | static_cast<uint32_t>(b[at + 1]) << 8 |
         static_cast<uint32_t>(b[at + 2]) << 16 | static_cast<uint32_t>(b[at + 3]) << 24;
}

J json_chunk(const Bytes& glb) {
  uint32_t length = le32(glb, 12);
  return J::parse(std::string_view(reinterpret_cast<const char*>(glb.data()) + 20, length));
}

/// make_glb's (glb, triangles, tables): write_glb without stale-mesh
/// regeneration is exactly make_glb.
struct Made {
  Bytes glb;
  J doc;
  size_t triangles;
  size_t tables;
};
Made make_glb(const Model& model) {
  GlbOptions options;
  options.part = false;
  Made out;
  out.glb = write_glb(model, options);
  out.doc = json_chunk(out.glb);
  out.triangles = static_cast<size_t>(out.doc.find("accessors")->arr().at(2).find("count")->as_int()) / 3;
  out.tables = model.tessellations.size();
  return out;
}
Made make_glb(const J& data) { return make_glb(load(data)); }

const J& at(const J& v, std::string_view key) {
  CHECK(v.is_object());
  const J* m = v.find(key);
  CHECK(m);
  return *m;
}
const J& at(const J& v, size_t i) {
  CHECK(v.is_array() && i < v.arr().size());
  return v.arr()[i];
}
const J& last(const J& v) {
  CHECK(v.is_array() && !v.arr().empty());
  return v.arr().back();
}
bool is_null(const J& v) { return v.dump() == "null"; }
bool has(const J& v, std::string_view key) { return v.is_object() && v.contains(key); }

bool numbers_eq(const J& v, const std::vector<double>& expected) {
  if (!v.is_array() || v.arr().size() != expected.size()) return false;
  for (size_t i = 0; i < expected.size(); ++i) {
    const J& e = v.arr()[i];
    if (!(e.is_int() || e.is_float()) || e.as_double() != expected[i]) return false;
  }
  return true;
}

template <class F>
bool throws_decode(F f) {
  try {
    f();
  } catch (const DecodeError&) {
    return true;
  }
  return false;
}

Lookup lookup_of(const Model& model) {
  Lookup lookup;
  for (const auto& f : model.faces) lookup.faces.emplace(f.id, &f);
  for (const auto& b : model.bodies) lookup.bodies.emplace(b.id, &b);
  return lookup;
}

double srgb_to_linear(double c) { return c <= .04045 ? c / 12.92 : std::pow((c + .055) / 1.055, 2.4); }

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
void stale_surface_is_replaced_without_losing_boundary_lines() {
  J data = sample(arr({strip({vertex(0, 0), vertex(1000, 0), vertex(0, 2000)})}));
  J& model = data["model"];
  model["bodies"] = arr({obj({{"id", "b"}})});
  model["surfaces"] = arr({obj({{"geometry", obj({{"kind", "plane"}})}})});
  model["attributes"] = arr({mode_attribute(3)});
  model["curves"] = arr({obj({{"id", "c"}, {"geometry", obj({{"kind", "line"}})}})});
  model["points"] = arr({obj({{"id", "a"}, {"position", xyz(0, 0, 0)}}),
                         obj({{"id", "b"}, {"position", xyz(1000, 1000, 0)}})});
  model["vertices"] = arr({obj({{"id", "a"}, {"point", "a"}}), obj({{"id", "b"}, {"point", "b"}})});
  model["edges"] = arr({obj({{"id", "e"}, {"carrier", obj({{"curve", "c"}})}, {"start", "a"}, {"end", "b"}})});
  Model loaded = load(data);
  Made saved = make_glb(loaded);
  // stale_surface_bounds reads the first surface primitive's POSITION bounds.
  const J& accessor = at(at(saved.doc, "accessors"),
                         static_cast<size_t>(at(at(at(at(at(at(saved.doc, "meshes"), 0), "primitives"), 0),
                                                   "attributes"), "POSITION").as_int()));
  Bounds surface;
  for (int i = 0; i < 3; ++i) {
    surface.first[i] = at(at(accessor, "min"), i).as_double();
    surface.second[i] = at(at(accessor, "max"), i).as_double();
  }
  auto boundary = stale_surface_bounds(loaded, surface);
  CHECK(boundary.has_value());
  Made corrected = make_glb(triangle());
  J result = glb_parts(replace_surface_glb(saved.glb, corrected.glb, *boundary, kKey)).doc;
  CHECK(at(result, "meshes").size() == 2);
  CHECK(at(at(at(at(result, "meshes"), 1), "primitives"), 0).find("mode")->as_int() == 1);
  const J& primitive = at(at(at(at(result, "meshes"), 0), "primitives"), 0);
  size_t position = static_cast<size_t>(at(at(primitive, "attributes"), "POSITION").as_int());
  CHECK(numbers_eq(at(at(at(result, "accessors"), position), "max"), {1.0, 0.0, -0.0}));
  CHECK(throws_decode([&] { replace_surface_glb(saved.glb, saved.glb, *boundary, kKey); }));
}

void bounds_tolerance_avoids_rebuilding_small_tessellation_error() {
  CHECK(!bounds_differ({{0, 0, 0}, {1, 1, 1}}, {{0, 0, 0}, {1.0005, 1, 1}}));
  CHECK(bounds_differ({{0, 0, 0}, {1, 1, 1}}, {{0, 0, 0}, {1.01, 1, 1}}));
}

void unsupported_boundary_preserves_visible_surface() {
  for (int mode : {0, 3}) {
    J data = sample(arr({strip({vertex(0, 0), vertex(1, 0), vertex(0, 1)})}));
    J& model = data["model"];
    model["attributes"] = arr({mode_attribute(mode)});
    model["edges"] = arr({obj({{"id", "e"}, {"carrier", obj({{"curve", "c"}, {"param_range", arr({0, 1})}})}})});
    model["curves"] = arr({obj({{"id", "c"}, {"geometry", obj({{"kind", "bspline"}})}})});
    Made made = make_glb(data);
    CHECK(made.triangles == 1);
    CHECK(!has(at(at(made.doc, "nodes"), 0), "extras"));
    if (const J* materials = made.doc.find("materials")) {
      const J* alpha = at(*materials, 0).find("alphaMode");
      CHECK(!alpha || alpha->str() != "BLEND");
    }
  }
}

void native_reference_pen_colors_reach_both_frame_and_label() {
  // Independent native observations grouped by color, including aliases.
  const std::vector<std::pair<const char*, std::vector<int>>> observed = {
      {"000000", {0, 1, 4, 12, 13, 18, 34, 41, 47, 48, 52}},
      {"c0c0c0", {2, 8, 15, 22, 23, 43, 45, 54}},
      {"ff0000", {3, 17, 31, 35, 50, 53}},
      {"0000ff", {5, 37, 40, 51}},
      {"808000", {6, 9, 16}},
      {"ffff00", {7, 24}},
      {"ffffff", {10, 11, 28, 29}},
      {"00ff00", {14, 36}},
      {"808080", {19, 21, 38, 44}},
      {"ff8080", {20, 25, 26}},
      {"0080c0", {27, 30}},
      {"800000", {32}},
      {"008000", {33}},
      {"004000", {39}},
      {"80ffff", {42}},
      {"ff8000", {46}},
      {"ff0080", {49}},
  };
  std::vector<int> all;
  for (const auto& [hex, ids] : observed) all.insert(all.end(), ids.begin(), ids.end());
  std::sort(all.begin(), all.end());
  CHECK(all.size() == 55);
  for (int i = 0; i < 55; ++i) CHECK(all[i] == i);
  for (const auto& [hex, indexes] : observed) {
    std::vector<double> expected;
    for (int c = 0; c < 3; ++c) expected.push_back(std::stoi(std::string(hex + 2 * c, 2), nullptr, 16) / 255.0);
    for (int index : indexes) {
      J data = sample(arr({strip({vertex(0, 0), vertex(1, 0), vertex(0, 1)})}));
      data["model"]["attributes"] = arr({plane_attribute(plane_values(unit_frame(), {index}))});
      Made made = make_glb(data);
      const J& node = last(at(made.doc, "nodes"));
      const J& label = at(at(node, "extras"), kKey);
      CHECK(at(label, "colorIndex").as_int() == index);
      CHECK(numbers_eq(at(label, "color"), expected));
      CHECK(at(label, "colorResolved").as_bool());
      const J& primitive = at(at(at(at(made.doc, "meshes"), static_cast<size_t>(at(node, "mesh").as_int())),
                                 "primitives"), 0);
      CHECK(at(primitive, "mode").as_int() == 1);
      const J& material = at(at(made.doc, "materials"), static_cast<size_t>(at(primitive, "material").as_int()));
      CHECK(has(at(material, "extensions"), "KHR_materials_unlit"));
      const J& rgba = at(at(material, "pbrMetallicRoughness"), "baseColorFactor");
      CHECK(at(rgba, 3).as_double() == 1);
      for (int c = 0; c < 3; ++c) {
        double linear = at(rgba, c).as_double();
        double restored = linear <= 0.0031308 ? 12.92 * linear : 1.055 * std::pow(linear, 1 / 2.4) - 0.055;
        CHECK(std::abs(restored - expected[c]) < 5e-8);  // assertAlmostEqual
      }
    }
  }
}

/// reference_pen_color(index) is the indexed branch of reference_plane_color.
/// Selectors that are not integers (null, bool, float, string, list) never
/// reach ReferencePlane::color_index; load them through load_cadir to show it.
PlaneColor pen_color(const J& selector) {
  J data = triangle();
  data["model"]["attributes"] = arr({plane_attribute(plane_values(unit_frame(), {selector}))});
  Model model = load(data);
  CHECK(model.reference_planes.size() == 1);
  const ReferencePlane& plane = model.reference_planes[0];
  CHECK(!plane.custom);
  CHECK(plane.color_index.has_value() == selector.is_int());
  PlaneColor color = reference_plane_color(plane);
  CHECK(std::string(color.source) == "indexed");
  return color;
}

void unknown_reference_pen_does_not_wrap_or_guess_rgb() {
  const std::array<double, 3> grey{0.8, 0.8, 0.8}, black{0.0, 0.0, 0.0};
  for (const J& index : {J(nullptr), J(-1), J(int64_t{4294967301}), J(true), J(5.0), J("5"), J::array()}) {
    PlaneColor color = pen_color(index);
    CHECK(color.srgb == grey);
    CHECK(!color.resolved);
  }
  for (int64_t index : {int64_t{55}, int64_t{56}, int64_t{63}, int64_t{64}, int64_t{255}, int64_t{0xff00ff},
                        int64_t{0xffffffff}}) {
    PlaneColor color = pen_color(J(index));
    CHECK(color.srgb == black);
    CHECK(!color.resolved);
    ReferencePlane plane;
    plane.color_index = index;
    color = reference_plane_color(plane);
    CHECK(color.srgb == black);
    CHECK(!color.resolved);
  }
}

void custom_reference_rgb_reaches_frame_and_label_without_palette_quantization() {
  const std::vector<std::vector<double>> colors = {
      {1, 0, 1}, {23 / 255.0, 45 / 255.0, 180 / 255.0}, {0, 0, 0}, {1, 1, 1}};
  for (const auto& rgb : colors) {
    J data = sample(arr({strip({vertex(0, 0), vertex(1, 0), vertex(0, 1)})}));
    data["model"]["attributes"] =
        arr({plane_attribute(plane_values(unit_frame(), {"rgb", arr({rgb[0], rgb[1], rgb[2]})}))});
    Made made = make_glb(data);
    const J& node = last(at(made.doc, "nodes"));
    const J& saved = at(at(node, "extras"), kKey);
    CHECK(is_null(at(saved, "colorIndex")));
    CHECK(at(saved, "colorSource").str() == "custom");
    CHECK(at(saved, "colorResolved").as_bool());
    CHECK(numbers_eq(at(saved, "color"), rgb));
    const J& primitive = at(at(at(at(made.doc, "meshes"), static_cast<size_t>(at(node, "mesh").as_int())),
                               "primitives"), 0);
    const J& material = at(at(made.doc, "materials"), static_cast<size_t>(at(primitive, "material").as_int()));
    CHECK(has(at(material, "extensions"), "KHR_materials_unlit"));
    std::vector<double> linear;
    for (double c : rgb) linear.push_back(srgb_to_linear(c));
    linear.push_back(1);
    CHECK(numbers_eq(at(at(material, "pbrMetallicRoughness"), "baseColorFactor"), linear));
  }
}

void check_unresolved_custom(const PlaneColor& color) {
  CHECK(is_null(color.selector));
  CHECK((color.srgb == std::array<double, 3>{.8, .8, .8}));
  CHECK(!color.resolved);
  CHECK(std::string(color.source) == "custom");
}

void invalid_custom_rgb_is_unresolved_and_never_clamped_or_used_as_index() {
  // Three-channel values reach ReferencePlane::custom_rgb as doubles.
  const double inf = std::numeric_limits<double>::infinity();
  for (const std::array<double, 3>& rgb : {std::array<double, 3>{-1, 0, 0}, std::array<double, 3>{2, 0, 0},
                                           std::array<double, 3>{kNaN, 0, 0}, std::array<double, 3>{inf, 0, 0}}) {
    ReferencePlane plane;
    plane.custom = true;
    plane.custom_rgb = rgb;
    check_unresolved_custom(reference_plane_color(plane));
  }
  // Other shapes never become a custom RGB value. (A boolean channel such as
  // [true, 0, 0] has no representation in the typed model.)
  for (const J& rgb : {J(nullptr), arr({1, 0}), arr({1, 0, 0, 1}), J("red"), J(3)}) {
    J data = triangle();
    data["model"]["attributes"] = arr({plane_attribute(plane_values(unit_frame(), {"rgb", rgb}))});
    Model model = load(data);
    CHECK(model.reference_planes.size() == 1);
    CHECK(model.reference_planes[0].custom);
    CHECK(!model.reference_planes[0].custom_rgb);
    CHECK(!model.reference_planes[0].color_index);
    check_unresolved_custom(reference_plane_color(model.reference_planes[0]));
  }
  // The 'rgb' selector without a value.
  ReferencePlane plane;
  plane.custom = true;
  check_unresolved_custom(reference_plane_color(plane));
  J data = triangle();
  data["model"]["attributes"] = arr({plane_attribute(plane_values(unit_frame(), {"rgb"}))});
  Model model = load(data);
  CHECK(model.reference_planes.size() == 1);
  check_unresolved_custom(reference_plane_color(model.reference_planes[0]));
}

void unresolved_table_inherits_only_complete_single_body_uniform_color() {
  J data = triangle(1);
  J& model = data["model"];
  model["bodies"] = arr({obj({{"id", "b"}})});
  model["faces"] = arr({obj({{"id", "f"}, {"color", obj({{"r", 1}, {"g", 0}, {"b", 0}, {"a", 1}})}})});
  model["tessellations"].push_back(obj({{"body", "b"}, {"mesh", obj({{"kind", "unsupported"}})}}));
  auto material_count = [&] {
    Made made = make_glb(data);
    const J* materials = made.doc.find("materials");
    return materials ? materials->size() : 0;
  };
  CHECK(material_count() == 1);
  data["model"]["faces"].push_back(obj({{"id", "uncolored"}}));
  CHECK(material_count() == 0);
  data["model"]["faces"].arr().pop_back();
  data["model"]["bodies"].push_back(obj({{"id", "other"}}));
  CHECK(material_count() == 0);
}

void shaded_edges_keep_opaque_color_and_exclude_inactive_configuration() {
  J data = triangle();
  J& model = data["model"];
  model["attributes"] = arr({mode_attribute(3)});
  model["tessellations"].arr()[0]["source_object"] =
      obj({{"color", obj({{"r", 1}, {"g", 0}, {"b", 0}, {"a", 1}})}});
  model["configurations"] = arr({obj({{"active", true}, {"bodies", arr({"active"})}})});
  model["regions"] = arr({obj({{"id", "r"}, {"body", "active"}})});
  model["shells"] = arr({obj({{"id", "s"}, {"region", "r"}})});
  model["faces"] = arr({obj({{"id", "f"}, {"shell", "s"}, {"loops", obj({{"loops", arr({"l"})}})}})});
  model["loops"] = arr({obj({{"id", "l"}, {"boundary", obj({{"coedges", arr({"c"})}})}})});
  model["coedges"] = arr({obj({{"id", "c"}, {"edge", "shown"}})});
  J edges = J::array();
  for (const char* e : {"shown", "inactive"})
    edges.push_back(obj({{"id", e}, {"carrier", obj({{"curve", "line"}, {"param_range", arr({0, 1000})}})}}));
  model["edges"] = edges;
  model["curves"] = arr({obj({{"id", "line"},
                              {"geometry", obj({{"kind", "line"}, {"origin", xyz(0, 0, 0)},
                                                {"direction", xyz(1, 0, 0)}})}})});
  Made made = make_glb(data);
  CHECK(numbers_eq(at(at(at(at(made.doc, "materials"), 0), "pbrMetallicRoughness"), "baseColorFactor"),
                   {1, 0, 0, 1}));
  CHECK(at(at(at(at(at(made.doc, "nodes"), 0), "extras"), kKey), "mode").str() == "shadedEdges");
  size_t edge_accessor = static_cast<size_t>(
      at(at(at(at(at(at(made.doc, "meshes"), 1), "primitives"), 0), "attributes"), "POSITION").as_int());
  CHECK(at(at(at(made.doc, "accessors"), edge_accessor), "count").as_int() == 2);
}

void wireframe_keeps_reference_plane_and_hides_uncolored_surface() {
  J data = triangle();
  data["model"]["attributes"] =
      arr({mode_attribute(0),
           plane_attribute(plane_values(arr({0, 0, 0, 1000, 0, 0, 1000, 1000, 0, 0, 1000, 0, 0, 0, 0}), {}))});
  Made made = make_glb(data);
  const J& doc = made.doc;
  CHECK(at(at(at(at(at(doc, "materials"), 0), "pbrMetallicRoughness"), "baseColorFactor"), 3).as_double() == 0);
  CHECK(at(at(at(at(doc, "meshes"), 1), "primitives"), 0).find("mode")->as_int() == 1);
  const J& plane = at(at(at(at(doc, "nodes"), 1), "extras"), kKey);
  CHECK(at(plane, "label").str() == "Plane1");
  CHECK(numbers_eq(at(at(plane, "corners"), 2), {1, 0, -1}));
  CHECK(numbers_eq(at(plane, "advances"), std::vector<double>(6, .002)));
  data["model"]["attributes"].arr()[1]["values"].push_back(obj({{"value", 3}}));
  Made red = make_glb(data);
  const J& presentation = at(at(at(at(red.doc, "nodes"), 1), "extras"), kKey);
  CHECK(at(presentation, "colorIndex").as_int() == 3);
  CHECK(numbers_eq(at(presentation, "color"), {1, 0, 0}));
  size_t material = static_cast<size_t>(at(at(at(at(at(red.doc, "meshes"), 1), "primitives"), 0), "material").as_int());
  CHECK(numbers_eq(at(at(at(at(red.doc, "materials"), material), "pbrMetallicRoughness"), "baseColorFactor"),
                   {1, 0, 0, 1}));
}

void periodic_seam_is_removed_but_boundary_remains() {
  J edges = J::array();
  for (const char* e : {"seam", "boundary"})
    edges.push_back(obj({{"id", e}, {"carrier", obj({{"curve", "c"}, {"param_range", arr({0, 1000})}})}}));
  J coedges = J::array();
  for (auto [c, e] : {std::pair{"a", "seam"}, std::pair{"b", "seam"}, std::pair{"c", "boundary"}})
    coedges.push_back(obj({{"id", c}, {"edge", e}}));
  J model = obj({{"curves", arr({obj({{"id", "c"},
                                       {"geometry", obj({{"kind", "line"}, {"origin", xyz(0, 0, 0)},
                                                         {"direction", xyz(1, 0, 0)}})}})})},
                 {"edges", edges},
                 {"faces", arr({obj({{"id", "f"}, {"loops", obj({{"loops", arr({"l"})}})}})})},
                 {"loops", arr({obj({{"id", "l"}, {"boundary", obj({{"coedges", arr({"a", "b", "c"})}})}})})},
                 {"coedges", coedges}});
  CHECK((wire_edges(load(obj({{"model", model}}))) == std::vector<P3>{{0, 0, 0}, {1, 0, 0}}));
}

void line_without_parameter_range_uses_topology_endpoints() {
  J model = obj({{"curves", arr({obj({{"id", "c"}, {"geometry", obj({{"kind", "line"}})}})})},
                 {"edges", arr({obj({{"id", "e"}, {"carrier", obj({{"curve", "c"}})}, {"start", "a"}, {"end", "b"}})})},
                 {"vertices", arr({obj({{"id", "a"}, {"point", "p"}}), obj({{"id", "b"}, {"point", "q"}})})},
                 {"points", arr({obj({{"id", "p"}, {"position", xyz(0, 0, 0)}}),
                                 obj({{"id", "q"}, {"position", xyz(1000, 2000, 3000)}})})}});
  CHECK((wire_edges(load(obj({{"model", model}}))) == std::vector<P3>{{0, 0, 0}, {1, 3, -2}}));
}

void strip_winding_scale_and_degenerate_removal() {
  Made made = make_glb(sample(arr({strip({vertex(0, 0), vertex(1000, 0), vertex(0, 1000), vertex(1000, 1000),
                                          vertex(1000, 1000)})})));
  CHECK(made.triangles == 2 && made.tables == 1);
  const Bytes& glb = made.glb;
  CHECK(std::memcmp(glb.data(), "glTF", 4) == 0);
  CHECK(le32(glb, 4) == 2 && le32(glb, 8) == glb.size());
  size_t start = 20 + le32(glb, 12) + 8;
  size_t offset = static_cast<size_t>(at(at(at(made.doc, "bufferViews"), 2), "byteOffset").as_int());
  const uint32_t expected[6] = {0, 1, 2, 2, 1, 3};
  for (size_t i = 0; i < 6; ++i) CHECK(le32(glb, start + offset + 4 * i) == expected[i]);
  CHECK(numbers_eq(at(at(at(made.doc, "accessors"), 0), "min"), {0.0, 0.0, -1.0}));
  CHECK(numbers_eq(at(at(at(made.doc, "accessors"), 0), "max"), {1.0, 0.0, 0.0}));
}

void unreadable_strips_do_not_hide_usable_geometry() {
  J data = sample(arr({strip({vertex(0, 0), vertex(1000, 0), vertex(0, 1000)}),
                       strip({vertex(0, 0), vertex(1, 0), vertex(0, 1)})}));
  data["model"]["tessellations"].push_back(obj({{"mesh", obj({{"kind", "unsupported"}})}}));
  Model model = load(data);
  model.tessellations[0].strips[1][0].position.x = kNaN;  // JSON can not carry NaN.
  CHECK(make_glb(model).triangles == 1);
}

void face_color_and_transparency_are_written_to_glb() {
  J data = triangle();
  data["model"]["faces"] = arr({obj({{"id", "f"}, {"color", obj({{"r", 1}, {"g", 0.5}, {"b", 0}, {"a", 0.4}})}})});
  data["model"]["tessellations"].arr()[0]["faces"] = arr({"f"});
  Made made = make_glb(data);
  const J& material = at(at(made.doc, "materials"), 0);
  CHECK(at(material, "alphaMode").str() == "BLEND");
  const J& rgba = at(at(material, "pbrMetallicRoughness"), "baseColorFactor");
  CHECK(std::abs(at(rgba, 1).as_double() - 0.21404114) < 5e-8);  // assertAlmostEqual
  CHECK(at(rgba, 0).as_double() == 1 && at(rgba, 2).as_double() == 0 && at(rgba, 3).as_double() == 0.4);
  CHECK(at(at(at(at(at(made.doc, "meshes"), 0), "primitives"), 0), "material").as_int() == 0);
}

void unbound_mesh_does_not_borrow_another_body_color() {
  Model model = load(obj({{"model", obj({{"bodies", arr({obj({{"id", "b"},
                                                               {"color", obj({{"r", 1}, {"g", 0}, {"b", 0}, {"a", 1}})}})})}})}}));
  Lookup lookup = lookup_of(model);
  CHECK(!table_color(Tessellation{}, lookup));
  Tessellation bound;
  bound.body = "b";
  auto color = table_color(bound, lookup);
  CHECK(color && (*color == std::array<double, 4>{1, 0, 0, 1}));
}

void assembly_display_binding_without_brep_is_written() {
  J data = triangle();
  data["model"]["tessellations"].arr()[0]["id"] = "display:1";
  data["model"]["appearances"] =
      arr({obj({{"id", "red"}, {"base_color", obj({{"r", 1}, {"g", 0}, {"b", 0}, {"a", 1}})}})});
  data["model"]["appearance_bindings"] =
      arr({obj({{"target", obj({{"kind", "tessellation"}, {"id", "display:1"}})}, {"appearance", "red"}})});
  Made made = make_glb(data);
  CHECK(numbers_eq(at(at(at(at(made.doc, "materials"), 0), "pbrMetallicRoughness"), "baseColorFactor"),
                   {1, 0, 0, 1}));
  data["model"]["appearance_bindings"].arr()[0]["target"]["id"] = "another mesh";
  CHECK(!has(make_glb(data).doc, "materials"));
}

void distinct_assembly_colors_keep_separate_triangle_ranges() {
  J data = triangle();
  J table = data["model"]["tessellations"].arr()[0];
  J tables = J::array();
  for (int i = 0; i < 3; ++i) {
    J copy = table.clone();
    copy["id"] = std::to_string(i);
    tables.push_back(copy);
  }
  data["model"]["tessellations"] = tables;
  data["model"]["appearances"] =
      arr({obj({{"id", "red"}, {"base_color", obj({{"r", 1}, {"g", 0}, {"b", 0}, {"a", 1}})}}),
           obj({{"id", "green"}, {"base_color", obj({{"r", 0}, {"g", 1}, {"b", 0}, {"a", 1}})}})});
  data["model"]["appearance_bindings"] =
      arr({obj({{"target", obj({{"kind", "tessellation"}, {"id", "0"}})}, {"appearance", "red"}}),
           obj({{"target", obj({{"kind", "tessellation"}, {"id", "1"}})}, {"appearance", "green"}})});
  Made made = make_glb(data);
  const J& primitives = at(at(at(made.doc, "meshes"), 0), "primitives");
  CHECK(made.triangles == 3 && primitives.size() == 3);
  for (size_t i = 0; i < 3; ++i) {
    const J& p = at(primitives, i);
    CHECK(at(at(at(made.doc, "accessors"), static_cast<size_t>(at(p, "indices").as_int())), "byteOffset").as_int() ==
          static_cast<int64_t>(12 * i));
    if (i < 2) CHECK(at(p, "material").as_int() == static_cast<int64_t>(i));
    else CHECK(!has(p, "material"));
  }
}

void identical_colors_share_one_primitive() {
  J data = triangle();
  J table = data["model"]["tessellations"].arr()[0];
  J tables = J::array();
  for (int i = 0; i < 100; ++i) tables.push_back(table);
  data["model"]["tessellations"] = tables;
  Made made = make_glb(data);
  CHECK(made.triangles == 100);
  CHECK(at(at(at(made.doc, "meshes"), 0), "primitives").size() == 1);
}

void hidden_assembly_components_do_not_expand_visible_bounds() {
  J data = triangle();
  J hidden = shaded(arr({strip({vertex(9000, 0), vertex(10000, 0), vertex(9000, 1000)})}));
  hidden["source_object"] = obj({{"visible", false}});
  data["model"]["tessellations"].push_back(hidden);
  Made made = make_glb(data);
  CHECK(made.triangles == 1);
  CHECK(numbers_eq(at(at(at(made.doc, "accessors"), 0), "max"), {1.0, 0.0, 0.0}));
}

void empty_or_invalid_mesh_is_rejected() {
  CHECK(throws_decode([] { make_glb(sample(arr({strip({vertex(0, 0), vertex(0, 0), vertex(0, 0)})}))); }));
  Model model = load(sample(arr({strip({vertex(0, 0), vertex(1, 0), vertex(0, 1)})})));
  model.tessellations[0].strips[0][0].position.x = kNaN;  // JSON can not carry NaN.
  CHECK(throws_decode([&] { make_glb(model); }));
}

}  // namespace

int main() {
  stale_surface_is_replaced_without_losing_boundary_lines();
  bounds_tolerance_avoids_rebuilding_small_tessellation_error();
  unsupported_boundary_preserves_visible_surface();
  native_reference_pen_colors_reach_both_frame_and_label();
  unknown_reference_pen_does_not_wrap_or_guess_rgb();
  custom_reference_rgb_reaches_frame_and_label_without_palette_quantization();
  invalid_custom_rgb_is_unresolved_and_never_clamped_or_used_as_index();
  unresolved_table_inherits_only_complete_single_body_uniform_color();
  shaded_edges_keep_opaque_color_and_exclude_inactive_configuration();
  wireframe_keeps_reference_plane_and_hides_uncolored_surface();
  periodic_seam_is_removed_but_boundary_remains();
  line_without_parameter_range_uses_topology_endpoints();
  strip_winding_scale_and_degenerate_removal();
  unreadable_strips_do_not_hide_usable_geometry();
  face_color_and_transparency_are_written_to_glb();
  unbound_mesh_does_not_borrow_another_body_color();
  assembly_display_binding_without_brep_is_written();
  distinct_assembly_colors_keep_separate_triangle_ranges();
  identical_colors_share_one_primitive();
  hidden_assembly_components_do_not_expand_visible_bounds();
  empty_or_invalid_mesh_is_rejected();
  std::puts("glb_test: ok");
  return 0;
}
