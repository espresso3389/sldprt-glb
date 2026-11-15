// SPDX-License-Identifier: Apache-2.0
// Port of the reference CADIR-to-GLB writer (make_glb,
// table_color, wire_edges, reference_plane_color, stale_surface_bounds).
// The output is byte-identical to the reference writer for the same decoded model:
// arithmetic follows the Python expressions in the same order, colours use
// the doubles Python parses from the CADIR's f32 text, and JSON follows
// json.dumps(separators=(',', ':')).
#include "glb.h"

#include <cmath>
#include <cstring>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>

#include "errors.h"
#include "json_value.h"
#include "regenerate.h"

namespace swglb {
namespace {

constexpr size_t kMaxVertices = 4'000'000;

/// the reference writer REFERENCE_PEN_RGB: eDrawings 2026 default palette, ids 0..54.
std::optional<std::array<int, 3>> reference_pen_rgb(int64_t index) {
  static const std::map<int64_t, std::array<int, 3>> table = [] {
    std::map<int64_t, std::array<int, 3>> t;
    auto set = [&](std::initializer_list<int64_t> ids, std::array<int, 3> rgb) {
      for (auto id : ids) t[id] = rgb;
    };
    set({0, 1, 4, 12, 13, 18}, {0, 0, 0});
    set({2, 8, 15, 22, 23}, {192, 192, 192});
    set({3, 17, 31}, {255, 0, 0});
    set({5}, {0, 0, 255});
    set({6, 9, 16}, {128, 128, 0});
    set({7, 24}, {255, 255, 0});
    set({10, 11, 28, 29}, {255, 255, 255});
    set({14}, {0, 255, 0});
    set({19, 21}, {128, 128, 128});
    set({20, 25, 26}, {255, 128, 128});
    set({27, 30}, {0, 128, 192});
    set({32}, {128, 0, 0});
    set({33}, {0, 128, 0});
    set({34, 41, 47, 48, 52}, {0, 0, 0});
    set({35, 50, 53}, {255, 0, 0});
    set({36}, {0, 255, 0});
    set({37, 40, 51}, {0, 0, 255});
    set({38, 44}, {128, 128, 128});
    set({39}, {0, 64, 0});
    set({42}, {128, 255, 255});
    set({43, 45, 54}, {192, 192, 192});
    set({46}, {255, 128, 0});
    set({49}, {255, 0, 128});
    return t;
  }();
  auto it = table.find(index);
  if (it == table.end()) return std::nullopt;
  return it->second;
}

double srgb_to_linear(double c) {
  return c <= 0.04045 ? c / 12.92 : std::pow((c + 0.055) / 1.055, 2.4);
}

/// Colour as Python reads it from the CADIR JSON (4 doubles).
std::array<double, 4> json_color(const Color& c) {
  return {f32_as_json_double(c.r), f32_as_json_double(c.g), f32_as_json_double(c.b),
          f32_as_json_double(c.a)};
}

/// the reference writer table_color's final conversion: linear RGB + alpha.
std::array<double, 4> linear_rgba(const std::array<double, 4>& rgba) {
  for (double c : rgba)
    if (!std::isfinite(c) || c < 0 || c > 1) throw DecodeError("Invalid display color");
  return {srgb_to_linear(rgba[0]), srgb_to_linear(rgba[1]), srgb_to_linear(rgba[2]), rgba[3]};
}

P3 cad_position(const P3& v) { return {v[0] * 0.001, v[2] * 0.001, -v[1] * 0.001}; }
P3 cad_position(const Vec3& v) { return cad_position(P3{v.x, v.y, v.z}); }

/// Python min()/max(): the first extreme element wins ties (-0.0 vs 0.0).
void python_bounds(const std::vector<P3>& points, P3* lo, P3* hi) {
  *lo = points.front();
  *hi = points.front();
  for (const auto& p : points)
    for (int i = 0; i < 3; ++i) {
      if (p[i] < (*lo)[i]) (*lo)[i] = p[i];
      if (p[i] > (*hi)[i]) (*hi)[i] = p[i];
    }
}

JValue jvec(const P3& p) { return JValue::Array{p[0], p[1], p[2]}; }

struct Lookup {
  std::unordered_map<std::string, const Face*> faces;
  std::unordered_map<std::string, const Body*> bodies;
  /// Tessellation id -> bound appearance colour (nullopt: bound appearance
  /// without base colour).
  std::unordered_map<std::string, std::optional<std::array<double, 4>>> bound;
};

/// the reference writer table_color (returns nullopt for None).
std::optional<std::array<double, 4>> table_color(const Tessellation& table, const Lookup& lookup) {
  std::optional<std::array<double, 4>> color;
  if (auto it = lookup.bound.find(table.id); it != lookup.bound.end()) color = it->second;
  if (!color && table.source_object && table.source_object->color)
    color = json_color(*table.source_object->color);
  if (!color && !table.faces.empty()) {
    std::vector<std::optional<std::array<double, 4>>> owned;
    for (const auto& key : table.faces) {
      auto it = lookup.faces.find(key);
      if (it != lookup.faces.end() && it->second->color)
        owned.push_back(json_color(*it->second->color));
      else
        owned.push_back(std::nullopt);
    }
    bool all_same = true;
    for (const auto& c : owned) all_same &= (c == owned[0]);
    if (all_same) color = owned[0];
  }
  if (!color && table.body) {
    auto it = lookup.bodies.find(*table.body);
    if (it != lookup.bodies.end() && it->second->color) color = json_color(*it->second->color);
  }
  if (!color) return std::nullopt;
  return linear_rgba(*color);
}

}  // namespace

// ---------------------------------------------------------------------------
// the reference writer wire_edges
// ---------------------------------------------------------------------------
std::vector<P3> wire_edges(const Model& model) {
  std::unordered_map<std::string, const CurveGeometry*> curves;
  for (const auto& c : model.curves) curves.emplace(c.id, &c.geometry);
  std::unordered_map<std::string, const Vec3*> points_by_id;
  for (const auto& p : model.points) points_by_id.emplace(p.id, &p.position);
  std::unordered_map<std::string, const Vec3*> vertices;
  for (const auto& v : model.vertices) {
    auto it = points_by_id.find(v.point);
    vertices.emplace(v.id, it == points_by_id.end() ? nullptr : it->second);
  }
  std::unordered_map<std::string, const Loop*> loops;
  for (const auto& l : model.loops) loops.emplace(l.id, &l);
  std::unordered_map<std::string, const Coedge*> coedges;
  for (const auto& c : model.coedges) coedges.emplace(c.id, &c);

  std::vector<const Configuration*> active;
  for (const auto& c : model.configurations)
    if (c.active) active.push_back(&c);
  std::set<std::string> active_bodies;
  if (active.size() == 1) active_bodies.insert(active[0]->bodies.begin(), active[0]->bodies.end());
  std::set<std::string> regions, shells;
  for (const auto& r : model.regions)
    if (active_bodies.count(r.body)) regions.insert(r.id);
  for (const auto& s : model.shells)
    if (regions.count(s.region)) shells.insert(s.id);

  std::unordered_map<std::string, std::vector<std::string>> owners;
  for (const auto& face : model.faces) {
    if (!active_bodies.empty() && !shells.count(face.shell)) continue;
    for (const auto& loop_id : face.loops) {
      auto lit = loops.find(loop_id);
      if (lit == loops.end()) continue;
      for (const auto& coedge_id : lit->second->coedges) {
        auto cit = coedges.find(coedge_id);
        // Python: coedges.get(id, {}).get('edge') -> None for a missing coedge.
        std::string edge_id = cit == coedges.end() ? std::string("\x01none") : cit->second->edge;
        owners[edge_id].push_back(face.id);
      }
    }
  }

  std::vector<P3> lines;
  for (const auto& edge : model.edges) {
    auto oit = owners.find(edge.id);
    static const std::vector<std::string> kNone;
    const auto& face_ids = oit == owners.end() ? kNone : oit->second;
    if (!active_bodies.empty() && face_ids.empty()) continue;
    if (face_ids.size() == 2 && face_ids[0] == face_ids[1]) continue;
    const CurveGeometry* geometry = nullptr;
    if (edge.curve)
      if (auto it = curves.find(*edge.curve); it != curves.end()) geometry = it->second;
    const std::string kind = geometry ? geometry->kind : "";
    const auto& interval = edge.param_range;
    if (kind == "line" && !interval) {
      const Vec3* ends[2] = {nullptr, nullptr};
      const std::optional<std::string>* keys[2] = {&edge.start, &edge.end};
      for (int k = 0; k < 2; ++k)
        if (*keys[k])
          if (auto it = vertices.find(**keys[k]); it != vertices.end()) ends[k] = it->second;
      if (!ends[0] || !ends[1]) throw DecodeError("Missing line endpoints");
      lines.push_back(cad_position(*ends[0]));
      lines.push_back(cad_position(*ends[1]));
      continue;
    }
    if (!interval || (kind != "circle" && kind != "line")) throw DecodeError("Unsupported wire edge");
    double t0 = (*interval)[0], t1 = (*interval)[1];
    std::vector<P3> points;
    if (kind == "circle") {
      P3 center{geometry->center.x, geometry->center.y, geometry->center.z};
      P3 axis{geometry->axis.x, geometry->axis.y, geometry->axis.z};
      P3 u{geometry->ref_direction.x, geometry->ref_direction.y, geometry->ref_direction.z};
      P3 v{axis[1] * u[2] - axis[2] * u[1], axis[2] * u[0] - axis[0] * u[2],
           axis[0] * u[1] - axis[1] * u[0]};
      double span = std::ceil(std::abs(t1 - t0) * 24);
      if (!std::isfinite(span)) throw DecodeError("Invalid circle interval");
      int64_t count = static_cast<int64_t>(std::max(8.0, std::min(512.0, span)));
      for (int64_t i = 0; i <= count; ++i) {
        double t = t0 + (t1 - t0) * static_cast<double>(i) / static_cast<double>(count);
        P3 p;
        for (int j = 0; j < 3; ++j)
          p[j] = center[j] + geometry->radius * (u[j] * std::cos(t) + v[j] * std::sin(t));
        points.push_back(cad_position(p));
      }
    } else {
      P3 origin{geometry->origin.x, geometry->origin.y, geometry->origin.z};
      P3 direction{geometry->direction.x, geometry->direction.y, geometry->direction.z};
      for (double t : {t0, t1}) {
        P3 p;
        for (int j = 0; j < 3; ++j) p[j] = origin[j] + direction[j] * t;
        points.push_back(cad_position(p));
      }
    }
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      lines.push_back(points[i]);
      lines.push_back(points[i + 1]);
    }
  }
  return lines;
}

namespace {

/// the reference writer reference_plane_color: (selector, srgb, resolved, source).
struct PlaneColor {
  JValue selector;  // int or null
  std::array<double, 3> srgb;
  bool resolved;
  const char* source;
};

PlaneColor reference_plane_color(const ReferencePlane& plane) {
  if (plane.custom) {
    if (plane.custom_rgb) {
      const auto& rgb = *plane.custom_rgb;
      bool ok = true;
      for (double c : rgb) ok &= std::isfinite(c) && c >= 0 && c <= 1;
      if (ok) return {nullptr, rgb, true, "custom"};
    }
    return {nullptr, {0.8, 0.8, 0.8}, false, "custom"};
  }
  if (!plane.color_index) return {nullptr, {0.8, 0.8, 0.8}, false, "indexed"};
  int64_t index = *plane.color_index;
  if (auto rgb = reference_pen_rgb(index))
    return {index, {(*rgb)[0] / 255.0, (*rgb)[1] / 255.0, (*rgb)[2] / 255.0}, true, "indexed"};
  if (index >= 0 && index <= 0xffffffffLL) return {index, {0.0, 0.0, 0.0}, false, "indexed"};
  return {index, {0.8, 0.8, 0.8}, false, "indexed"};
}

class GlbBuilder {
 public:
  JValue doc = JValue::object();
  Bytes binary;
  JValue* views = nullptr;

  size_t add_buffer_f32x3(const std::vector<P3>& values, int target) {
    size_t start = binary.size();
    binary.reserve(binary.size() + values.size() * 12);
    for (const auto& v : values)
      for (double c : v) put_f32(static_cast<float>(c));
    return add_view(start, target);
  }
  size_t add_buffer_u32(const std::vector<uint32_t>& values, int target) {
    size_t start = binary.size();
    binary.reserve(binary.size() + values.size() * 4);
    for (uint32_t v : values) put_u32(v);
    return add_view(start, target);
  }

 private:
  size_t add_view(size_t start, int target) {
    JValue view = JValue::object();
    view["buffer"] = 0;
    view["byteOffset"] = start;
    view["byteLength"] = binary.size() - start;
    view["target"] = target;
    (*views).push_back(std::move(view));
    return (*views).size() - 1;
  }
  void put_f32(float f) {
    uint32_t raw;
    std::memcpy(&raw, &f, 4);
    put_u32(raw);
  }
  void put_u32(uint32_t v) {
    binary.push_back(static_cast<uint8_t>(v));
    binary.push_back(static_cast<uint8_t>(v >> 8));
    binary.push_back(static_cast<uint8_t>(v >> 16));
    binary.push_back(static_cast<uint8_t>(v >> 24));
  }
};

}  // namespace

Bytes glb_encode(JValue doc, Bytes binary) {
  doc["buffers"].arr()[0]["byteLength"] = binary.size();
  std::string encoded = doc.dump();
  encoded.append((4 - encoded.size() % 4) % 4, ' ');
  binary.resize(binary.size() + (4 - binary.size() % 4) % 4, 0);
  size_t total = 12 + 8 + encoded.size() + 8 + binary.size();
  Bytes out;
  out.reserve(total);
  auto u32 = [&](uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
  };
  out.insert(out.end(), {'g', 'l', 'T', 'F'});
  u32(2);
  u32(static_cast<uint32_t>(total));
  u32(static_cast<uint32_t>(encoded.size()));
  out.insert(out.end(), {'J', 'S', 'O', 'N'});
  out.insert(out.end(), encoded.begin(), encoded.end());
  u32(static_cast<uint32_t>(binary.size()));
  out.insert(out.end(), {'B', 'I', 'N', '\0'});
  out.insert(out.end(), binary.begin(), binary.end());
  return out;
}

/// the reference writer make_glb. `surface_bounds` receives the POSITION accessor
/// bounds of the first surface primitive (for stale_surface_bounds).
static Bytes make_glb(const Model& model, const GlbOptions& options, Bounds* surface_bounds) {
  if (model.length_unit != "millimeter") throw DecodeError("Expected CADIR positions in millimeters");
  const auto& tables = model.tessellations;

  Lookup lookup;
  for (const auto& f : model.faces) lookup.faces.emplace(f.id, &f);
  for (const auto& b : model.bodies) lookup.bodies.emplace(b.id, &b);
  {
    std::unordered_map<std::string, std::optional<std::array<double, 4>>> appearances;
    for (const auto& a : model.appearances)
      appearances[a.id] = a.base_color ? std::optional(json_color(*a.base_color)) : std::nullopt;
    for (const auto& binding : model.appearance_bindings) {
      if (binding.target_kind != "tessellation") continue;
      std::optional<std::array<double, 4>> color;
      if (auto it = appearances.find(binding.appearance); it != appearances.end()) color = it->second;
      auto [it, inserted] = lookup.bound.emplace(binding.target_id, color);
      if (!inserted && it->second != color)
        throw DecodeError("Conflicting display appearance bindings");
    }
  }
  // Python stores the bound colour as-is; table_color converts it later.
  std::optional<std::array<double, 4>> uniform_color;
  if (model.bodies.size() == 1 && !model.faces.empty() && model.occurrences.empty()) {
    const auto& first = model.faces[0].color;
    bool all_same = first.has_value();
    for (const auto& f : model.faces) all_same &= f.color.has_value() && *f.color == *first;
    if (all_same) {
      bool any_body = false;
      for (const auto& t : tables) any_body |= t.body && *t.body == model.bodies[0].id;
      if (any_body) uniform_color = linear_rgba(json_color(*first));
    }
  }

  std::vector<P3> positions, normals;
  std::vector<uint32_t> indices;
  struct Group {
    size_t offset, count;
    std::optional<std::array<double, 4>> color;
  };
  std::vector<Group> groups;
  for (const auto& table : tables) {
    if (table.source_object && table.source_object->visible == false) continue;
    if (!table.has_normals) continue;  // Only shaded_strips tables are drawn.
    size_t first_index = indices.size();
    for (const auto& strip : table.strips) {
      if (positions.size() + strip.size() > kMaxVertices) throw DecodeError("Vertex limit exceeded");
      std::vector<P3> pending_positions, pending_normals;
      pending_positions.reserve(strip.size());
      pending_normals.reserve(strip.size());
      bool readable = true;
      for (const auto& vertex : strip) {
        P3 pos{vertex.position.x * 0.001, vertex.position.z * 0.001, vertex.position.y * -0.001};
        P3 norm{vertex.normal.x * 1, vertex.normal.z * 1, vertex.normal.y * -1};
        for (double n : pos) readable &= std::isfinite(n) && std::abs(n) <= 3.4e38;
        for (double n : norm) readable &= std::isfinite(n) && std::abs(n) <= 3.4e38;
        if (!readable) break;
        pending_positions.push_back(pos);
        pending_normals.push_back(norm);
      }
      if (!readable) continue;
      size_t base = positions.size();
      positions.insert(positions.end(), pending_positions.begin(), pending_positions.end());
      normals.insert(normals.end(), pending_normals.begin(), pending_normals.end());
      for (size_t i = 0; i + 2 < strip.size(); ++i) {
        size_t a = base + i, b = base + i + 1, c = base + i + 2;
        if (i % 2) std::swap(a, b);
        const P3 &p = positions[a], &q = positions[b], &r = positions[c];
        P3 u{q[0] - p[0], q[1] - p[1], q[2] - p[2]};
        P3 v{r[0] - p[0], r[1] - p[1], r[2] - p[2]};
        P3 cross{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
        if (cross[0] != 0 || cross[1] != 0 || cross[2] != 0) {
          indices.push_back(static_cast<uint32_t>(a));
          indices.push_back(static_cast<uint32_t>(b));
          indices.push_back(static_cast<uint32_t>(c));
        }
      }
    }
    if (indices.size() > first_index) {
      auto color = table_color(table, lookup);
      groups.push_back({first_index, indices.size() - first_index, color ? color : uniform_color});
    }
  }
  if (indices.empty()) throw DecodeError("No non-degenerate saved triangles");

  // One primitive per effective colour, in order of first occurrence.
  std::vector<std::pair<std::optional<std::array<double, 4>>, std::vector<uint32_t>>> grouped;
  std::map<std::optional<std::array<double, 4>>, size_t> group_index;
  for (const auto& g : groups) {
    auto [it, inserted] = group_index.emplace(g.color, grouped.size());
    if (inserted) grouped.push_back({g.color, {}});
    auto& members = grouped[it->second].second;
    members.insert(members.end(), indices.begin() + static_cast<std::ptrdiff_t>(g.offset),
                   indices.begin() + static_cast<std::ptrdiff_t>(g.offset + g.count));
  }
  indices.clear();
  groups.clear();
  for (auto& [color, members] : grouped) {
    groups.push_back({indices.size(), members.size(), color});
    indices.insert(indices.end(), members.begin(), members.end());
  }

  GlbBuilder glb;
  JValue& doc = glb.doc;
  JValue views = JValue::array();
  glb.views = &views;
  glb.add_buffer_f32x3(positions, 34962);
  glb.add_buffer_f32x3(normals, 34962);
  glb.add_buffer_u32(indices, 34963);
  P3 minimum, maximum;
  python_bounds(positions, &minimum, &maximum);
  if (surface_bounds) *surface_bounds = {minimum, maximum};

  JValue asset = JValue::object();
  asset["version"] = "2.0";
  asset["generator"] = options.generator;
  doc["asset"] = asset;
  doc["scene"] = 0;
  JValue scene = JValue::object();
  scene["nodes"] = JValue::Array{0};
  doc["scenes"] = JValue::Array{scene};
  JValue node0 = JValue::object();
  node0["mesh"] = 0;
  doc["nodes"] = JValue::Array{node0};
  doc["meshes"] = JValue::Array{JValue::object()};
  JValue buffer = JValue::object();
  buffer["byteLength"] = glb.binary.size();
  doc["buffers"] = JValue::Array{buffer};
  doc["bufferViews"] = views;  // shares storage with `views`
  JValue accessors = JValue::array();
  {
    JValue a = JValue::object();
    a["bufferView"] = 0;
    a["componentType"] = 5126;
    a["count"] = positions.size();
    a["type"] = "VEC3";
    a["min"] = jvec(minimum);
    a["max"] = jvec(maximum);
    accessors.push_back(a);
    JValue n = JValue::object();
    n["bufferView"] = 1;
    n["componentType"] = 5126;
    n["count"] = normals.size();
    n["type"] = "VEC3";
    accessors.push_back(n);
    JValue i = JValue::object();
    i["bufferView"] = 2;
    i["componentType"] = 5125;
    i["count"] = indices.size();
    i["type"] = "SCALAR";
    accessors.push_back(i);
  }
  doc["accessors"] = accessors;

  JValue primitives = JValue::array();
  JValue materials = JValue::array();
  for (const auto& g : groups) {
    size_t accessor = accessors.size();
    JValue a = JValue::object();
    a["bufferView"] = 2;
    a["byteOffset"] = g.offset * 4;
    a["componentType"] = 5125;
    a["count"] = g.count;
    a["type"] = "SCALAR";
    accessors.push_back(a);
    JValue primitive = JValue::object();
    JValue attributes = JValue::object();
    attributes["POSITION"] = 0;
    attributes["NORMAL"] = 1;
    primitive["attributes"] = attributes;
    primitive["indices"] = accessor;
    if (g.color) {
      primitive["material"] = materials.size();
      JValue pbr = JValue::object();
      const auto& c = *g.color;
      pbr["baseColorFactor"] = JValue::Array{c[0], c[1], c[2], c[3]};
      pbr["metallicFactor"] = 0;
      pbr["roughnessFactor"] = 0.5;
      JValue material = JValue::object();
      material["pbrMetallicRoughness"] = pbr;
      material["alphaMode"] = c[3] < 1 ? "BLEND" : "OPAQUE";
      materials.push_back(material);
    }
    primitives.push_back(primitive);
  }
  doc["meshes"].arr()[0]["primitives"] = primitives;
  if (materials.size()) doc["materials"] = materials;

  bool wire = model.saved_display_mode == 0;
  bool shaded_edges = model.saved_display_mode == 3;
  std::vector<P3> boundaries;
  if (wire || shaded_edges) {
    try {
      boundaries = wire_edges(model);
    } catch (const DecodeError&) {
      // An unsupported boundary must not discard the decoded surface. In
      // wireframe mode keep it opaque so the resulting model stays visible.
      wire = shaded_edges = false;
    }
  }
  if (wire || shaded_edges) {
    JValue presentation = JValue::object();
    presentation["version"] = 1;
    presentation["role"] = "surface";
    presentation["mode"] = wire ? "wireframe" : "shadedEdges";
    JValue extras = JValue::object();
    extras[options.extras_key] = presentation;
    doc["nodes"].arr()[0]["extras"] = extras;
  }
  if (wire) {
    bool missing = false;
    for (const auto& p : primitives.arr()) missing |= !p.contains("material");
    if (missing) {
      size_t fallback = materials.size();
      JValue pbr = JValue::object();
      pbr["baseColorFactor"] = JValue::Array{1, 1, 1, 0};
      JValue material = JValue::object();
      material["pbrMetallicRoughness"] = pbr;
      materials.push_back(material);
      for (auto& p : primitives.arr()) p.setdefault("material", fallback);
    }
    doc["materials"] = materials;
    for (auto& material : materials.arr()) {
      material["pbrMetallicRoughness"]["baseColorFactor"].arr()[3] = 0;
      material["alphaMode"] = "BLEND";
    }
  }

  auto add_lines = [&](const std::vector<P3>& points, JValue color, const std::string& name,
                       JValue presentation) {
    if (points.empty()) return;
    if (points.size() > kMaxVertices) throw DecodeError("Invalid presentation geometry");
    for (const auto& p : points)
      for (double v : p)
        if (!std::isfinite(v)) throw DecodeError("Invalid presentation geometry");
    size_t view = views.size();
    glb.add_buffer_f32x3(points, 34962);
    size_t accessor = accessors.size();
    P3 lo, hi;
    python_bounds(points, &lo, &hi);
    JValue a = JValue::object();
    a["bufferView"] = view;
    a["componentType"] = 5126;
    a["count"] = points.size();
    a["type"] = "VEC3";
    a["min"] = jvec(lo);
    a["max"] = jvec(hi);
    accessors.push_back(a);
    JValue& doc_materials = doc.setdefault("materials", JValue::array());
    size_t material = doc_materials.size();
    color.push_back(1);
    JValue pbr = JValue::object();
    pbr["baseColorFactor"] = color;
    pbr["metallicFactor"] = 0;
    pbr["roughnessFactor"] = 1;
    JValue unlit = JValue::object();
    unlit["KHR_materials_unlit"] = JValue::object();
    JValue m = JValue::object();
    m["pbrMetallicRoughness"] = pbr;
    m["extensions"] = unlit;
    doc_materials.push_back(m);
    size_t mesh = doc["meshes"].size();
    JValue attributes = JValue::object();
    attributes["POSITION"] = accessor;
    JValue primitive = JValue::object();
    primitive["attributes"] = attributes;
    primitive["mode"] = 1;
    primitive["material"] = material;
    JValue mesh_value = JValue::object();
    mesh_value["primitives"] = JValue::Array{primitive};
    doc["meshes"].push_back(mesh_value);
    size_t node = doc["nodes"].size();
    JValue extras_inner = JValue::object();
    extras_inner["version"] = 1;
    for (auto& [k, v] : presentation.obj()) extras_inner[k] = v;
    JValue extras = JValue::object();
    extras[options.extras_key] = extras_inner;
    JValue node_value = JValue::object();
    node_value["mesh"] = mesh;
    node_value["name"] = name;
    node_value["extras"] = extras;
    doc["nodes"].push_back(node_value);
    doc["scenes"].arr()[0]["nodes"].push_back(node);
    doc["extensionsUsed"] = JValue::Array{"KHR_materials_unlit"};
  };

  if (wire || shaded_edges) {
    JValue presentation = JValue::object();
    presentation["role"] = "boundary";
    add_lines(boundaries, JValue::Array{0, 0, 0}, "CAD boundaries", presentation);
  }
  for (const auto& plane : model.reference_planes) {
    PlaneColor pc = reference_plane_color(plane);
    std::array<double, 3> linear{srgb_to_linear(pc.srgb[0]), srgb_to_linear(pc.srgb[1]),
                                 srgb_to_linear(pc.srgb[2])};
    std::vector<P3> points;
    for (const auto& c : plane.corners) points.push_back(cad_position(c));
    std::vector<P3> segments;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
      segments.push_back(points[i]);
      segments.push_back(points[i + 1]);
    }
    JValue presentation = JValue::object();
    presentation["role"] = "referencePlane";
    presentation["label"] = plane.label;
    presentation["origin"] = jvec(cad_position(plane.origin));
    presentation["colorIndex"] = pc.selector;
    presentation["color"] = JValue::Array{pc.srgb[0], pc.srgb[1], pc.srgb[2]};
    presentation["colorResolved"] = pc.resolved;
    presentation["colorSource"] = pc.source;
    JValue corners = JValue::array();
    for (const auto& p : points) corners.push_back(jvec(p));
    presentation["corners"] = corners;
    presentation["font"] = plane.font;
    presentation["height"] = static_cast<double>(plane.point_size) * 0.0254 / 72 * plane.scale;
    JValue advances = JValue::array();
    for (double v : plane.advances) advances.push_back(v * 0.001 * plane.scale);
    presentation["advances"] = advances;
    add_lines(segments, JValue::Array{linear[0], linear[1], linear[2]}, plane.label, presentation);
  }
  doc["buffers"].arr()[0]["byteLength"] = glb.binary.size();
  return glb_encode(doc, std::move(glb.binary));
}

// ---------------------------------------------------------------------------
// the reference writer bounds_differ / stale_surface_bounds
// ---------------------------------------------------------------------------
bool bounds_differ(const Bounds& a, const Bounds& b) {
  double scale = -INFINITY;
  for (int i = 0; i < 3; ++i)
    scale = std::max(scale, std::max(a.second[i] - a.first[i], b.second[i] - b.first[i]));
  double tolerance = std::max(0.00005, scale * 0.001);
  for (int i = 0; i < 3; ++i)
    if (std::abs(a.first[i] - b.first[i]) > tolerance ||
        std::abs(a.second[i] - b.second[i]) > tolerance)
      return true;
  return false;
}

static std::optional<Bounds> stale_surface_bounds(const Model& model,
                                                             const Bounds& surface) {
  if (model.bodies.size() != 1 || model.edges.empty()) return std::nullopt;
  if (model.surfaces.empty()) return std::nullopt;
  for (const auto& s : model.surfaces)
    if (s.geometry.kind != "plane" && s.geometry.kind != "cylinder") return std::nullopt;
  std::vector<P3> lines;
  try {
    lines = wire_edges(model);
  } catch (const DecodeError&) {
    return std::nullopt;
  }
  if (lines.empty()) return std::nullopt;
  P3 lo, hi;
  python_bounds(lines, &lo, &hi);
  Bounds boundary{lo, hi};
  if (!bounds_differ(surface, boundary)) return std::nullopt;
  return boundary;
}

Bytes write_glb(const Model& model, const GlbOptions& options) {
  Bounds surface;
  Bytes output = make_glb(model, options, &surface);
  // Part display caches can predate the active B-rep. Rebuild only when
  // complete analytic boundaries prove that the bounds differ.
  if (options.part) {
    if (auto boundary = stale_surface_bounds(model, surface))
      output = regenerate_stale_surface(model, output, *boundary, options.extras_key);
  }
  return output;
}

}  // namespace swglb
