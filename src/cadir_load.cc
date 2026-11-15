// SPDX-License-Identifier: Apache-2.0
// Load the subset of a cadmpeg CADIR JSON document that the reference writer reads.
// Used to verify the GLB writer independently of the decoder.
#include "cadir_load.h"

#include <cmath>

#include "errors.h"
#include "json_value.h"

namespace swglb {
namespace {

const JValue* member(const JValue& v, std::string_view key) {
  return v.is_object() ? v.find(key) : nullptr;
}

std::optional<std::string> opt_string(const JValue& v, std::string_view key) {
  const JValue* m = member(v, key);
  if (!m || !m->is_string()) return std::nullopt;
  return m->str();
}

std::string string_or_empty(const JValue& v, std::string_view key) {
  return opt_string(v, key).value_or("");
}

double number(const JValue& v) { return v.as_double(); }

Vec3 vec3(const JValue* v) {
  if (!v || !v->is_object()) return {};
  auto c = [&](const char* k) {
    const JValue* m = v->find(k);
    return m ? number(*m) : 0.0;
  };
  return {c("x"), c("y"), c("z")};
}

std::optional<Color> color(const JValue& v, std::string_view key) {
  const JValue* c = member(v, key);
  if (!c || !c->is_object()) return std::nullopt;
  auto ch = [&](const char* k) { return static_cast<float>(number(*c->find(k))); };
  return Color{ch("r"), ch("g"), ch("b"), ch("a")};
}

std::vector<std::string> strings(const JValue* v) {
  std::vector<std::string> out;
  if (v && v->is_array())
    for (const auto& s : v->arr())
      if (s.is_string()) out.push_back(s.str());
  return out;
}

const JValue::Array& list(const JValue& model, std::string_view key) {
  static const JValue::Array empty;
  const JValue* v = member(model, key);
  return v && v->is_array() ? v->arr() : empty;
}

bool reversed(const JValue& v) { return string_or_empty(v, "sense") == "reversed"; }

}  // namespace

Model load_cadir(std::string_view text) {
  JValue data = JValue::parse(text);
  Model model;
  if (const JValue* units = member(data, "units")) model.length_unit = string_or_empty(*units, "length");
  else model.length_unit.clear();
  const JValue& m = *member(data, "model");

  for (const auto& t : list(m, "tessellations")) {
    Tessellation table;
    table.id = string_or_empty(t, "id");
    table.body = opt_string(t, "body");
    table.faces = strings(member(t, "faces"));
    if (const JValue* so = member(t, "source_object"); so && so->is_object()) {
      SourceObject source;
      source.object_id = string_or_empty(*so, "object_id");
      source.name = opt_string(*so, "name");
      source.color = color(*so, "color");
      if (const JValue* vis = so->find("visible"); vis && vis->is_bool()) source.visible = vis->as_bool();
      table.source_object = source;
    }
    const JValue* mesh = member(t, "mesh");
    table.has_normals = mesh && string_or_empty(*mesh, "kind") == "shaded_strips";
    if (mesh)
      if (const JValue* strips = mesh->find("strips"); strips && strips->is_array())
        for (const auto& strip : strips->arr()) {
          std::vector<MeshVertex> vertices;
          for (const auto& vertex : strip.arr())
            vertices.push_back({vec3(vertex.find("position")), vec3(vertex.find("normal"))});
          table.strips.push_back(std::move(vertices));
        }
    model.tessellations.push_back(std::move(table));
  }
  for (const auto& a : list(m, "appearances"))
    model.appearances.push_back({string_or_empty(a, "id"), opt_string(a, "name"), opt_string(a, "schema"),
                                 color(a, "base_color")});
  for (const auto& b : list(m, "appearance_bindings")) {
    AppearanceBinding binding;
    binding.id = string_or_empty(b, "id");
    if (const JValue* target = member(b, "target")) {
      binding.target_kind = string_or_empty(*target, "kind");
      binding.target_id = string_or_empty(*target, "id");
    }
    binding.appearance = string_or_empty(b, "appearance");
    model.appearance_bindings.push_back(std::move(binding));
  }
  for (const auto& b : list(m, "bodies"))
    model.bodies.push_back({string_or_empty(b, "id"), string_or_empty(b, "kind"), strings(member(b, "regions")),
                            color(b, "color")});
  for (const auto& r : list(m, "regions"))
    model.regions.push_back({string_or_empty(r, "id"), string_or_empty(r, "body"), strings(member(r, "shells"))});
  for (const auto& s : list(m, "shells")) {
    Shell shell{string_or_empty(s, "id"), string_or_empty(s, "region"), {}};
    for (const auto& member_ref : list(s, "members"))
      if (string_or_empty(member_ref, "kind") == "face") shell.faces.push_back(string_or_empty(member_ref, "id"));
    model.shells.push_back(std::move(shell));
  }
  for (const auto& f : list(m, "faces")) {
    Face face;
    face.id = string_or_empty(f, "id");
    face.shell = string_or_empty(f, "shell");
    face.surface = opt_string(f, "surface");
    face.reversed = reversed(f);
    if (const JValue* loops = member(f, "loops")) face.loops = strings(member(*loops, "loops"));
    face.color = color(f, "color");
    model.faces.push_back(std::move(face));
  }
  for (const auto& l : list(m, "loops")) {
    Loop loop{string_or_empty(l, "id"), string_or_empty(l, "face"), {}};
    if (const JValue* boundary = member(l, "boundary")) loop.coedges = strings(member(*boundary, "coedges"));
    model.loops.push_back(std::move(loop));
  }
  for (const auto& c : list(m, "coedges"))
    model.coedges.push_back({string_or_empty(c, "id"), string_or_empty(c, "owner_loop"),
                             string_or_empty(c, "edge"), reversed(c)});
  for (const auto& e : list(m, "edges")) {
    Edge edge;
    edge.id = string_or_empty(e, "id");
    if (const JValue* carrier = member(e, "carrier")) {
      edge.curve = opt_string(*carrier, "curve");
      if (const JValue* range = carrier->find("param_range"); range && range->is_array() && range->arr().size() == 2)
        edge.param_range = std::array<double, 2>{number(range->arr()[0]), number(range->arr()[1])};
    }
    edge.start = opt_string(e, "start");
    edge.end = opt_string(e, "end");
    model.edges.push_back(std::move(edge));
  }
  for (const auto& v : list(m, "vertices"))
    model.vertices.push_back({string_or_empty(v, "id"), string_or_empty(v, "point")});
  for (const auto& p : list(m, "points"))
    model.points.push_back({string_or_empty(p, "id"), vec3(member(p, "position"))});
  for (const auto& s : list(m, "surfaces")) {
    Surface surface;
    surface.id = string_or_empty(s, "id");
    if (const JValue* g = member(s, "geometry")) {
      auto& geometry = surface.geometry;
      geometry.kind = string_or_empty(*g, "kind");
      geometry.origin = vec3(g->find(g->find("center") ? "center" : "origin"));
      geometry.normal = vec3(g->find(g->find("normal") ? "normal" : "axis"));
      geometry.u_axis = vec3(g->find(g->find("u_axis") ? "u_axis" : "ref_direction"));
      if (const JValue* r = g->find("radius")) geometry.radius = number(*r);
      if (const JValue* r = g->find("major_radius")) geometry.radius = number(*r);
      if (const JValue* r = g->find("minor_radius")) geometry.minor_radius = number(*r);
      if (const JValue* r = g->find("half_angle")) geometry.half_angle = number(*r);
    }
    model.surfaces.push_back(std::move(surface));
  }
  for (const auto& c : list(m, "curves")) {
    Curve curve;
    curve.id = string_or_empty(c, "id");
    if (const JValue* g = member(c, "geometry")) {
      auto& geometry = curve.geometry;
      geometry.kind = string_or_empty(*g, "kind");
      geometry.origin = vec3(g->find("origin"));
      geometry.direction = vec3(g->find("direction"));
      geometry.center = vec3(g->find("center"));
      geometry.axis = vec3(g->find("axis"));
      geometry.ref_direction = vec3(g->find(g->find("ref_direction") ? "ref_direction" : "major_direction"));
      if (const JValue* r = g->find("radius")) geometry.radius = number(*r);
      if (const JValue* r = g->find("major_radius")) geometry.radius = number(*r);
      if (const JValue* r = g->find("minor_radius")) geometry.minor_radius = number(*r);
    }
    model.curves.push_back(std::move(curve));
  }
  for (const auto& c : list(m, "configurations")) {
    Configuration configuration;
    configuration.id = string_or_empty(c, "id");
    if (const JValue* active = member(c, "active"); active && active->is_bool()) configuration.active = active->as_bool();
    configuration.bodies = strings(member(c, "bodies"));
    model.configurations.push_back(std::move(configuration));
  }
  for (const auto& o : list(m, "occurrences")) model.occurrences.push_back({string_or_empty(o, "id"), "", true});

  int modes = 0;
  for (const auto& a : list(m, "attributes")) {
    std::string name = string_or_empty(a, "name");
    const auto& values = list(a, "values");
    if (name == "saved_display_mode") {
      ++modes;
      model.saved_display_mode = values.at(0).find("value")->as_int();
    } else if (name == "saved_reference_plane") {
      ReferencePlane plane;
      auto value = [&](size_t i) -> const JValue& { return *values.at(i).find("value"); };
      plane.label = value(0).str();
      const auto& corners = value(1).arr();
      for (size_t i = 0; i + 2 < corners.size(); i += 3)
        plane.corners.push_back({number(corners[i]), number(corners[i + 1]), number(corners[i + 2])});
      const auto& origin = value(2).arr();
      plane.origin = {number(origin[0]), number(origin[1]), number(origin[2])};
      plane.font = value(3).str();
      plane.point_size = value(4).as_int();
      for (const auto& advance : value(5).arr()) plane.advances.push_back(number(advance));
      plane.scale = number(value(6));
      if (values.size() > 7) {
        const JValue& selector = value(7);
        if (selector.is_string() && selector.str() == "rgb") {
          plane.custom = true;
          if (values.size() > 8 && value(8).is_array() && value(8).arr().size() == 3) {
            const auto& rgb = value(8).arr();
            plane.custom_rgb = std::array<double, 3>{number(rgb[0]), number(rgb[1]), number(rgb[2])};
          }
        } else if (selector.is_int()) {
          plane.color_index = selector.as_int();
        }
      }
      model.reference_planes.push_back(std::move(plane));
    }
  }
  if (modes != 1) model.saved_display_mode.reset();
  return model;
}

}  // namespace swglb
