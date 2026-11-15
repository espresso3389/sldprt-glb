// SPDX-License-Identifier: Apache-2.0
#include "model_json.h"

#include <charconv>
#include <cmath>
#include <string>

namespace swglb {
namespace {

class Json {
 public:
  explicit Json(std::ostream& out) : out_(out) {}

  void str(std::string_view s) {
    out_ << '"';
    for (unsigned char c : s) {
      switch (c) {
        case '"': out_ << "\\\""; break;
        case '\\': out_ << "\\\\"; break;
        case '\n': out_ << "\\n"; break;
        case '\r': out_ << "\\r"; break;
        case '\t': out_ << "\\t"; break;
        default:
          if (c < 0x20) {
            char buf[8];
            std::snprintf(buf, sizeof buf, "\\u%04x", c);
            out_ << buf;
          } else {
            out_ << c;
          }
      }
    }
    out_ << '"';
  }
  void num(double v) {
    if (!std::isfinite(v)) {
      out_ << "null";
      return;
    }
    char buf[64];
    auto [p, ec] = std::to_chars(buf, buf + sizeof buf, v);
    std::string_view s(buf, static_cast<size_t>(p - buf));
    out_ << s;
    if (s.find_first_of(".eEn") == std::string_view::npos) out_ << ".0";
  }
  void num(float v) {
    char buf[64];
    auto [p, ec] = std::to_chars(buf, buf + sizeof buf, v);
    std::string_view s(buf, static_cast<size_t>(p - buf));
    out_ << s;
    if (s.find_first_of(".eEn") == std::string_view::npos) out_ << ".0";
  }
  void integer(int64_t v) { out_ << v; }
  void boolean(bool v) { out_ << (v ? "true" : "false"); }
  void raw(std::string_view s) { out_ << s; }

  void begin_obj() { out_ << '{'; first_ = true; }
  void end_obj() { out_ << '}'; first_ = false; }
  void begin_arr() { out_ << '['; first_ = true; }
  void end_arr() { out_ << ']'; first_ = false; }
  void key(std::string_view k) {
    sep();
    str(k);
    out_ << ':';
    first_ = false;
  }
  void item() { sep(); }

 private:
  void sep() {
    if (!first_) out_ << ',';
    first_ = false;
  }
  std::ostream& out_;
  bool first_ = true;
};

void vec(Json& j, const Vec3& v) {
  j.begin_obj();
  j.key("x"); j.num(v.x);
  j.key("y"); j.num(v.y);
  j.key("z"); j.num(v.z);
  j.end_obj();
}

void color(Json& j, const Color& c) {
  j.begin_obj();
  j.key("r"); j.num(c.r);
  j.key("g"); j.num(c.g);
  j.key("b"); j.num(c.b);
  j.key("a"); j.num(c.a);
  j.end_obj();
}

template <class T, class F>
void array(Json& j, std::string_view name, const std::vector<T>& items, F each) {
  j.key(name);
  j.begin_arr();
  for (const auto& it : items) {
    j.item();
    each(it);
  }
  j.end_arr();
}

void strings(Json& j, std::string_view name, const std::vector<std::string>& items) {
  array(j, name, items, [&](const std::string& s) { j.str(s); });
}

}  // namespace

void write_model_json(std::ostream& out, const Model& model, bool include_mesh) {
  Json j(out);
  j.begin_obj();
  j.key("units");
  j.begin_obj();
  j.key("length");
  j.str(model.length_unit);
  j.end_obj();
  j.key("model");
  j.begin_obj();

  array(j, "bodies", model.bodies, [&](const Body& b) {
    j.begin_obj();
    j.key("id"); j.str(b.id);
    j.key("kind"); j.str(b.kind);
    strings(j, "regions", b.regions);
    if (b.color) { j.key("color"); color(j, *b.color); }
    j.end_obj();
  });
  array(j, "regions", model.regions, [&](const Region& r) {
    j.begin_obj();
    j.key("id"); j.str(r.id);
    j.key("body"); j.str(r.body);
    strings(j, "shells", r.shells);
    j.end_obj();
  });
  array(j, "shells", model.shells, [&](const Shell& s) {
    j.begin_obj();
    j.key("id"); j.str(s.id);
    j.key("region"); j.str(s.region);
    j.key("members");
    j.begin_arr();
    for (const auto& f : s.faces) {
      j.item();
      j.begin_obj();
      j.key("kind"); j.str("face");
      j.key("id"); j.str(f);
      j.end_obj();
    }
    j.end_arr();
    j.end_obj();
  });
  array(j, "faces", model.faces, [&](const Face& f) {
    j.begin_obj();
    j.key("id"); j.str(f.id);
    j.key("shell"); j.str(f.shell);
    if (f.surface) { j.key("surface"); j.str(*f.surface); }
    j.key("sense"); j.str(f.reversed ? "reversed" : "forward");
    j.key("loops");
    j.begin_obj();
    strings(j, "loops", f.loops);
    j.end_obj();
    if (f.color) { j.key("color"); color(j, *f.color); }
    j.end_obj();
  });
  array(j, "loops", model.loops, [&](const Loop& l) {
    j.begin_obj();
    j.key("id"); j.str(l.id);
    j.key("face"); j.str(l.face);
    j.key("boundary");
    j.begin_obj();
    strings(j, "coedges", l.coedges);
    j.end_obj();
    j.end_obj();
  });
  array(j, "coedges", model.coedges, [&](const Coedge& c) {
    j.begin_obj();
    j.key("id"); j.str(c.id);
    j.key("owner_loop"); j.str(c.owner_loop);
    j.key("edge"); j.str(c.edge);
    j.key("sense"); j.str(c.reversed ? "reversed" : "forward");
    j.end_obj();
  });
  array(j, "edges", model.edges, [&](const Edge& e) {
    j.begin_obj();
    j.key("id"); j.str(e.id);
    j.key("carrier");
    j.begin_obj();
    if (e.curve) { j.key("curve"); j.str(*e.curve); }
    if (e.param_range) {
      j.key("param_range");
      j.begin_arr();
      j.item(); j.num((*e.param_range)[0]);
      j.item(); j.num((*e.param_range)[1]);
      j.end_arr();
    }
    j.end_obj();
    if (e.start) { j.key("start"); j.str(*e.start); }
    if (e.end) { j.key("end"); j.str(*e.end); }
    j.end_obj();
  });
  array(j, "vertices", model.vertices, [&](const Vertex& v) {
    j.begin_obj();
    j.key("id"); j.str(v.id);
    j.key("point"); j.str(v.point);
    j.end_obj();
  });
  array(j, "points", model.points, [&](const Point& p) {
    j.begin_obj();
    j.key("id"); j.str(p.id);
    j.key("position"); vec(j, p.position);
    j.end_obj();
  });
  array(j, "surfaces", model.surfaces, [&](const Surface& s) {
    j.begin_obj();
    j.key("id"); j.str(s.id);
    j.key("geometry");
    j.begin_obj();
    const auto& g = s.geometry;
    j.key("kind"); j.str(g.kind);
    if (g.kind == "plane") {
      j.key("origin"); vec(j, g.origin);
      j.key("normal"); vec(j, g.normal);
      j.key("u_axis"); vec(j, g.u_axis);
    } else if (g.kind == "cylinder" || g.kind == "cone") {
      j.key("origin"); vec(j, g.origin);
      j.key("axis"); vec(j, g.normal);
      j.key("ref_direction"); vec(j, g.u_axis);
      j.key("radius"); j.num(g.radius);
      if (g.kind == "cone") { j.key("half_angle"); j.num(g.half_angle); }
    } else if (g.kind == "sphere" || g.kind == "torus") {
      j.key("center"); vec(j, g.origin);
      j.key("axis"); vec(j, g.normal);
      j.key("ref_direction"); vec(j, g.u_axis);
      if (g.kind == "torus") {
        j.key("major_radius"); j.num(g.radius);
        j.key("minor_radius"); j.num(g.minor_radius);
      } else {
        j.key("radius"); j.num(g.radius);
      }
    }
    j.end_obj();
    j.end_obj();
  });
  array(j, "curves", model.curves, [&](const Curve& c) {
    j.begin_obj();
    j.key("id"); j.str(c.id);
    j.key("geometry");
    j.begin_obj();
    const auto& g = c.geometry;
    j.key("kind"); j.str(g.kind);
    if (g.kind == "line") {
      j.key("origin"); vec(j, g.origin);
      j.key("direction"); vec(j, g.direction);
    } else if (g.kind == "circle") {
      j.key("center"); vec(j, g.center);
      j.key("axis"); vec(j, g.axis);
      j.key("ref_direction"); vec(j, g.ref_direction);
      j.key("radius"); j.num(g.radius);
    } else if (g.kind == "ellipse") {
      j.key("center"); vec(j, g.center);
      j.key("axis"); vec(j, g.axis);
      j.key("major_direction"); vec(j, g.ref_direction);
      j.key("major_radius"); j.num(g.radius);
      j.key("minor_radius"); j.num(g.minor_radius);
    }
    j.end_obj();
    j.end_obj();
  });
  array(j, "configurations", model.configurations, [&](const Configuration& c) {
    j.begin_obj();
    j.key("id"); j.str(c.id);
    j.key("ordinal"); j.integer(static_cast<int64_t>(c.ordinal));
    if (c.active) { j.key("active"); j.boolean(true); }
    if (c.name) { j.key("name"); j.str(*c.name); }
    strings(j, "bodies", c.bodies);
    j.end_obj();
  });
  array(j, "occurrences", model.occurrences, [&](const Occurrence& o) {
    j.begin_obj();
    j.key("id"); j.str(o.id);
    j.key("path"); j.str(o.path);
    j.key("visible"); j.boolean(o.visible);
    j.end_obj();
  });
  array(j, "tessellations", model.tessellations, [&](const Tessellation& t) {
    j.begin_obj();
    j.key("id"); j.str(t.id);
    if (t.body) { j.key("body"); j.str(*t.body); }
    if (!t.faces.empty()) strings(j, "faces", t.faces);
    if (t.source_object) {
      j.key("source_object");
      j.begin_obj();
      j.key("object_id"); j.str(t.source_object->object_id);
      if (t.source_object->name) { j.key("name"); j.str(*t.source_object->name); }
      if (t.source_object->color) { j.key("color"); color(j, *t.source_object->color); }
      if (t.source_object->visible) { j.key("visible"); j.boolean(*t.source_object->visible); }
      j.end_obj();
    }
    j.key("mesh");
    j.begin_obj();
    j.key("kind"); j.str(t.has_normals ? "shaded_strips" : "strips");
    if (include_mesh) {
      j.key("strips");
      j.begin_arr();
      for (const auto& strip : t.strips) {
        j.item();
        j.begin_arr();
        for (const auto& v : strip) {
          j.item();
          j.begin_obj();
          j.key("position"); vec(j, v.position);
          if (t.has_normals) { j.key("normal"); vec(j, v.normal); }
          j.end_obj();
        }
        j.end_arr();
      }
      j.end_arr();
    }
    j.end_obj();
    j.end_obj();
  });
  array(j, "appearances", model.appearances, [&](const Appearance& a) {
    j.begin_obj();
    j.key("id"); j.str(a.id);
    if (a.name) { j.key("name"); j.str(*a.name); }
    if (a.schema) { j.key("schema"); j.str(*a.schema); }
    if (a.base_color) { j.key("base_color"); color(j, *a.base_color); }
    j.end_obj();
  });
  array(j, "appearance_bindings", model.appearance_bindings, [&](const AppearanceBinding& b) {
    j.begin_obj();
    j.key("id"); j.str(b.id);
    j.key("target");
    j.begin_obj();
    j.key("kind"); j.str(b.target_kind);
    j.key("id");
    if (b.target_kind == "source") {  // CADIR AppearanceTarget::Source { source_id }
      j.begin_obj();
      j.key("source_id"); j.str(b.target_id);
      j.end_obj();
    } else {
      j.str(b.target_id);
    }
    j.end_obj();
    j.key("appearance"); j.str(b.appearance);
    if (b.source_entity_id) { j.key("source_entity_id"); j.str(*b.source_entity_id); }
    if (b.object_type) { j.key("object_type"); j.str(*b.object_type); }
    j.key("channels");
    j.begin_obj();
    for (const auto& [k, v] : b.channels) { j.key(k); j.str(v); }
    j.end_obj();
    j.end_obj();
  });

  // Presentation in CADIR attribute form.
  j.key("attributes");
  j.begin_arr();
  if (model.saved_display_mode) {
    j.item();
    j.begin_obj();
    j.key("name"); j.str("saved_display_mode");
    j.key("values");
    j.begin_arr();
    j.item();
    j.begin_obj();
    j.key("kind"); j.str("integer");
    j.key("value"); j.integer(*model.saved_display_mode);
    j.end_obj();
    j.end_arr();
    j.end_obj();
  }
  for (const auto& p : model.reference_planes) {
    j.item();
    j.begin_obj();
    j.key("name"); j.str("saved_reference_plane");
    j.key("values");
    j.begin_arr();
    auto typed = [&](std::string_view kind, auto write) {
      j.item();
      j.begin_obj();
      j.key("kind"); j.str(kind);
      j.key("value");
      write();
      j.end_obj();
    };
    auto flat = [&](const std::vector<double>& values) {
      j.begin_arr();
      for (double v : values) { j.item(); j.num(v); }
      j.end_arr();
    };
    typed("string", [&] { j.str(p.label); });
    typed("vector", [&] {
      std::vector<double> c;
      for (const auto& v : p.corners) { c.push_back(v.x); c.push_back(v.y); c.push_back(v.z); }
      flat(c);
    });
    typed("vector", [&] { flat({p.origin.x, p.origin.y, p.origin.z}); });
    typed("string", [&] { j.str(p.font); });
    typed("integer", [&] { j.integer(p.point_size); });
    typed("vector", [&] { flat(p.advances); });
    typed("float", [&] { j.num(p.scale); });
    if (p.custom) {
      typed("string", [&] { j.str("rgb"); });
      if (p.custom_rgb) {
        typed("vector", [&] { flat({(*p.custom_rgb)[0], (*p.custom_rgb)[1], (*p.custom_rgb)[2]}); });
      }
    } else if (p.color_index) {
      typed("integer", [&] { j.integer(*p.color_index); });
    }
    j.end_arr();
    j.end_obj();
  }
  j.end_arr();

  j.end_obj();
  j.end_obj();
  out << '\n';
}

}  // namespace swglb
