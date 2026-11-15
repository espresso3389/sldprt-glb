// SPDX-License-Identifier: Apache-2.0
// Unit tests for appearance.cc, ported from cadmpeg-codec-sldprt
// appearance/tests.rs (including the display-colour and
// appearance-ownership extension tests: appearance class reuse, face override,
// component override, colourless body boundary).
#undef NDEBUG  // the checks below are assert()s; keep them in Release builds
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <set>
#include <string>
#include <tuple>

#include "display_lists_internal.h"
#include "stages.h"

using namespace swglb;

namespace {

using Rgb = std::array<int, 3>;

void put32(Bytes& out, uint32_t v) {
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}
void put16(Bytes& out, uint16_t v) {
  out.push_back(static_cast<uint8_t>(v));
  out.push_back(static_cast<uint8_t>(v >> 8));
}
void putf(Bytes& out, float f) {
  uint32_t v;
  std::memcpy(&v, &f, 4);
  put32(out, v);
}
void append(Bytes& out, const Bytes& more) { out.insert(out.end(), more.begin(), more.end()); }

Bytes descriptor(uint32_t item_size, uint32_t kind, uint32_t count, const Bytes& data) {
  Bytes out;
  put32(out, item_size);
  put32(out, kind);
  put32(out, 2);
  put32(out, count);
  append(out, data);
  return out;
}

// appearance/tests.rs `display_table`.
Bytes display_table(float x) {
  Bytes positions;
  for (float f : {x, 0.0f, 0.0f, x + 1.0f, 0.0f, x, 0.0f, 1.0f, x}) putf(positions, f);
  Bytes strips;
  put32(strips, 3);
  Bytes list_c;
  put32(list_c, 4);
  Bytes out = descriptor(4, 8, 1, strips);
  append(out, descriptor(12, 100, 3, positions));
  append(out, descriptor(12, 100, 3, Bytes(36, 0)));
  append(out, descriptor(4, 8, 4, Bytes(16, 0)));
  append(out, descriptor(4, 8, 1, list_c));
  append(out, descriptor(1, 8, 4, Bytes(4, 0)));
  return out;
}

void display_class(Bytes& out, const std::string& name) {
  for (uint8_t b : {0xff, 0xff, 0x01, 0x00}) out.push_back(b);
  put16(out, static_cast<uint16_t>(name.size()));
  out.insert(out.end(), name.begin(), name.end());
}

void utf16_name(Bytes& out, const std::u16string& name) {
  for (uint8_t b : {0xff, 0xfe, 0xff}) out.push_back(b);
  out.push_back(static_cast<uint8_t>(name.size()));
  for (char16_t c : name) put16(out, c);
}
std::u16string u16(const std::string& s) { return std::u16string(s.begin(), s.end()); }

// test_support/appearance.rs `material_payload`.
Bytes material_payload(const std::string& name, Rgb rgb) {
  Bytes m;
  for (char c : std::string("moVisualProperties_c")) m.push_back(static_cast<uint8_t>(c));
  for (int c : rgb) m.push_back(static_cast<uint8_t>(c));
  m.push_back(0);
  put32(m, 0);
  put32(m, 0x00c0c0c0);
  for (uint8_t b : {0xff, 0xfe, 0xff, 0x00}) m.push_back(b);
  utf16_name(m, u16(name));
  return m;
}

Bytes inline_display_appearance(const std::string& name, Rgb rgb) {
  Bytes payload = material_payload(name, rgb);
  Bytes out = {0x33, 0x80};
  out.insert(out.end(), payload.begin() + 20, payload.end());
  return out;
}

// appearance/tests.rs `feature_visual_record`.
Bytes feature_visual_record(uint32_t source_id, uint32_t timestamp, Rgb rgb) {
  Bytes r(36, 0);
  auto set32 = [&](size_t at, uint32_t v) { std::memcpy(r.data() + at, &v, 4); };
  set32(4, source_id);
  set32(8, timestamp);
  set32(12, 1);
  set32(16, 1);
  set32(20, 2);
  const uint8_t prefix[6] = {0x07, 0x80, 0x01, 0x00, 0x00, 0x00};
  std::memcpy(r.data() + 24, prefix, 6);
  r[30] = 0x09;
  r[31] = 0x80;
  set32(32, static_cast<uint32_t>(rgb[0] | (rgb[1] << 8) | (rgb[2] << 16)));
  return r;
}

Bytes framed_surface_reference(const std::string& klass, uint32_t source, uint32_t local) {
  Bytes out;
  utf16_name(out, u16(klass + "," + std::to_string(source) + "," + std::to_string(local) +
                      ",opaque"));
  return out;
}

struct Ref {
  std::string klass;
  uint32_t source, local;
};

Container container_with(const Bytes& display, const Bytes* visual_states = nullptr) {
  Container c;
  Section s;
  s.name = "Contents/DisplayLists";
  s.source_stream = "Contents/DisplayLists";
  s.ordinal = 0x41;
  s.payload = display;
  c.sections.push_back(s);
  if (visual_states) {
    Section v;
    v.name = "ThirdPtyStore/VisualStates";
    v.source_stream = "ThirdPtyStore/VisualStates";
    v.ordinal = 0x42;
    v.payload = *visual_states;
    c.sections.push_back(v);
  }
  return c;
}

// appearance/tests.rs `display_fixture`.
Container display_fixture(Rgb body, const std::vector<std::vector<Ref>>& faces,
                          const std::vector<std::pair<size_t, Rgb>>& overrides,
                          const std::vector<std::tuple<uint32_t, uint32_t, Rgb>>& features,
                          Bytes* visual_states_out) {
  Bytes display;
  display_class(display, "uoTempFaceTessData_c");
  put32(display, 1);
  put32(display, 1);
  for (size_t index = 0; index < faces.size(); ++index) {
    append(display, display_table(static_cast<float>(index) * 10.0f));
    for (const auto& r : faces[index])
      append(display, framed_surface_reference(r.klass, r.source, r.local));
    for (const auto& [face, rgb] : overrides)
      if (face == index) {
        append(display, inline_display_appearance("Face override", rgb));
        break;
      }
  }
  display_class(display, "uoBodyPropInfo_c");
  append(display, inline_display_appearance("Body default", body));
  if (features.empty()) return container_with(display);
  Bytes& vs = *visual_states_out;
  vs.clear();
  display_class(vs, "moCompFeature_c");
  for (const auto& [source, timestamp, rgb] : features)
    append(vs, feature_visual_record(source, timestamp, rgb));
  return container_with(display, &vs);
}

Rgb rgb_of(const Color& c) {
  return {static_cast<int>(std::lround(c.r * 255)), static_cast<int>(std::lround(c.g * 255)),
          static_cast<int>(std::lround(c.b * 255))};
}

struct Bound {
  std::vector<std::pair<size_t, Rgb>> colors;  ///< bound table index -> colour
  Model model;
  std::set<uint32_t> matched;
};

// appearance/tests.rs `display_colors` over collect + resolve + ensure (the
// engine's push_display_meshes binding path).
Bound display_colors(const Container& c, const std::set<uint32_t>& eligible = {}) {
  Bound out;
  for (const auto& display : collect_display_sections(c)) {
    auto resolved = resolve_display_appearances(c, display, eligible);
    out.matched.insert(resolved.matched_feature_sources.begin(),
                       resolved.matched_feature_sources.end());
    for (const auto& [index, definition] : resolved.by_face) {
      std::string id = ensure_display_appearance(out.model, definition,
                                                 c.sections[display.section_index].ordinal);
      for (const auto& a : out.model.appearances)
        if (a.id == id) out.colors.emplace_back(index, rgb_of(*a.base_color));
    }
  }
  return out;
}

std::vector<Rgb> colors_only(const Bound& b) {
  std::vector<Rgb> out;
  for (const auto& [i, c] : b.colors) out.push_back(c);
  return out;
}

Bytes body_properties(Rgb rgb) {
  Bytes bytes;
  put16(bytes, 0);
  for (uint32_t v : {1u, 6u, 0u, 1u, 0u}) put32(bytes, v);
  for (uint8_t b : {0xff, 0xfe, 0xff, 0x00}) bytes.push_back(b);
  put32(bytes, 1);
  put32(bytes, UINT32_MAX);
  append(bytes, inline_display_appearance("Steel", rgb));
  return bytes;
}

// --- tests ------------------------------------------------------------------

void material_definitions_become_document_appearances() {
  Container c;
  Section s;
  s.source_stream = "block@8";
  s.ordinal = 8;
  s.payload = material_payload("Steel", {32, 64, 128});
  append(s.payload, material_payload("  ", {160, 170, 180}));
  c.sections.push_back(s);
  Model geometry;
  add_document_appearances(c, true, geometry);
  assert(geometry.appearances.size() == 2);
  const auto& a = geometry.appearances[0];
  assert(a.id == "sldprt:appearance:material#0" && a.name == "Steel" &&
         a.schema == "moVisualProperties_c");
  assert(*a.base_color == Color::from_rgba8(32, 64, 128));
  assert(geometry.appearances[1].id == "sldprt:appearance:material#1" &&
         geometry.appearances[1].name == "SolidWorks appearance");
  Model metadata;
  add_document_appearances(c, false, metadata);
  assert(metadata.appearances.empty());
}

void appearance_utf16_name_decodes() {
  Bytes payload = material_payload("x", {32, 64, 128});
  payload[payload.size() - 2] = 0x00;
  payload[payload.size() - 1] = 0x08;
  Container c;
  Section s;
  s.payload = payload;
  c.sections.push_back(s);
  Model m;
  add_document_appearances(c, true, m);
  assert(m.appearances.size() == 1 && m.appearances[0].name == "\xe0\xa0\x80");  // U+0800
}

void display_body_and_face_assignments_use_structural_precedence() {
  std::vector<std::vector<Ref>> faces(3);
  auto plain = display_colors(display_fixture({46, 255, 7}, faces, {}, {}, nullptr));
  assert(colors_only(plain) == (std::vector<Rgb>{{46, 255, 7}, {46, 255, 7}, {46, 255, 7}}));
  auto overridden = display_colors(display_fixture(
      {202, 209, 238}, faces, {{0, {247, 0, 23}}, {1, {7, 15, 255}}}, {}, nullptr));
  assert(colors_only(overridden) ==
         (std::vector<Rgb>{{247, 0, 23}, {7, 15, 255}, {202, 209, 238}}));
  // Every binding carries the definition's packed colour and one appearance
  // per distinct (name, colour).
  assert(overridden.model.appearances.size() == 3);
  assert(overridden.model.appearances[0].id.rfind("sldprt:appearance:displaylist#65:", 0) == 0);
}

void unresolved_history_and_display_state_do_not_bind_feature_appearances() {
  const char* classes[] = {"moFromSktEntSurfIdRep_c", "moFromSktEnt3IntSurfIdRep_c",
                           "moEndFace3IntSurfIdRep_c"};
  std::vector<std::vector<Ref>> faces;
  for (uint32_t index = 0; index < 12; ++index) {
    uint32_t source = index <= 5 ? 36 : index <= 9 ? 44 : 51;
    faces.push_back({{classes[index % 3], source, index + 1}});
  }
  Bytes vs;
  auto c = display_fixture({128, 128, 128}, faces, {},
                           {{36, 10, {236, 255, 0}}, {44, 20, {239, 0, 0}}, {51, 30, {7, 255, 43}}},
                           &vs);
  auto bound = display_colors(c);  // no eligible features
  assert(colors_only(bound) == std::vector<Rgb>(12, Rgb{128, 128, 128}));
  assert(bound.matched.empty());
}

void eligible_feature_appearance_colours_its_faces() {
  Bytes vs;
  auto c = display_fixture({128, 128, 128}, {{{"moFromSktEntSurfIdRep_c", 36, 1}}, {}}, {},
                           {{36, 10, {236, 255, 0}}}, &vs);
  auto bound = display_colors(c, {36});
  assert(colors_only(bound) == (std::vector<Rgb>{{236, 255, 0}, {128, 128, 128}}));
  assert(bound.matched == std::set<uint32_t>{36});
  bool found = false;
  for (const auto& a : bound.model.appearances)
    if (a.name == "SolidWorks feature appearance") {
      found = true;
      assert(a.id.rfind("sldprt:appearance:displaylist#65:", 0) == 0);
    }
  assert(found);
  // Ineligible: the body default stays.
  auto ineligible = display_colors(c, {99});
  assert(colors_only(ineligible) == std::vector<Rgb>(2, Rgb{128, 128, 128}));
}

void face_local_wins_and_conflicting_or_missing_feature_sources_do_not_guess() {
  std::vector<std::vector<Ref>> faces = {
      {{"moFromSktEntSurfIdRep_c", 36, 1}, {"moFromSktEntSurfIdRep_c", 36, 1}},
      {{"moFromSktEntSurfIdRep_c", 36, 2}, {"moFromSktEntSurfIdRep_c", 44, 2}},
      {{"moEndFace3IntSurfIdRep_c", 99, 1}},
  };
  Bytes vs;
  auto c = display_fixture({128, 128, 128}, faces, {{0, {0, 0, 255}}},
                           {{36, 10, {255, 0, 0}}, {44, 20, {0, 255, 0}}}, &vs);
  auto bound = display_colors(c, {36, 44});
  assert(colors_only(bound) ==
         (std::vector<Rgb>{{0, 0, 255}, {128, 128, 128}, {128, 128, 128}}));
}

void conflicting_feature_appearances_stay_unresolved() {
  Bytes display;
  display_class(display, "uoTempFaceTessData_c");
  put32(display, 1);
  put32(display, 1);
  append(display, display_table(0));
  append(display, framed_surface_reference("moFromSktEntSurfIdRep_c", 36, 1));
  display_class(display, "uoBodyPropInfo_c");
  append(display, inline_display_appearance("Body default", {128, 128, 128}));
  Bytes vs;
  display_class(vs, "moCompFeature_c");
  append(vs, feature_visual_record(36, 10, {255, 0, 0}));
  append(vs, feature_visual_record(36, 10, {0, 255, 0}));
  auto c = container_with(display, &vs);
  auto sections = collect_display_sections(c);
  auto resolved = resolve_display_appearances(c, sections[0], {36});
  assert(resolved.matched_feature_sources.empty());
  assert(resolved.by_face.at(0).name == "Body default");
  assert(resolved.by_face.at(0).packed_hex == "00808080");
}

void display_face_colors_continue_after_body_class_declaration() {
  Bytes display;
  display_class(display, "uoTempFaceTessData_c");
  put32(display, 1);
  put32(display, 1);
  append(display, display_table(0));
  display_class(display, "uoBodyPropInfo_c");
  append(display, inline_display_appearance("Body", {0, 255, 0}));
  append(display, display_table(10));
  append(display, inline_display_appearance("Face", {255, 0, 0}));
  append(display, display_table(20));
  append(display, inline_display_appearance("Face", {0, 0, 255}));
  auto bound = display_colors(container_with(display));
  assert(colors_only(bound) == (std::vector<Rgb>{{0, 255, 0}, {255, 0, 0}, {0, 0, 255}}));
}

// Appearance ownership: appearance class reuse.
void reused_body_properties_color_every_face_and_keep_face_overrides() {
  Bytes display;
  display_class(display, "uoTempFaceTessData_c");
  put32(display, 1);
  put32(display, 1);
  append(display, display_table(0));
  display_class(display, "uoBodyPropInfo_c");
  append(display, body_properties({147, 147, 147}));
  append(display, display_table(10));
  append(display, inline_display_appearance("Face", {255, 0, 0}));
  append(display, display_table(20));
  append(display, display_table(30));
  put16(display, 0x9234);
  append(display, body_properties({0, 192, 192}));
  auto bound = display_colors(container_with(display));
  assert(colors_only(bound) ==
         (std::vector<Rgb>{{147, 147, 147}, {255, 0, 0}, {0, 192, 192}, {0, 192, 192}}));
}

// Appearance ownership: component override and document default.
void component_appearance_overrides_part_faces_without_leaking_to_previous_body() {
  Bytes display;
  display_class(display, "moVisualProperties_c");
  Bytes doc = inline_display_appearance("Document default", {192, 192, 192});
  display.insert(display.end(), doc.begin() + 2, doc.end());
  display_class(display, "uoTempAssemblySHDData_c");
  display_class(display, "uoTempFaceTessData_c");
  put32(display, 1);
  put32(display, 1);
  append(display, display_table(-10));
  display_class(display, "uoBodyPropInfo_c");
  display.push_back(0);
  display.push_back(0);
  append(display, display_table(0));
  display_class(display, "uoBodyPropInfo_c");
  append(display, body_properties({147, 147, 147}));
  put16(display, 0x8020);
  utf16_name(display, u"bottom-1@assembly");
  display.push_back(1);
  append(display, inline_display_appearance("Component", {202, 209, 238}));
  append(display, display_table(10));
  append(display, inline_display_appearance("Face", {0, 0, 255}));
  append(display, display_table(20));
  append(display, inline_display_appearance("Face", {255, 255, 0}));
  put16(display, 0x9234);
  append(display, body_properties({128, 128, 128}));
  auto c = container_with(display);
  auto sections = collect_display_sections(c);
  assert(sections.size() == 1 && sections[0].faces.size() == 4);
  auto result = resolve_display_appearances(c, sections[0], {});
  std::vector<Rgb> expected = {{192, 192, 192}, {147, 147, 147}, {202, 209, 238}, {202, 209, 238}};
  for (size_t i = 0; i < expected.size(); ++i) {
    const Color& got = result.by_face.at(i).color;
    Color want = Color::from_rgba8(expected[i][0], expected[i][1], expected[i][2]);
    assert(std::abs(got.r - want.r) < 1e-6 && std::abs(got.g - want.g) < 1e-6 &&
           std::abs(got.b - want.b) < 1e-6);
  }
}

// Appearance ownership: a colourless body closes the previous group.
void body_without_local_appearance_closes_the_previous_group() {
  Bytes display;
  display_class(display, "uoTempFaceTessData_c");
  put32(display, 1);
  put32(display, 1);
  append(display, display_table(0));
  display_class(display, "uoBodyPropInfo_c");
  append(display, body_properties({147, 147, 147}));
  append(display, display_table(10));
  put16(display, 0x9234);
  Bytes empty = body_properties({0, 0, 0});
  empty.resize(empty.size() - inline_display_appearance("Steel", {0, 0, 0}).size() - 8);
  put16(empty, 0x803b);
  append(display, empty);
  append(display, display_table(20));
  append(display, display_table(30));
  put16(display, 0x9234);
  append(display, body_properties({0, 192, 192}));
  auto bound = display_colors(container_with(display));
  assert(colors_only(bound) ==
         (std::vector<Rgb>{{147, 147, 147}, {0, 192, 192}, {0, 192, 192}}));
}

void ensure_display_appearance_reuses_name_and_colour() {
  Model model;
  Appearance material;
  material.id = "sldprt:appearance:material#0";
  material.name = "Steel";
  material.schema = "moVisualProperties_c";
  material.base_color = Color::from_rgba8(128, 128, 0);
  model.appearances.push_back(material);
  DisplayAppearance same{Color::from_rgba8(128, 128, 0), "00008080", 19306, "Steel"};
  assert(ensure_display_appearance(model, same, 86564) == "sldprt:appearance:material#0");
  DisplayAppearance other{Color::from_rgba8(128, 128, 1), "01008080", 19306, "Steel"};
  assert(ensure_display_appearance(model, other, 86564) ==
         "sldprt:appearance:displaylist#86564:19306");
  assert(ensure_display_appearance(model, other, 1) == "sldprt:appearance:displaylist#86564:19306");
  assert(model.appearances.size() == 2 && model.appearances[1].schema == "moVisualProperties_c" &&
         model.appearances[1].name == "Steel");
}

}  // namespace

int main() {
  material_definitions_become_document_appearances();
  appearance_utf16_name_decodes();
  display_body_and_face_assignments_use_structural_precedence();
  unresolved_history_and_display_state_do_not_bind_feature_appearances();
  eligible_feature_appearance_colours_its_faces();
  face_local_wins_and_conflicting_or_missing_feature_sources_do_not_guess();
  conflicting_feature_appearances_stay_unresolved();
  display_face_colors_continue_after_body_class_declaration();
  reused_body_properties_color_every_face_and_keep_face_overrides();
  component_appearance_overrides_part_faces_without_leaking_to_previous_body();
  body_without_local_appearance_closes_the_previous_group();
  ensure_display_appearance_reuses_name_and_colour();
  std::cout << "appearance_test OK\n";
  return 0;
}
