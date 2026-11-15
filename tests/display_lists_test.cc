// SPDX-License-Identifier: Apache-2.0
// Unit tests for display_lists.cc, ported from cadmpeg-codec-sldprt
// tessellation/tests/{display_tables,display_references,mod}.rs.
#undef NDEBUG  // the checks below are assert()s; keep them in Release builds
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#include "display_lists_internal.h"
#include "errors.h"
#include "stages.h"

using namespace swglb;

namespace {

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

// test_support/tessellation.rs `descriptor`.
Bytes descriptor(uint32_t item_size, uint32_t kind, uint32_t count, const Bytes& data) {
  Bytes out;
  put32(out, item_size);
  put32(out, kind);
  put32(out, 2);
  put32(out, count);
  append(out, data);
  return out;
}

Bytes floats(std::initializer_list<float> values) {
  Bytes out;
  for (float f : values) putf(out, f);
  return out;
}

Bytes u32s(std::initializer_list<uint32_t> values) {
  Bytes out;
  for (uint32_t v : values) put32(out, v);
  return out;
}

// tessellation/tests/mod.rs `table`.
Bytes table(float x = 0) {
  Bytes out = descriptor(4, 8, 1, u32s({3}));
  append(out, descriptor(12, 100, 3, floats({x, 0, 0, x + 1, 0, 0, x, 1, 0})));
  append(out, descriptor(12, 100, 3, Bytes(36, 0)));
  append(out, descriptor(4, 8, 4, Bytes(16, 0)));
  append(out, descriptor(4, 8, 1, u32s({4})));
  append(out, descriptor(1, 8, 4, Bytes(4, 0)));
  return out;
}

// tessellation/tests/mod.rs `class` (without scene sources).
void klass(Bytes& payload, const std::string& name) {
  for (uint8_t b : {0xff, 0xff, 0x01, 0x00}) payload.push_back(b);
  put16(payload, static_cast<uint16_t>(name.size()));
  payload.insert(payload.end(), name.begin(), name.end());
}

Bytes framed(const std::u16string& text) {
  Bytes out = {0xff, 0xfe, 0xff, static_cast<uint8_t>(text.size())};
  for (char16_t c : text) put16(out, c);
  return out;
}
Bytes framed(const std::string& ascii) { return framed(std::u16string(ascii.begin(), ascii.end())); }

std::u32string u32(const std::string& s) { return std::u32string(s.begin(), s.end()); }

std::vector<SurfaceReferencePtr> refs_of(const Bytes& payload) {
  return persistent_surface_references(ByteSpan(payload), ByteRange{0, payload.size()});
}

PersistentFaceIdentity identity(uint32_t source, uint32_t local, std::vector<uint32_t> tail = {}) {
  return PersistentFaceIdentity{source, local, std::move(tail)};
}

SurfaceReferencePtr complete(uint32_t source, uint32_t local, std::vector<uint32_t> tail = {}) {
  auto r = std::make_shared<DisplayFace::SurfaceReference>();
  r->complete = true;
  r->identity = identity(source, local, std::move(tail));
  return r;
}
SurfaceReferencePtr source_only(uint32_t source, uint32_t local) {
  auto r = std::make_shared<DisplayFace::SurfaceReference>();
  r->identity = identity(source, local);
  return r;
}

template <class F>
std::string error_of(F f) {
  try {
    f();
  } catch (const DecodeError& e) {
    return e.what();
  }
  return {};
}

Bytes face_payload(uint32_t triangles = 1, uint32_t strips = 1) {
  Bytes p;
  klass(p, "uoTempFaceTessData_c");
  put32(p, triangles);
  put32(p, strips);
  append(p, table());
  return p;
}

// --- display_tables.rs --------------------------------------------------

void compact_and_extended_headers() {
  Bytes compact = u32s({1, 1});
  append(compact, table());
  assert(descriptor_table_offset(compact, 0) == 8);
  assert(parse_table(compact, 8));

  Bytes extended = u32s({1, 1, 1, 0, 0, 0x00201296, 0, 0, 0, 0});
  append(extended, table());
  assert(descriptor_table_offset(extended, 0) == 40);
  assert(parse_table(extended, 40));

  Bytes incomplete = u32s({1, 1, 1, 0, 0, 0, 0, 0, 0, 0});
  append(incomplete, table());
  assert(descriptor_table_offset(incomplete, 0) == 8);
}

void body_property_class_does_not_end_face_table_sequence() {
  Bytes p = face_payload();
  klass(p, "uoBodyPropInfo_c");
  for (uint8_t b : {0x37, 0x80}) p.push_back(b);
  append(p, u32s({1, 1}));
  append(p, table());
  assert(section_display_faces(p).size() == 2);
}

void next_face_class_ends_face_table_sequence() {
  Bytes p = face_payload();
  append(p, face_payload());
  auto faces = section_display_faces(p);
  assert(faces.size() == 2);
  // The sequence limit is the next face class name, before its 8-byte header.
  assert(faces[0].metadata.end == faces[1].table.start - 8 - 20);
}

void inconsistent_auxiliary_count_invalidates_the_table() {
  Bytes p = table();
  size_t list_b_count = 20 + 52 + 52 + 12;
  std::memcpy(p.data() + list_b_count, u32s({3}).data(), 4);
  assert(!parse_table(p, 0));
  Bytes q = table();
  size_t list_c_value = 20 + 52 + 52 + 32 + 16;
  std::memcpy(q.data() + list_c_value, u32s({5}).data(), 4);
  assert(!parse_table(q, 0));
}

void a_strip_span_past_the_vertex_lane_refuses_the_recognised_table() {
  Bytes p = descriptor(4, 8, 1, u32s({4}));
  append(p, descriptor(12, 100, 3, floats({0, 0, 0, 1, 0, 0, 0, 1, 0})));
  append(p, descriptor(12, 100, 3, Bytes(36, 0)));
  append(p, descriptor(4, 8, 6, Bytes(24, 0)));
  append(p, descriptor(4, 8, 1, u32s({6})));
  append(p, descriptor(1, 8, 6, Bytes(6, 0)));
  std::string text = error_of([&] { parse_table(p, 0); });
  assert(text.find("sldprt display-list table at byte 0") != std::string::npos);
  assert(text.find("strip span(s) do not cut the vertex lane") != std::string::npos);
}

void a_short_normal_lane_refuses_the_display_table() {
  Bytes p = descriptor(4, 8, 1, u32s({3}));
  append(p, descriptor(12, 100, 3, floats({0, 0, 0, 1, 0, 0, 0, 1, 0})));
  append(p, descriptor(12, 100, 2, Bytes(24, 0)));
  append(p, descriptor(4, 8, 4, Bytes(16, 0)));
  append(p, descriptor(4, 8, 1, u32s({4})));
  append(p, descriptor(1, 8, 4, Bytes(4, 0)));
  assert(error_of([&] { parse_table(p, 0); }).find("vertex normal(s)") != std::string::npos);
}

void display_table_reads_as_a_mesh() {
  Bytes p = table();
  auto t = parse_table(p, 0);
  assert(t && t->end == p.size() && t->has_normals);
  assert(t->strips.size() == 1 && t->strips[0].size() == 3);
  assert(t->strips[0][1].position.x == 1000.0);
}

void empty_normal_lane_is_unshaded() {
  Bytes p = descriptor(4, 8, 1, u32s({3}));
  append(p, descriptor(12, 100, 3, floats({0, 0, 0, 1, 0, 0, 0, 1, 0})));
  append(p, descriptor(12, 100, 0, {}));
  append(p, descriptor(4, 8, 4, Bytes(16, 0)));
  append(p, descriptor(4, 8, 1, u32s({4})));
  append(p, descriptor(1, 8, 4, Bytes(4, 0)));
  auto t = parse_table(p, 0);
  assert(t && !t->has_normals);
}

void rejected_candidate_tables_are_skipped() {
  const size_t candidates = 64, span = 64 * 1024;
  Bytes p;
  klass(p, "uoTempFaceTessData_c");
  append(p, u32s({1, 1}));
  size_t base = p.size();
  append(p, table());
  size_t candidates_at = p.size();
  size_t terminator_at = candidates_at + candidates * 16 + span;
  for (size_t i = 0; i < candidates; ++i) {
    size_t data = candidates_at + i * 16 + 16;
    append(p, descriptor(4, 8, static_cast<uint32_t>((terminator_at - data) / 4), {}));
  }
  p.resize(terminator_at, 0);
  append(p, descriptor(4, 9, 0, {}));
  append(p, table());
  auto faces = section_display_faces(p);
  assert(faces.size() == 2);
  assert(faces[0].table.start == base);
}

void a_zero_face_header_states_no_display_face() {
  assert(section_display_faces(face_payload(0, 0)).empty());
}

void a_declared_face_count_disagreement_refuses_the_display_face() {
  Bytes p = face_payload(2, 1);
  std::string text = error_of([&] { section_display_faces(p); });
  assert(text.find("sldprt display-face table at byte") != std::string::npos);
  assert(text.find("header states 2 triangle(s) and 1 strip(s)") != std::string::npos);
  assert(text.find("parsed mesh has 1 triangle(s) and 1 strip(s)") != std::string::npos);
}

void inconsistent_or_nonfinite_tables_are_probe_misses() {
  // mod.rs decode_rejects_inconsistent_display_list_table
  Bytes p = face_payload();
  size_t at = 6 + 20 + 8 + 16;
  std::memcpy(p.data() + at, u32s({4}).data(), 4);
  assert(section_display_faces(p).empty());
  // mod.rs decode_rejects_nonfinite_display_list_values
  Bytes q = face_payload();
  size_t position = 6 + 20 + 8 + 16 + 4 + 16;
  float nan = std::nanf("");
  std::memcpy(q.data() + position, &nan, 4);
  assert(section_display_faces(q).empty());
}

Bytes reused_tag_block(uint16_t tag, uint32_t triangles, uint32_t strips) {
  Bytes b;
  put16(b, tag);
  put32(b, triangles);
  put32(b, strips);
  return b;
}

Bytes two_face_payload(const Bytes& gap, const Bytes& second_metadata) {
  Bytes p = face_payload();
  append(p, gap);
  append(p, face_payload());
  append(p, second_metadata);
  return p;
}

void reused_tags() {
  {
    Bytes p = two_face_payload(Bytes(24, 0), {});
    auto faces = section_display_faces(p);
    assert(faces.size() == 2);
    auto classes = class_intervals(p);
    assert(admit_reused_face_tag(p, faces[1], faces[0].table.end, classes).kind ==
           ReusedTagVerdict::Absent);
    bool ambiguous = true;
    assert(!persistent_join(p, faces, 1, &ambiguous) && !ambiguous);  // InvalidTokens
  }
  {
    Bytes p = two_face_payload(reused_tag_block(0x8001, 1, 1), framed("moPlaneSurfIdRep_c,7,3,"));
    auto faces = section_display_faces(p);
    assert(faces.size() == 2);
    auto classes = class_intervals(p);
    auto verdict = admit_reused_face_tag(p, faces[1], faces[0].table.end, classes);
    assert(verdict.kind == ReusedTagVerdict::Admit && verdict.tag == 0x8001);
    bool ambiguous = true;
    auto joined = persistent_join(p, faces, 1, &ambiguous);
    assert(joined && *joined == identity(7, 3) && !ambiguous);
    assert(faces[1].surface_reference_features == std::vector<uint32_t>{7});
  }
  {
    Bytes gap = reused_tag_block(0x8001, 1, 1);
    append(gap, reused_tag_block(0x8002, 1, 1));
    Bytes p = two_face_payload(gap, framed("moPlaneSurfIdRep_c,7,3,"));
    auto faces = section_display_faces(p);
    auto classes = class_intervals(p);
    assert(admit_reused_face_tag(p, faces[1], faces[0].table.end, classes).kind ==
           ReusedTagVerdict::Withhold);
    bool ambiguous = false;
    assert(!persistent_join(p, faces, 1, &ambiguous) && ambiguous);
  }
  {
    Bytes gap = reused_tag_block(0xffff, 1, 1);
    append(gap, reused_tag_block(0x8000, 1, 1));
    Bytes p = two_face_payload(gap, {});
    auto faces = section_display_faces(p);
    assert(admit_reused_face_tag(p, faces[1], faces[0].table.end, class_intervals(p)).kind ==
           ReusedTagVerdict::Absent);
  }
  {
    Bytes p = two_face_payload(reused_tag_block(0x8001, 2, 1), {});
    auto faces = section_display_faces(p);
    assert(admit_reused_face_tag(p, faces[1], faces[0].table.end, class_intervals(p)).kind ==
           ReusedTagVerdict::Absent);
  }
  {
    Bytes p = two_face_payload(reused_tag_block(0x8001, 1, 1), {});
    auto faces = section_display_faces(p);
    assert(admit_reused_face_tag(p, faces[1], faces[0].table.end, {}).kind ==
           ReusedTagVerdict::Absent);
  }
  {
    Bytes p = two_face_payload({}, {});
    assert(!persistent_join(p, {}, 0, nullptr));
  }
}

// --- display_references.rs ----------------------------------------------

void overlapping_tables_narrow_to_an_empty_metadata_range() {
  Bytes payload(192, 0);
  Bytes reference = framed("moPlaneSurfIdRep_c,7,3,");
  std::memcpy(payload.data() + 64, reference.data(), reference.size());
  assert(persistent_surface_references(payload, ByteRange{64, 64}).empty());
  assert(persistent_surface_references(payload, ByteRange{64, 120}).size() ==
         persistent_surface_references(payload, ByteRange{64, 128}).size());
  assert(persistent_surface_references(payload, ByteRange{64, 128}).size() == 1);
}

void surface_reference_parsing() {
  auto signed_tail = refs_of(framed("moContent3IntSurfIdRep_c,300,4,-1,0,"));
  assert(signed_tail.size() == 1 && signed_tail[0]->complete &&
         signed_tail[0]->identity == identity(300, 4, {UINT32_MAX, 0}));

  auto opaque = refs_of(framed("moFromSktEntSurfIdRep_c,7,3,opaque"));
  assert(opaque.size() == 1 && !opaque[0]->complete && opaque[0]->identity == identity(7, 3));
  DisplayFace face;
  face.surface_references = opaque;
  assert(face_feature_source_id(face) == 7u);
  SurfaceTokenWithhold reason;
  assert(!agree_surface_tokens(opaque, &reason) && reason == SurfaceTokenWithhold::OpaqueTail);

  for (uint32_t source : {0u, UINT32_MAX})
    assert(refs_of(framed("moPlaneSurfIdRep_c," + std::to_string(source) + ",3,")).empty());

  for (std::string text :
       {"moPlaneSurf_c,7,3,", "moPlaneSurfIdRep_c,seven,3,", "moPlaneSurfIdRep_c,7,three,",
        "moPlaneSurfIdRep_c,0,3,", "moPlaneSurfIdRep_c,4294967295,3,"}) {
    SurfaceTokenStatus status = SurfaceTokenStatus::Complete;
    assert(!classify_surface_token(u32(text), &status) && status == SurfaceTokenStatus::Invalid);
    assert(refs_of(framed(text)).empty());
  }
  // Surrounding whitespace is trimmed; a whitespace-only last field is dropped.
  auto trimmed = classify_surface_token(U"  moPlaneSurfIdRep_c,+7,3,-2, 　");
  assert(trimmed && trimmed->complete && trimmed->identity == identity(7, 3, {UINT32_MAX - 1}));
  // An unpaired surrogate is skipped, not fatal.
  std::u16string bad = u"moPlaneSurfIdRep_c,7,3,";
  bad.push_back(static_cast<char16_t>(0xd800));
  assert(refs_of(framed(bad)).empty());
}

void agreement() {
  SurfaceTokenWithhold reason;
  auto agreed = agree_surface_tokens({complete(7, 3), complete(7, 3)}, &reason);
  assert(agreed && *agreed == identity(7, 3) && reason == SurfaceTokenWithhold::None);
  assert(!agree_surface_tokens({complete(7, 3, {1}), complete(7, 3, {2})}, &reason) &&
         reason == SurfaceTokenWithhold::Conflicting);
  assert(!agree_surface_tokens({complete(7, 3), complete(7, 4)}, &reason) &&
         reason == SurfaceTokenWithhold::Conflicting);
  assert(!agree_surface_tokens({source_only(7, 3)}, &reason) &&
         reason == SurfaceTokenWithhold::OpaqueTail);
  assert(!agree_surface_tokens({complete(7, 3), source_only(7, 3)}, &reason) &&
         reason == SurfaceTokenWithhold::OpaqueTail);
  assert(!agree_surface_tokens({}, &reason) && reason == SurfaceTokenWithhold::Invalid);

  DisplayFace face;
  face.surface_references = {complete(7, 3), complete(7, 4)};
  assert(face_feature_source_id(face) == 7u);
  face.surface_references = {complete(7, 3), complete(8, 3)};
  assert(!face_feature_source_id(face));
}

// --- mod.rs display_face_key ----------------------------------------------

void display_face_key_reads_the_one_cell_and_the_key() {
  auto key_table = [](uint32_t one, uint32_t key) {
    Bytes p(0x100, 0);
    std::memcpy(p.data() + 0x60, &one, 4);
    std::memcpy(p.data() + 0x64, &key, 4);
    return p;
  };
  auto face = [](size_t start, size_t end) {
    DisplayFace f;
    f.table = ByteRange{0, start};
    f.metadata = ByteRange{start, end};
    return f;
  };
  std::vector<DisplayFace> alone = {face(0, 0x100)};
  assert(display_face_key(key_table(1, 0x417300a6), alone, 0) == 0x417300a6u);
  assert(!display_face_key(key_table(1, 0), alone, 0));
  assert(!display_face_key(key_table(2, 0x417300a6), alone, 0));
  DisplayFace next = face(0x80, 0x100);
  next.table = ByteRange{0x66, 0x80};
  assert(!display_face_key(key_table(1, 0x417300a6), {face(0, 0x100), next}, 0));
  assert(!display_face_key(key_table(1, 0x417300a6), alone, 1));
}

void collect_skips_sections_without_faces() {
  Container c;
  Section empty;
  empty.name = "Contents/Config-0";
  empty.payload = Bytes(64, 0);
  Section display;
  display.name = "Contents/DisplayLists";
  display.ordinal = 0x41;
  display.payload = face_payload();
  c.sections = {empty, display};
  auto sections = collect_display_sections(c);
  assert(sections.size() == 1 && sections[0].section_index == 1 && sections[0].faces.size() == 1);
}

}  // namespace

int main() {
  compact_and_extended_headers();
  body_property_class_does_not_end_face_table_sequence();
  next_face_class_ends_face_table_sequence();
  inconsistent_auxiliary_count_invalidates_the_table();
  a_strip_span_past_the_vertex_lane_refuses_the_recognised_table();
  a_short_normal_lane_refuses_the_display_table();
  display_table_reads_as_a_mesh();
  empty_normal_lane_is_unshaded();
  rejected_candidate_tables_are_skipped();
  a_zero_face_header_states_no_display_face();
  a_declared_face_count_disagreement_refuses_the_display_face();
  inconsistent_or_nonfinite_tables_are_probe_misses();
  reused_tags();
  overlapping_tables_narrow_to_an_empty_metadata_range();
  surface_reference_parsing();
  agreement();
  display_face_key_reads_the_one_cell_and_the_key();
  collect_skips_sections_without_faces();
  std::cout << "display_lists_test OK\n";
  return 0;
}
