// SPDX-License-Identifier: Apache-2.0
// Ports of cadmpeg metadata/presentation/tests.rs (with the
// scene-mode and reference-plane pen extensions).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "container.h"
#include "errors.h"
#include "presentation_internal.h"

using namespace swglb;
using namespace swglb::presentation;

namespace {

#define CHECK(cond)                                                      \
  do {                                                                   \
    if (!(cond)) {                                                       \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      std::abort();                                                      \
    }                                                                    \
  } while (0)

void u16le(Bytes& b, uint16_t v) {
  b.push_back(static_cast<uint8_t>(v));
  b.push_back(static_cast<uint8_t>(v >> 8));
}
void u32le(Bytes& b, uint32_t v) {
  for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>(v >> (8 * i)));
}
void f32le(Bytes& b, float f) {
  uint32_t v;
  std::memcpy(&v, &f, 4);
  u32le(b, v);
}
void doubles(Bytes& b, std::initializer_list<double> values) {
  for (double d : values) {
    uint64_t v;
    std::memcpy(&v, &d, 8);
    for (int i = 0; i < 8; ++i) b.push_back(static_cast<uint8_t>(v >> (8 * i)));
  }
}
void put_u32(Bytes& b, size_t at, uint32_t v) {
  for (int i = 0; i < 4; ++i) b[at + i] = static_cast<uint8_t>(v >> (8 * i));
}
// ASCII-only framed string with a short count.
void string(Bytes& b, std::string_view text) {
  b.insert(b.end(), {255, 254, 255, static_cast<uint8_t>(text.size())});
  for (char c : text) u16le(b, static_cast<uint8_t>(c));
}
ByteSpan span(const Bytes& b, size_t n) { return ByteSpan(b.data(), n); }

std::array<double, 3> rgb_of(uint32_t packed) {
  return {(packed & 255) / 255.0, ((packed >> 8) & 255) / 255.0, ((packed >> 16) & 255) / 255.0};
}

Bytes custom_pen_bytes(uint32_t rgb) {
  Bytes b;
  for (uint32_t v : {88u, 12u, rgb}) u32le(b, v);
  u16le(b, 0xffff);
  f32le(b, -1.0f);
  u32le(b, 0xffffffff);
  u32le(b, 1);
  b.push_back(15);
  for (char c : std::string_view("Continuous     ")) b.push_back(static_cast<uint8_t>(c));
  return b;
}

void font_bytes(Bytes& b) {
  u32le(b, 126);
  u32le(b, 10);
  doubles(b, {-1.});
  u32le(b, 12);
  b.insert(b.end(), 20, 0);
  string(b, "Century Gothic");
  doubles(b, {1., 0.});
}

Bytes plane_fixture() {
  Bytes b;
  for (uint32_t v : {168u, 6u, 0u, 5u}) u32le(b, v);
  doubles(b, {0., 0., 0., 1., 0., 0., 1., 1., 0., 0., 1., 0., 0., 0., 0.});
  font_bytes(b);
  font_bytes(b);
  for (uint32_t v : {206u, 18u}) u32le(b, v);
  doubles(b, {0.01, 0.02, 0.03, 0., 0.});
  string(b, "Plane1");
  u16le(b, 1);
  u32le(b, 6);
  for (int i = 0; i < 6; ++i) f32le(b, 0.002f);
  u32le(b, 1);
  doubles(b, {1.});
  return b;
}

Bytes scene_fixture(size_t* mode_offset = nullptr) {
  Bytes b;
  string(b, "templatewidth=0.279400,templateheight=0.215900");
  b.insert(b.end(), 16, 0);
  b.push_back(1);
  for (int i = 0; i < 13; ++i) doubles(b, {1.});
  b.insert(b.end(), 2, 0);
  for (int i = 0; i < 4; ++i) doubles(b, {1.});
  b.push_back(0);
  string(b, "");
  u32le(b, 1);
  if (mode_offset) *mode_offset = b.size();
  for (uint32_t v : {0u, 1u}) u32le(b, v);
  u16le(b, 0);
  u16le(b, 75);
  return b;
}

void display_archive_version_uses_root_names_and_rejects_ambiguity() {
  using N = std::vector<std::string>;
  CHECK(display_version(N{"_DL_VERSION_10000/DLUpdateStamp", "_MO_VERSION_12000/History"}) ==
        10000u);
  CHECK(display_version(N{"_DL_VERSION_3160/a", "_DL_VERSION_3160/b"}) == 3160u);
  CHECK(display_version(N{"_DL_VERSION_0/a", "_MO_VERSION_3238/History"}) == 3238u);
  CHECK(display_version(N{"_MO_VERSION_3905/History"}) == 3905u);
  for (const N& names : {N{"Contents/DisplayLists"}, N{"Contents/_DL_VERSION_3160/a"},
                         N{"_DL_VERSION_0/a"}, N{"_DL_VERSION_/a"}, N{"_DL_VERSION_3160junk/a"},
                         N{"_DL_VERSION_4294967296/a"},
                         N{"_DL_VERSION_3160/a", "_DL_VERSION_3238/b"}}) {
    CHECK(!display_version(names));
  }
}

void custom_pen_reads_versioned_fields_without_guessing_another_layout() {
  for (uint32_t version : {3159u, 3160u, 3237u, 3238u, 3904u, 3905u, 10000u}) {
    Bytes b = custom_pen_bytes(0xb42d17);
    if (version < 3905) {
      b.erase(b.begin() + 12, b.begin() + 18);
      Bytes w;
      u32le(w, 0xffffffff);
      b.insert(b.begin() + 12, w.begin(), w.end());
      if (version < 3238) b.erase(b.begin() + 20, b.begin() + 24);
      if (version < 3160) b.erase(b.begin() + 16, b.begin() + 20);
    }
    auto pen = custom_pen_color(b, 0, version);
    CHECK(pen);
    CHECK(pen->first == rgb_of(0xb42d17));
    CHECK(pen->second == b.size());
    CHECK(!custom_pen_color(b, 0, std::nullopt));
    for (size_t n = 0; n < b.size(); ++n) CHECK(!custom_pen_color(span(b, n), 0, version));
    if (version < 3905) CHECK(!custom_pen_color(b, 0, 10000u));
  }
}

void custom_pen_retains_colorref_channels_and_serialized_end() {
  for (uint32_t packed : {0u, 0xffffffu, 0xff00ffu, 0xb42d17u}) {
    Bytes b = custom_pen_bytes(packed);
    auto pen = custom_pen_color(b, 0, 10000u);
    CHECK(pen);
    CHECK(pen->first == rgb_of(packed));
    CHECK(pen->second == 42);
  }
}

void custom_pen_requires_complete_record_and_valid_color() {
  const Bytes original = custom_pen_bytes(0xb42d17);
  for (size_t n = 0; n < original.size(); ++n)
    CHECK(!custom_pen_color(span(original, n), 0, 10000u));
  float nan = std::nanf("");
  uint32_t nan_bits;
  std::memcpy(&nan_bits, &nan, 4);
  const std::pair<size_t, uint32_t> edits[] = {
      {0, 90u}, {4, 9u}, {8, 0x01000000u}, {14, nan_bits}, {22, 2u}};
  for (auto [offset, value] : edits) {
    Bytes b = original;
    put_u32(b, offset, value);
    CHECK(!custom_pen_color(b, 0, 10000u));
  }
}

void custom_pen_accepts_counted_ansi_and_unicode_styles() {
  for (bool unicode : {false, true}) {
    Bytes b = custom_pen_bytes(0xb42d17);
    b.resize(26);
    if (unicode) {
      string(b, "Continuous     ");
    } else {
      b.push_back(255);
      u16le(b, 15);
      for (char c : std::string_view("Continuous     ")) b.push_back(static_cast<uint8_t>(c));
    }
    auto pen = custom_pen_color(b, 0, 10000u);
    CHECK(pen);
    CHECK(pen->second == b.size());
    for (size_t n = 26; n < b.size(); ++n) CHECK(!custom_pen_color(span(b, n), 0, 10000u));
  }
}

void indexed_pen_color_requires_a_complete_style_command() {
  Bytes b(42, 0);
  put_u32(b, 0, 60);
  put_u32(b, 4, 9);
  for (uint32_t index : {15u, 3u, 123u}) {
    put_u32(b, 8, index);
    auto pen = pen_color(b, 0, 10000u);
    CHECK(pen);
    CHECK(pen->first == (PenColor{PenColor::Indexed, index, {}}));
    CHECK(pen->second == 42);
  }
  CHECK(!pen_color(span(b, 41), 0, 10000u));
  b[4] = 10;
  CHECK(!pen_color(b, 0, 10000u));
}

void indexed_pen_custom_fallback_and_versioned_lengths() {
  const std::pair<uint32_t, size_t> layouts[] = {{3237, 32}, {3238, 36}, {3904, 36}, {3905, 38},
                                                 {6233, 38}, {6234, 42}, {10000, 42}};
  for (auto [version, length] : layouts) {
    struct Case {
      int32_t primary, fallback;
      PenColor expected;
    };
    const Case cases[] = {
        {-2, 49, PenColor{PenColor::Custom, 0, rgb_of(0xb42d17)}},
        {-1, 49, PenColor{PenColor::Indexed, 49, {}}},
        {-3, -1, PenColor{PenColor::Unchanged, 0, {}}},
        {15, 49, PenColor{PenColor::Indexed, 15, {}}},
    };
    for (const auto& c : cases) {
      Bytes b;
      for (uint32_t w : {60u, 9u, static_cast<uint32_t>(c.primary),
                         static_cast<uint32_t>(c.fallback), 0xb42d17u, 0u})
        u32le(b, w);
      if (version >= 3905) {
        u16le(b, 0xffff);
        f32le(b, -1.0f);
      } else {
        u32le(b, 0xffffffff);
      }
      u32le(b, 99);
      if (version >= 3238) u32le(b, 0);
      if (version >= 6234) u32le(b, 0);
      CHECK(b.size() == length);
      auto pen = pen_color(b, 0, version);
      CHECK(pen);
      CHECK(pen->first == c.expected);
      CHECK(pen->second == length);
      CHECK(!pen_color(b, 0, std::nullopt));
      for (size_t n = 0; n < length; ++n) CHECK(!pen_color(span(b, n), 0, version));
    }
  }
}

void reference_annotation_reads_coordinates_label_and_advances() {
  const Bytes b = plane_fixture();
  auto p = plane(b, 0);
  CHECK(p);
  CHECK(p->label == "Plane1");
  CHECK(std::abs(p->origin.x - 10.) < 1e-12 && std::abs(p->origin.y - 20.) < 1e-12 &&
        std::abs(p->origin.z - 30.) < 1e-12);
  CHECK(p->corners.size() == 5 && p->corners[2].x == 1000. && p->corners[2].y == 1000.);
  CHECK(p->font == "Century Gothic" && p->point_size == 12);
  CHECK(p->advances.size() == 6 && p->advances[0] == static_cast<double>(0.002f) * 1000.0);
  CHECK(p->scale == 1.);
  CHECK(!p->color_index && !p->custom);
  ByteSpan truncated = span(b, b.size() - 1);
  CHECK(!plane(truncated, 0));
  CHECK(!plane(truncated, 16));
}

void scene_mode_is_separate_from_mesh_material() {
  size_t mode_offset = 0;
  Bytes b = scene_fixture(&mode_offset);
  auto mode = saved_mode(b, 0);
  CHECK(mode && mode->first == 0 && mode->second == b.size());
  Bytes shaded = b;
  shaded[mode_offset] = 3;
  mode = saved_mode(shaded, 0);
  CHECK(mode && mode->first == 3 && mode->second == b.size());
  // Invalid template dimensions and an out-of-range mode are not scenes.
  Bytes bad = b;
  bad[mode_offset] = 5;
  CHECK(!saved_mode(bad, 0));
}

void plane_pen_state_changes_without_cross_contaminating_custom_and_indexed_colors() {
  Bytes b = scene_fixture();
  const Bytes annotation = plane_fixture();
  Bytes indexed(42, 0);
  put_u32(indexed, 0, 60);
  put_u32(indexed, 4, 9);
  put_u32(indexed, 8, 49);
  auto add = [&](const Bytes& x) { b.insert(b.end(), x.begin(), x.end()); };
  add(indexed);
  add(annotation);
  add(custom_pen_bytes(0xb42d17));
  add(annotation);
  add(annotation);  // inherit within the same pen state
  add(indexed);
  add(annotation);  // indexed command resets explicit RGB
  add(custom_pen_bytes(0x01000000));
  add(annotation);  // unsupported custom command must clear the old pen
  Bytes explicit_pen = indexed;
  put_u32(explicit_pen, 8, static_cast<uint32_t>(-2));
  put_u32(explicit_pen, 16, 0xb42d17);
  add(explicit_pen);
  add(annotation);
  Bytes inherit = indexed;
  put_u32(inherit, 8, static_cast<uint32_t>(-1));
  put_u32(inherit, 12, static_cast<uint32_t>(-1));
  add(inherit);
  add(annotation);
  put_u32(inherit, 12, 49);
  add(inherit);
  add(annotation);
  for (char c : std::string_view("uoTempBodyTessData_c")) b.push_back(static_cast<uint8_t>(c));

  std::optional<int64_t> mode;
  std::vector<ReferencePlane> planes;
  append(b, 10000u, mode, planes);
  CHECK(mode == 0);
  CHECK(planes.size() == 8);  // cadmpeg out[1..9]
  CHECK(planes[0].color_index == 49 && !planes[0].custom);
  for (size_t at : {1u, 2u, 5u, 6u}) {
    CHECK(planes[at].custom && !planes[at].color_index);
    CHECK(planes[at].custom_rgb == rgb_of(0xb42d17));
  }
  CHECK(planes[3].color_index == 49 && !planes[3].custom);
  CHECK(!planes[4].color_index && !planes[4].custom);
  CHECK(planes[7].color_index == 49 && !planes[7].custom);

  // Without an archive version every pen is unresolved.
  std::optional<int64_t> mode2;
  std::vector<ReferencePlane> planes2;
  append(b, std::nullopt, mode2, planes2);
  CHECK(planes2.size() == 8);
  for (const auto& p : planes2) CHECK(!p.color_index && !p.custom);

  // A second scene header makes the stream ambiguous: nothing is emitted.
  Bytes twice = scene_fixture();
  Bytes rest = b;
  twice.insert(twice.end(), rest.begin(), rest.end());
  std::optional<int64_t> mode3;
  std::vector<ReferencePlane> planes3;
  append(twice, 10000u, mode3, planes3);
  CHECK(!mode3 && planes3.empty());

  // No end token: nothing is emitted.
  Bytes no_end(b.begin(), b.end() - 20);
  std::optional<int64_t> mode4;
  std::vector<ReferencePlane> planes4;
  append(no_end, 10000u, mode4, planes4);
  CHECK(!mode4 && planes4.empty());
}

void truncated_framed_label_is_malformed() {
  Bytes b = plane_fixture();
  // Cut inside the label's code units: the framed string read fails.
  size_t label = 0;
  for (size_t i = 0; i + 4 < b.size(); ++i)
    if (b[i] == 255 && b[i + 1] == 254 && b[i + 2] == 255 && b[i + 3] == 6) label = i;
  CHECK(label != 0);
  bool threw = false;
  try {
    plane(span(b, label + 6), 0);
  } catch (const DecodeError&) {
    threw = true;
  }
  CHECK(threw);
}

void write_dir_entry(uint8_t* entry, std::string_view name, uint8_t type, uint32_t child) {
  for (size_t i = 0; i < name.size(); ++i) {
    entry[2 * i] = static_cast<uint8_t>(name[i]);
    entry[2 * i + 1] = 0;
  }
  uint16_t len = static_cast<uint16_t>((name.size() + 1) * 2);
  entry[64] = static_cast<uint8_t>(len);
  entry[65] = static_cast<uint8_t>(len >> 8);
  entry[66] = type;
  entry[67] = 1;
  std::memset(entry + 68, 0xff, 8);
  for (int i = 0; i < 4; ++i) entry[76 + i] = static_cast<uint8_t>(child >> (8 * i));
  std::memset(entry + 116, 0xff, 4);
}

void empty_cfb_version_storage_is_not_lost_by_stream_only_scanning() {
  constexpr size_t kSector = 512;
  Bytes file(kSector * 3, 0);
  const uint8_t magic[] = {0xd0, 0xcf, 0x11, 0xe0, 0xa1, 0xb1, 0x1a, 0xe1};
  std::memcpy(file.data(), magic, 8);
  auto w16 = [&](size_t at, uint16_t v) {
    file[at] = static_cast<uint8_t>(v);
    file[at + 1] = static_cast<uint8_t>(v >> 8);
  };
  w16(24, 0x003e);
  w16(26, 3);
  w16(28, 0xfffe);
  w16(30, 9);
  w16(32, 6);
  put_u32(file, 44, 1);
  put_u32(file, 48, 0);
  put_u32(file, 56, 4096);
  put_u32(file, 60, 0xfffffffe);
  put_u32(file, 68, 0xfffffffe);
  for (size_t i = 0; i < 109; ++i) put_u32(file, 76 + i * 4, 0xffffffff);
  put_u32(file, 76, 1);
  uint8_t* dir = file.data() + kSector;
  for (size_t e = 0; e < 4; ++e) std::memset(dir + e * 128 + 68, 0xff, 12);
  write_dir_entry(dir, "Root Entry", 5, 1);
  write_dir_entry(dir + 128, "_DL_VERSION_3238", 1, 0xffffffff);
  uint8_t* fat = file.data() + kSector * 2;
  std::memset(fat, 0xff, kSector);
  put_u32(file, kSector * 2, 0xfffffffe);
  put_u32(file, kSector * 2 + 4, 0xfffffffd);

  ContainerError error;
  auto container = read_container(file, &error);
  CHECK(container);
  CHECK(container->sections.empty());
  CHECK(display_version(container->entry_names) == 3238u);
}

}  // namespace

int main() {
  display_archive_version_uses_root_names_and_rejects_ambiguity();
  custom_pen_reads_versioned_fields_without_guessing_another_layout();
  custom_pen_retains_colorref_channels_and_serialized_end();
  custom_pen_requires_complete_record_and_valid_color();
  custom_pen_accepts_counted_ansi_and_unicode_styles();
  indexed_pen_color_requires_a_complete_style_command();
  indexed_pen_custom_fallback_and_versioned_lengths();
  reference_annotation_reads_coordinates_label_and_advances();
  scene_mode_is_separate_from_mesh_material();
  plane_pen_state_changes_without_cross_contaminating_custom_and_indexed_colors();
  truncated_framed_label_is_malformed();
  empty_cfb_version_storage_is_not_lost_by_stream_only_scanning();
  std::puts("presentation_test: ok");
  return 0;
}
