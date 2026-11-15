// SPDX-License-Identifier: Apache-2.0
// `Contents/DisplayLists` descriptor tables (`uoTempFaceTessData_c`).
// Ported from cadmpeg-codec-sldprt tessellation.rs, framed_string.rs and
// layout.rs, and cadmpeg-core decode/utf16.rs (cadmpeg 5af95421,
// Apache-2.0). Loss notes, resource-budget charging and exact-read
// ledgers are not ported.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "display_lists_internal.h"
#include "errors.h"

namespace swglb {
namespace {

constexpr std::string_view kFaceTessellationClass = "uoTempFaceTessData_c";
constexpr uint8_t kClassMarker[4] = {0xff, 0xff, 0x01, 0x00};

// layout.rs `display_lists_compact_face_header`.
namespace compact_face {
constexpr size_t LEN = 8, TRIANGLE_COUNT = 0, STRIP_COUNT = 4;
}
// layout.rs `display_lists_extended_face_header`.
namespace extended_face {
constexpr size_t LEN = 40, FORM = 8, ZERO_AT_12 = 12, ZERO_AT_16 = 16, FORM_TOKEN = 20,
                 ZERO_TAIL = 24;
}
// layout.rs `display_lists_extended_metadata`.
namespace extended_meta {
constexpr size_t ONE = 96, FACE_KEY = 100;
constexpr uint32_t ONE_VALUE = 1;
}  // namespace extended_meta

// tessellation.rs reused-tag constants.
constexpr size_t kReusedTagProbeLen = 10;
constexpr uint16_t kClassTagFamilyBit = 0x8000;
constexpr uint16_t kClassTagAbsentHigh = 0xffff;
constexpr uint16_t kClassTagAbsentLow = 0x8000;

bool is_rust_whitespace(char32_t c) {
  return (c >= 0x09 && c <= 0x0d) || c == 0x20 || c == 0x85 || c == 0xa0 || c == 0x1680 ||
         (c >= 0x2000 && c <= 0x200a) || c == 0x2028 || c == 0x2029 || c == 0x202f ||
         c == 0x205f || c == 0x3000;
}

/// Rust `u32::from_str` (radix 10): optional '+', at least one digit.
std::optional<uint32_t> parse_u32(std::u32string_view s) {
  if (!s.empty() && s.front() == U'+') s.remove_prefix(1);
  if (s.empty()) return std::nullopt;
  uint64_t value = 0;
  for (char32_t c : s) {
    if (c < U'0' || c > U'9') return std::nullopt;
    value = value * 10 + (c - U'0');
    if (value > UINT32_MAX) return std::nullopt;
  }
  return static_cast<uint32_t>(value);
}

/// Rust `i32::from_str` (radix 10): optional sign, at least one digit.
std::optional<int32_t> parse_i32(std::u32string_view s) {
  bool negative = false;
  if (!s.empty() && (s.front() == U'+' || s.front() == U'-')) {
    negative = s.front() == U'-';
    s.remove_prefix(1);
  }
  if (s.empty()) return std::nullopt;
  int64_t value = 0;
  for (char32_t c : s) {
    if (c < U'0' || c > U'9') return std::nullopt;
    value = value * 10 + (c - U'0');
    if (value > (negative ? 2147483648LL : 2147483647LL)) return std::nullopt;
  }
  return static_cast<int32_t>(negative ? -value : value);
}

bool starts_with(std::u32string_view s, std::string_view prefix) {
  if (s.size() < prefix.size()) return false;
  for (size_t i = 0; i < prefix.size(); ++i)
    if (s[i] != static_cast<unsigned char>(prefix[i])) return false;
  return true;
}

bool ends_with(std::u32string_view s, std::string_view suffix) {
  if (s.size() < suffix.size()) return false;
  size_t base = s.size() - suffix.size();
  for (size_t i = 0; i < suffix.size(); ++i)
    if (s[base + i] != static_cast<unsigned char>(suffix[i])) return false;
  return true;
}

uint32_t le32(ByteSpan b, size_t at) { return *u32_at(b, at); }

// tessellation.rs `TABLE_DESCRIPTORS`: (item_size, kind, flags).
constexpr uint32_t kTableDescriptors[6][3] = {{4, 8, 2},   {12, 100, 2}, {12, 100, 2},
                                              {4, 8, 2},   {4, 8, 2},    {1, 8, 2}};

struct Lane {
  size_t data = 0, end = 0;
  uint32_t count = 0;
};

// cadmpeg tessellation.rs `auxiliary_lanes_are_consistent`.
bool auxiliary_lanes_are_consistent(ByteSpan bytes, const std::vector<uint32_t>& strips,
                                    const Lane& b, const Lane& c, const Lane& d) {
  uint64_t endpoint_count = 0;
  for (uint32_t length : strips) {
    if (length == 0) return false;  // checked_sub(2) of length * 2
    endpoint_count += static_cast<uint64_t>(length) * 2 - 2;
  }
  // c: one u32 per strip, each the strip's endpoint count.
  if ((c.end - c.data) / 4 != strips.size()) return false;
  for (size_t i = 0; i < strips.size(); ++i)
    if (le32(bytes, c.data + i * 4) != static_cast<uint64_t>(strips[i]) * 2 - 2) return false;
  bool counts = (b.count == 0 && d.count == 0) ||
                (b.count == endpoint_count && d.count == endpoint_count);
  if (!counts || c.count != strips.size()) return false;
  for (size_t at = b.data; at + 4 <= b.end; at += 4)
    if (!std::isfinite(*f32_at(bytes, at))) return false;
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// Text helpers
// ---------------------------------------------------------------------------

// cadmpeg-core decode/utf16.rs `decode_text` with Surrogates::Reject.
std::optional<std::u32string> decode_utf16le_strict(ByteSpan raw, size_t units) {
  if (raw.size() / 2 < units) return std::nullopt;
  std::u32string out;
  out.reserve(units);
  for (size_t i = 0; i < units; ++i) {
    char32_t unit = raw[2 * i] | (raw[2 * i + 1] << 8);
    if (unit >= 0xd800 && unit <= 0xdbff) {
      if (i + 1 >= units) return std::nullopt;
      char32_t low = raw[2 * i + 2] | (raw[2 * i + 3] << 8);
      if (low < 0xdc00 || low > 0xdfff) return std::nullopt;
      out.push_back(0x10000 + ((unit - 0xd800) << 10) + (low - 0xdc00));
      ++i;
    } else if (unit >= 0xdc00 && unit <= 0xdfff) {
      return std::nullopt;
    } else {
      out.push_back(unit);
    }
  }
  return out;
}

std::u32string_view trim_unicode(std::u32string_view text) {
  while (!text.empty() && is_rust_whitespace(text.front())) text.remove_prefix(1);
  while (!text.empty() && is_rust_whitespace(text.back())) text.remove_suffix(1);
  return text;
}

std::string to_utf8(std::u32string_view text) {
  std::string out;
  out.reserve(text.size());
  for (char32_t c : text) {
    if (c < 0x80) {
      out.push_back(static_cast<char>(c));
    } else if (c < 0x800) {
      out.push_back(static_cast<char>(0xc0 | (c >> 6)));
      out.push_back(static_cast<char>(0x80 | (c & 0x3f)));
    } else if (c < 0x10000) {
      out.push_back(static_cast<char>(0xe0 | (c >> 12)));
      out.push_back(static_cast<char>(0x80 | ((c >> 6) & 0x3f)));
      out.push_back(static_cast<char>(0x80 | (c & 0x3f)));
    } else {
      out.push_back(static_cast<char>(0xf0 | (c >> 18)));
      out.push_back(static_cast<char>(0x80 | ((c >> 12) & 0x3f)));
      out.push_back(static_cast<char>(0x80 | ((c >> 6) & 0x3f)));
      out.push_back(static_cast<char>(0x80 | (c & 0x3f)));
    }
  }
  return out;
}

// cadmpeg framed_string.rs `read_framed_string` / `read_after_marker`.
std::optional<FramedText> read_framed_string(ByteSpan data, size_t at) {
  static constexpr uint8_t kMarker[3] = {0xff, 0xfe, 0xff};
  if (!starts_with_at(data, at, ByteSpan(kMarker, 3))) return std::nullopt;
  size_t pos = at + 3;
  auto first = u8_at(data, pos);
  if (!first) return std::nullopt;
  pos += 1;
  uint64_t count = *first;
  if (*first == 0xff) {
    auto second = u16_at(data, pos);
    if (!second) return std::nullopt;
    pos += 2;
    count = *second;
    if (*second == 0xffff) {
      auto far = u32_at(data, pos);
      if (!far) return std::nullopt;
      pos += 4;
      count = *far;
    }
  }
  size_t remaining = data.size() - pos;
  if (count * 2 > remaining) return std::nullopt;
  auto value = decode_utf16le_strict(data.subspan(pos, count * 2), count);
  if (!value) return std::nullopt;
  return FramedText{std::move(*value), pos + count * 2};
}

// ---------------------------------------------------------------------------
// Class declarations
// ---------------------------------------------------------------------------

// cadmpeg tessellation.rs `class_intervals`.
std::vector<ClassInterval> class_intervals(ByteSpan payload) {
  std::vector<std::pair<size_t, std::string>> declarations;
  const uint8_t* base = payload.data();
  size_t n = payload.size();
  size_t at = 0;
  while (n >= 4 && at + 4 <= n) {
    const void* hit = std::memchr(base + at, 0xff, n - 3 - at);
    if (!hit) break;
    size_t offset = static_cast<size_t>(static_cast<const uint8_t*>(hit) - base);
    at = offset + 1;
    if (std::memcmp(base + offset, kClassMarker, 4) != 0) continue;
    auto length = u16_at(payload, offset + 4);
    if (!length || *length < 1 || *length > 128) continue;
    if (offset + 6 + *length > n) continue;
    bool ok = true;
    for (size_t i = 0; i < *length; ++i) {
      uint8_t c = base[offset + 6 + i];
      if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            c == '_' || c == '-')) {
        ok = false;
        break;
      }
    }
    if (!ok) continue;
    declarations.emplace_back(offset,
                              std::string(reinterpret_cast<const char*>(base + offset + 6), *length));
  }
  std::vector<ClassInterval> intervals;
  intervals.reserve(declarations.size());
  for (size_t i = 0; i < declarations.size(); ++i) {
    auto& [offset, name] = declarations[i];
    size_t start = offset + 6 + name.size();
    size_t end = i + 1 < declarations.size() ? declarations[i + 1].first : n;
    if (start > end) continue;
    intervals.push_back(ClassInterval{std::move(name), offset, ByteRange{start, end}});
  }
  return intervals;
}

// ---------------------------------------------------------------------------
// Descriptor tables
// ---------------------------------------------------------------------------

// cadmpeg tessellation.rs `probe_table` + `parse_table`. The six descriptor
// headers are validated before any lane is read, so a candidate whose later
// descriptor mismatches costs O(1) (cadmpeg reads the strip lane first; the
// verdict is the same because every failure is a probe miss).
std::optional<ParsedTable> parse_table(ByteSpan bytes, size_t at) {
  Lane lanes[6];
  size_t pos = at;
  for (size_t index = 0; index < 6; ++index) {
    auto item_size = u32_at(bytes, pos);
    auto kind = u32_at(bytes, pos + 4);
    auto flags = u32_at(bytes, pos + 8);
    if (!item_size || !kind || !flags) return std::nullopt;
    if (*item_size != kTableDescriptors[index][0] || *kind != kTableDescriptors[index][1] ||
        *flags != kTableDescriptors[index][2])
      return std::nullopt;
    auto count = u32_at(bytes, pos + 12);
    if (!count) return std::nullopt;
    size_t data = pos + 16;
    uint64_t size = static_cast<uint64_t>(*item_size) * *count;
    if (size > bytes.size() || data > bytes.size() - size) return std::nullopt;
    lanes[index] = Lane{data, data + static_cast<size_t>(size), *count};
    pos = lanes[index].end;
  }
  std::vector<uint32_t> strips(lanes[0].count);
  for (size_t i = 0; i < strips.size(); ++i) strips[i] = le32(bytes, lanes[0].data + i * 4);
  auto read_vec = [&](size_t p, Vec3& out) {
    float x = *f32_at(bytes, p), y = *f32_at(bytes, p + 4), z = *f32_at(bytes, p + 8);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
    out = Vec3{static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)};
    return true;
  };
  std::vector<MeshVertex> vertices(lanes[1].count);
  for (size_t i = 0; i < vertices.size(); ++i) {
    Vec3 p;
    if (!read_vec(lanes[1].data + i * 12, p)) return std::nullopt;
    vertices[i].position = Vec3{p.x * 1000.0, p.y * 1000.0, p.z * 1000.0};
  }
  size_t normal_count = lanes[2].count;
  std::vector<Vec3> normals(normal_count);
  for (size_t i = 0; i < normal_count; ++i)
    if (!read_vec(lanes[2].data + i * 12, normals[i])) return std::nullopt;
  if (strips.empty() || vertices.empty()) return std::nullopt;
  if (!auxiliary_lanes_are_consistent(bytes, strips, lanes[3], lanes[4], lanes[5]))
    return std::nullopt;

  // TessellationMesh::from_strip_lanes: a recognised table refuses its lane
  // errors.
  ParsedTable table;
  table.end = pos;
  table.has_normals = normal_count != 0;
  if (table.has_normals) {
    if (normal_count != vertices.size())
      throw DecodeError("sldprt display-list table at byte " + std::to_string(at) + ": " +
                        std::to_string(vertices.size()) + " vertex position(s) but " +
                        std::to_string(normal_count) + " vertex normal(s)");
    for (size_t i = 0; i < vertices.size(); ++i) vertices[i].normal = normals[i];
  }
  auto strip_error = [&] {
    return DecodeError("sldprt display-list table at byte " + std::to_string(at) + ": " +
                       std::to_string(strips.size()) +
                       " strip span(s) do not cut the vertex lane");
  };
  table.strips.reserve(strips.size());
  size_t next = 0;
  for (uint32_t span : strips) {
    if (span < 3 || span > vertices.size() - next) throw strip_error();
    table.strips.emplace_back(vertices.begin() + static_cast<std::ptrdiff_t>(next),
                              vertices.begin() + static_cast<std::ptrdiff_t>(next + span));
    next += span;
  }
  if (next != vertices.size()) throw strip_error();
  return table;
}

// cadmpeg tessellation.rs `descriptor_table_offset`.
size_t descriptor_table_offset(ByteSpan payload, size_t at) {
  bool extended = u32_at(payload, at + extended_face::FORM) == 1u &&
                  u32_at(payload, at + extended_face::ZERO_AT_12) == 0u &&
                  u32_at(payload, at + extended_face::ZERO_AT_16) == 0u;
  if (extended) {
    auto token = u32_at(payload, at + extended_face::FORM_TOKEN);
    extended = token && *token != 0;
  }
  if (extended) {
    auto tail = slice(payload, at + extended_face::ZERO_TAIL, at + extended_face::LEN);
    extended = tail && std::all_of(tail->begin(), tail->end(), [](uint8_t b) { return b == 0; });
  }
  return extended ? extended_face::LEN : compact_face::LEN;
}

namespace {

struct TableSpan {
  size_t start, end;
  ParsedTable table;
};

size_t vertex_count(const ParsedTable& t) {
  size_t n = 0;
  for (const auto& s : t.strips) n += s.size();
  return n;
}

size_t triangle_count(const std::vector<std::vector<MeshVertex>>& strips) {
  size_t n = 0;
  for (const auto& s : strips) n += s.size() - 2;
  return n;
}

// cadmpeg tessellation.rs `parse_table_sequence`.
std::optional<std::vector<TableSpan>> parse_table_sequence(ByteSpan payload, size_t at,
                                                           size_t limit) {
  size_t first_start = at;
  auto first = parse_table(payload, at);
  if (!first) return std::nullopt;
  at = first->end;
  if (at > limit) return std::nullopt;
  if (vertex_count(*first) == 0) return std::nullopt;
  std::vector<TableSpan> tables;
  tables.push_back(TableSpan{first_start, at, std::move(*first)});
  static constexpr uint8_t kItemSize4[4] = {4, 0, 0, 0};
  const uint8_t* base = payload.data();
  while (at + 16 <= limit) {
    // payload[at..limit].windows(4).position(== [4, 0, 0, 0])
    size_t found = SIZE_MAX;
    for (size_t p = at; p + 4 <= limit;) {
      const void* hit = std::memchr(base + p, 4, limit - 3 - p);
      if (!hit) break;
      size_t q = static_cast<size_t>(static_cast<const uint8_t*>(hit) - base);
      if (std::memcmp(base + q, kItemSize4, 4) == 0) {
        found = q;
        break;
      }
      p = q + 1;
    }
    if (found == SIZE_MAX) break;
    at = found;
    size_t start = at;
    auto next = parse_table(payload, at);
    if (next && next->end <= limit && vertex_count(*next) > 0) {
      at = next->end;
      tables.push_back(TableSpan{start, at, std::move(*next)});
    } else {
      at += 4;
    }
  }
  return tables;
}

}  // namespace

// cadmpeg tessellation.rs `classify_surface_token`.
SurfaceReferencePtr classify_surface_token(std::u32string_view text, SurfaceTokenStatus* status) {
  auto invalid = [&]() -> SurfaceReferencePtr {
    if (status) *status = SurfaceTokenStatus::Invalid;
    return nullptr;
  };
  text = trim_unicode(text);
  std::vector<std::u32string_view> fields;
  for (size_t start = 0;;) {
    size_t comma = text.find(U',', start);
    if (comma == std::u32string_view::npos) {
      fields.push_back(text.substr(start));
      break;
    }
    fields.push_back(text.substr(start, comma - start));
    start = comma + 1;
  }
  if (!starts_with(fields[0], "mo") || !ends_with(fields[0], "SurfIdRep_c")) return invalid();
  std::optional<uint32_t> source = fields.size() > 1 ? parse_u32(fields[1]) : std::nullopt;
  if (source && (*source == 0 || *source == UINT32_MAX)) source.reset();  // FeatureSourceId
  std::optional<uint32_t> local = fields.size() > 2 ? parse_u32(fields[2]) : std::nullopt;
  if (!source || !local) return invalid();
  std::vector<std::u32string_view> trailing;
  for (size_t i = 3; i < fields.size(); ++i) trailing.push_back(fields[i]);
  if (!trailing.empty() && trim_unicode(trailing.back()).empty()) trailing.pop_back();
  auto reference = std::make_shared<DisplayFace::SurfaceReference>();
  reference->identity.feature_source_id = *source;
  reference->identity.local_id = *local;
  reference->complete = true;
  for (auto field : trailing) {
    auto value = parse_i32(field);
    if (!value) {
      reference->complete = false;
      reference->identity.trailing_fields.clear();
      break;
    }
    reference->identity.trailing_fields.push_back(static_cast<uint32_t>(*value));
  }
  if (status)
    *status = reference->complete ? SurfaceTokenStatus::Complete : SurfaceTokenStatus::OpaqueTail;
  return reference;
}

// cadmpeg tessellation.rs `persistent_surface_references`.
std::vector<SurfaceReferencePtr> persistent_surface_references(
    ByteSpan payload, ByteRange range, std::shared_ptr<const DisplaySectionIndex> section) {
  static constexpr uint8_t kMarker[3] = {0xff, 0xfe, 0xff};
  std::vector<SurfaceReferencePtr> references;
  size_t at = range.start;
  size_t stop = std::min(range.end, payload.size());
  const uint8_t* base = payload.data();
  while (at + 4 <= stop) {
    if (std::memcmp(base + at, kMarker, 3) != 0) {
      const void* hit = std::memchr(base + at + 1, 0xff, stop - 3 - at);
      if (!hit) break;
      at = static_cast<size_t>(static_cast<const uint8_t*>(hit) - base);
      continue;
    }
    size_t count = base[at + 3];
    size_t start = at + 4;
    size_t end = start + count * 2;
    if (end > range.end || end > payload.size()) {
      at += 1;
      continue;
    }
    auto text = decode_utf16le_strict(payload.subspan(start, count * 2), count);
    if (text) {
      if (auto reference = classify_surface_token(*text)) {
        if (section) {
          auto copy = std::make_shared<DisplayFace::SurfaceReference>(*reference);
          copy->section = section;
          reference = std::move(copy);
        }
        references.push_back(std::move(reference));
      }
    }
    at = end;
  }
  return references;
}

size_t face_triangle_count(const DisplayFace& face) { return triangle_count(face.strips); }

std::optional<uint32_t> face_feature_source_id(const DisplayFace& face) {
  if (face.surface_references.empty()) return std::nullopt;
  uint32_t source = face.surface_references[0]->identity.feature_source_id;
  for (const auto& r : face.surface_references)
    if (r->identity.feature_source_id != source) return std::nullopt;
  return source;
}

// cadmpeg tessellation.rs `section_display_faces`.
std::vector<DisplayFace> section_display_faces(ByteSpan payload) {
  std::vector<size_t> markers = find_all(payload, kFaceTessellationClass);
  std::vector<DisplayFace> faces;
  for (size_t m = 0; m < markers.size(); ++m) {
    size_t header = markers[m] + kFaceTessellationClass.size();
    auto triangles = u32_at(payload, header + compact_face::TRIANGLE_COUNT);
    auto strips = u32_at(payload, header + compact_face::STRIP_COUNT);
    if (!triangles || !strips) continue;
    // `uoBodyPropInfo_c` declarations separate body-property groups, but face
    // descriptor tables continue after them; only another face declaration
    // starts a new sequence.
    size_t limit = m + 1 < markers.size() ? markers[m + 1] : payload.size();
    size_t start = header + descriptor_table_offset(payload, header);
    auto tables = parse_table_sequence(payload, start, limit);
    if (!tables) continue;
    if (tables->empty())
      throw DecodeError("sldprt display-face header at byte " + std::to_string(header) +
                        " states no table");
    // Two zero counts state no mesh: the marker states no display face.
    if (*triangles == 0 && *strips == 0) continue;
    const auto& first = tables->front();
    size_t parsed_triangles = triangle_count(first.table.strips);
    size_t parsed_strips = first.table.strips.size();
    if (*triangles != parsed_triangles || *strips != parsed_strips)
      throw DecodeError("sldprt display-face table at byte " + std::to_string(first.start) +
                        ": header states " + std::to_string(*triangles) +
                        " triangle(s) and " + std::to_string(*strips) +
                        " strip(s); the parsed mesh has " + std::to_string(parsed_triangles) +
                        " triangle(s) and " + std::to_string(parsed_strips) + " strip(s)");
    for (auto& span : *tables) {
      if (span.start > span.end || span.end > limit) continue;
      DisplayFace face;
      face.strips = std::move(span.table.strips);
      face.has_normals = span.table.has_normals;
      face.table = ByteRange{span.start, span.end};
      face.metadata = ByteRange{span.end, limit};
      faces.push_back(std::move(face));
    }
  }
  std::stable_sort(faces.begin(), faces.end(), [](const DisplayFace& a, const DisplayFace& b) {
    return a.table.start < b.table.start;
  });
  std::shared_ptr<DisplaySectionIndex> index;
  for (size_t i = 0; i < faces.size(); ++i) {
    size_t metadata_end = i + 1 < faces.size() ? faces[i + 1].table.start : faces[i].metadata.end;
    auto& meta = faces[i].metadata;
    meta.end = std::max(std::min(metadata_end, meta.end), meta.start);  // ByteRange::truncated
    if (!index) index = std::make_shared<DisplaySectionIndex>();
    faces[i].surface_references = persistent_surface_references(payload, meta, index);
    for (const auto& r : faces[i].surface_references)
      faces[i].surface_reference_features.push_back(r->identity.feature_source_id);
  }
  return faces;
}

// cadmpeg decode.rs `collect_display_sections` (summaries are not kept).
std::vector<DisplaySection> collect_display_sections(const Container& container) {
  std::vector<DisplaySection> sections;
  for (size_t i = 0; i < container.sections.size(); ++i) {
    auto faces = section_display_faces(ByteSpan(container.sections[i].payload));
    if (faces.empty()) continue;
    sections.push_back(DisplaySection{i, std::move(faces)});
  }
  return sections;
}

namespace {

// cadmpeg tessellation.rs `table_counts_match`.
bool table_counts_match(ByteSpan payload, size_t at, const DisplayFace& face) {
  return u32_at(payload, at + compact_face::TRIANGLE_COUNT) == face_triangle_count(face) &&
         u32_at(payload, at + compact_face::STRIP_COUNT) == face_strip_count(face);
}

// cadmpeg tessellation.rs `extended_header_word`.
std::optional<uint32_t> extended_header_word(ByteSpan payload, const DisplayFace& face) {
  if (face.table.start < extended_face::LEN) return std::nullopt;
  size_t header = face.table.start - extended_face::LEN;
  if (descriptor_table_offset(payload, header) != extended_face::LEN) return std::nullopt;
  return u32_at(payload, header + extended_face::FORM_TOKEN);
}

// cadmpeg tessellation.rs `table_header_start`.
size_t table_header_start(ByteSpan payload, const DisplayFace& face) {
  if (extended_header_word(payload, face)) return face.table.start - extended_face::LEN;
  if (face.table.start >= compact_face::LEN &&
      table_counts_match(payload, face.table.start - compact_face::LEN, face))
    return face.table.start - compact_face::LEN;
  return face.table.start;
}

// cadmpeg tessellation.rs `bounded_metadata_end`.
size_t bounded_metadata_end(ByteSpan payload, const DisplayFace& face, const DisplayFace* next) {
  if (!next) return face.metadata.end;
  return std::min(face.metadata.end, table_header_start(payload, *next));
}

}  // namespace

// cadmpeg tessellation.rs `display_face_key`.
std::optional<uint32_t> display_face_key(ByteSpan payload, const std::vector<DisplayFace>& faces,
                                         size_t index) {
  if (index >= faces.size()) return std::nullopt;
  const DisplayFace& face = faces[index];
  size_t start = face.metadata.start;
  size_t key_end = start + extended_meta::FACE_KEY + 4;
  const DisplayFace* next = index + 1 < faces.size() ? &faces[index + 1] : nullptr;
  if (key_end > bounded_metadata_end(payload, face, next)) return std::nullopt;
  if (u32_at(payload, start + extended_meta::ONE) != extended_meta::ONE_VALUE) return std::nullopt;
  auto key = u32_at(payload, start + extended_meta::FACE_KEY);
  if (!key || *key == 0) return std::nullopt;
  return key;
}

// cadmpeg tessellation.rs `agree_surface_tokens`.
std::optional<PersistentFaceIdentity> agree_surface_tokens(
    const std::vector<SurfaceReferencePtr>& references, SurfaceTokenWithhold* reason) {
  auto withhold = [&](SurfaceTokenWithhold r) -> std::optional<PersistentFaceIdentity> {
    if (reason) *reason = r;
    return std::nullopt;
  };
  for (const auto& r : references)
    if (!r->complete) return withhold(SurfaceTokenWithhold::OpaqueTail);
  if (references.empty()) return withhold(SurfaceTokenWithhold::Invalid);
  const PersistentFaceIdentity& first = references[0]->identity;
  for (const auto& r : references)
    if (!(r->identity == first)) return withhold(SurfaceTokenWithhold::Conflicting);
  if (reason) *reason = SurfaceTokenWithhold::None;
  return first;
}

// cadmpeg tessellation.rs `admit_reused_face_tag`. The interval test is a
// binary search: class contents are disjoint and in payload order.
ReusedTagVerdict admit_reused_face_tag(ByteSpan payload, const DisplayFace& face,
                                       size_t previous_table_end,
                                       const std::vector<ClassInterval>& classes) {
  size_t table_start = face.table.start;
  if (table_start < kReusedTagProbeLen) return {};
  size_t probe_last = table_start - kReusedTagProbeLen;
  size_t triangles = face_triangle_count(face), strips = face_strip_count(face);
  std::optional<uint16_t> survivor;
  for (size_t at = previous_table_end; at <= probe_last; ++at) {
    auto tag = u16_at(payload, at);
    if (!tag) continue;
    if ((*tag & kClassTagFamilyBit) == 0 || *tag == kClassTagAbsentHigh ||
        *tag == kClassTagAbsentLow)
      continue;
    auto t = u32_at(payload, at + 2), s = u32_at(payload, at + 6);
    if (!t || !s || *t != triangles || *s != strips) continue;
    auto it = std::upper_bound(classes.begin(), classes.end(), at,
                               [](size_t v, const ClassInterval& c) { return v < c.content.start; });
    bool contained = it != classes.begin() && std::prev(it)->name == kFaceTessellationClass &&
                     at < std::prev(it)->content.end;
    if (!contained) continue;
    if (survivor) return {ReusedTagVerdict::Withhold, 0};
    survivor = *tag;
  }
  if (survivor) return {ReusedTagVerdict::Admit, *survivor};
  return {};
}

// cadmpeg tessellation.rs `persistent_join_verdict`.
std::optional<PersistentFaceIdentity> persistent_join(ByteSpan payload,
                                                      const std::vector<DisplayFace>& faces,
                                                      size_t index, bool* ambiguous_reused_tag) {
  if (ambiguous_reused_tag) *ambiguous_reused_tag = false;
  if (index >= faces.size()) return std::nullopt;
  const DisplayFace& face = faces[index];
  auto identity = agree_surface_tokens(face.surface_references);
  if (!identity) return std::nullopt;
  size_t previous_table_end = index > 0 ? faces[index - 1].table.end : face.table.start;
  const auto& section = face.surface_references.front()->section;
  std::vector<ClassInterval> local;
  const std::vector<ClassInterval>* classes = nullptr;
  if (section) {
    classes = &section->classes(payload);
  } else {
    local = class_intervals(payload);
    classes = &local;
  }
  auto verdict = admit_reused_face_tag(payload, face, previous_table_end, *classes);
  if (verdict.kind == ReusedTagVerdict::Withhold) {
    if (ambiguous_reused_tag) *ambiguous_reused_tag = true;
    return std::nullopt;
  }
  return identity;
}

}  // namespace swglb
