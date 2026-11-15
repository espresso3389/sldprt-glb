// SPDX-License-Identifier: Apache-2.0
// Saved display mode and planar reference annotations (DisplayLists scene
// header, reference-plane command 6/10/18, pen commands 9 and 12).
// Ported from cadmpeg-codec-sldprt `metadata/presentation.rs` with
// scene-mode, reference-plane pen and legacy pen extensions.
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "errors.h"
#include "presentation_internal.h"
#include "stages.h"

namespace swglb::presentation {
namespace {

// cadmpeg-core `View` over a window `[0, data.size())` with an absolute cursor.
// Every read advances only on success.
struct View {
  ByteSpan data;
  size_t pos = 0;

  size_t remaining() const { return data.size() - pos; }
  std::optional<uint8_t> u8() {
    auto v = u8_at(data, pos);
    if (v) pos += 1;
    return v;
  }
  std::optional<uint16_t> u16() {
    auto v = u16_at(data, pos);
    if (v) pos += 2;
    return v;
  }
  std::optional<uint32_t> u32() {
    auto v = u32_at(data, pos);
    if (v) pos += 4;
    return v;
  }
  std::optional<float> f32() {
    auto v = f32_at(data, pos);
    if (v) pos += 4;
    return v;
  }
  std::optional<double> f64() {
    auto v = f64_at(data, pos);
    if (v) pos += 8;
    return v;
  }
  std::optional<ByteSpan> take(size_t n) {
    if (n > remaining()) return std::nullopt;
    ByteSpan out = data.subspan(pos, n);
    pos += n;
    return out;
  }
};

#define CANDIDATE(var, expr)         \
  auto var##_opt = (expr);           \
  if (!var##_opt) return std::nullopt; \
  auto var = *var##_opt

void append_utf8(std::string& out, uint32_t cp) {
  if (cp < 0x80) {
    out.push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out.push_back(static_cast<char>(0xc0 | (cp >> 6)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  } else if (cp < 0x10000) {
    out.push_back(static_cast<char>(0xe0 | (cp >> 12)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3f)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  } else {
    out.push_back(static_cast<char>(0xf0 | (cp >> 18)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3f)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3f)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  }
}

struct Text {
  std::string value;     ///< UTF-8
  size_t units = 0;      ///< UTF-16 code units
  size_t scalars = 0;    ///< Unicode scalar values
};

// Strict UTF-16LE decode (cadmpeg-core `decode_text`, Surrogates::Reject).
// Returns nullopt on an unpaired surrogate.
std::optional<Text> decode_utf16(ByteSpan raw, size_t units) {
  Text t;
  t.units = units;
  t.value.reserve(units);
  for (size_t i = 0; i < units; ++i) {
    uint32_t u = static_cast<uint32_t>(raw[2 * i] | (raw[2 * i + 1] << 8));
    if (u >= 0xd800 && u <= 0xdbff) {
      if (i + 1 >= units) return std::nullopt;
      uint32_t lo = static_cast<uint32_t>(raw[2 * i + 2] | (raw[2 * i + 3] << 8));
      if (lo < 0xdc00 || lo > 0xdfff) return std::nullopt;
      u = 0x10000 + ((u - 0xd800) << 10) + (lo - 0xdc00);
      ++i;
    } else if (u >= 0xdc00 && u <= 0xdfff) {
      return std::nullopt;
    }
    append_utf8(t.value, u);
    ++t.scalars;
  }
  return t;
}

// cadmpeg framed_string.rs `read_framed_string` (+ presentation.rs `text`).
// nullopt: marker absent (view unchanged). A present marker with a truncated
// frame, oversized count or invalid code units is Malformed.
std::optional<Text> text(View& view) {
  size_t start = view.pos;
  auto marker = view.take(3);
  if (!marker) return std::nullopt;
  if ((*marker)[0] != 0xff || (*marker)[1] != 0xfe || (*marker)[2] != 0xff) {
    view.pos = start;
    return std::nullopt;
  }
  auto first = view.u8();
  if (!first) throw DecodeError("SLDPRT framed string: truncated count frame");
  uint64_t count = *first;
  if (*first == 0xff) {
    auto second = view.u16();
    if (!second) throw DecodeError("SLDPRT framed string: truncated count frame");
    if (*second == 0xffff) {
      auto far = view.u32();
      if (!far) throw DecodeError("SLDPRT framed string: truncated count frame");
      count = *far;
    } else {
      count = *second;
    }
  }
  if (count * 2 > view.remaining())
    throw DecodeError("SLDPRT framed string count exceeds its unread bytes");
  ByteSpan raw = *view.take(static_cast<size_t>(count) * 2);
  auto t = decode_utf16(raw, static_cast<size_t>(count));
  if (!t) throw DecodeError("invalid UTF-16LE surrogate sequence");
  return t;
}

// cadmpeg presentation.rs `numbers`.
template <size_t N>
std::optional<std::array<double, N>> numbers(View& view) {
  std::array<double, N> values{};
  for (auto& v : values) {
    auto d = view.f64();
    if (!d || !std::isfinite(*d)) return std::nullopt;
    v = *d;
  }
  return values;
}

// Rust `str::parse::<f64>` grammar (finite decimal forms only; inf/nan are
// rejected by the caller's finiteness test anyway).
std::optional<double> parse_f64(std::string_view s) {
  size_t i = 0;
  if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
  size_t digits = 0;
  while (i < s.size() && s[i] >= '0' && s[i] <= '9') ++i, ++digits;
  if (i < s.size() && s[i] == '.') {
    ++i;
    while (i < s.size() && s[i] >= '0' && s[i] <= '9') ++i, ++digits;
  }
  if (digits == 0) return std::nullopt;
  if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
    ++i;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
    size_t exp = 0;
    while (i < s.size() && s[i] >= '0' && s[i] <= '9') ++i, ++exp;
    if (exp == 0) return std::nullopt;
  }
  if (i != s.size()) return std::nullopt;
  std::string copy(s);
  return std::strtod(copy.c_str(), nullptr);
}

// cadmpeg presentation.rs `font`: (family, point size).
std::optional<std::pair<std::string, uint32_t>> font(View& view) {
  CANDIDATE(lead, view.u32());
  (void)lead;
  CANDIDATE(cmd, view.u32());
  if (cmd != 10) return std::nullopt;
  CANDIDATE(height, numbers<1>(view));
  (void)height;
  CANDIDATE(size, view.u32());
  if (size < 1 || size > 512) return std::nullopt;
  for (int i = 0; i < 5; ++i) {
    CANDIDATE(skip, view.u32());
    (void)skip;
  }
  CANDIDATE(name, text(view));
  CANDIDATE(tail, numbers<2>(view));
  (void)tail;
  return std::pair{std::move(name.value), size};
}

std::array<double, 3> colorref(uint32_t packed) {
  return {static_cast<double>(packed & 255) / 255.0,
          static_cast<double>((packed >> 8) & 255) / 255.0,
          static_cast<double>((packed >> 16) & 255) / 255.0};
}

bool all_finite(const std::vector<double>& v) {
  for (double x : v)
    if (!std::isfinite(x)) return false;
  return true;
}

}  // namespace

// cadmpeg presentation.rs `display_version`.
std::optional<uint32_t> display_version(const std::vector<std::string>& names) {
  std::optional<uint32_t> dl, mo;
  for (const auto& name : names) {
    std::string_view root = std::string_view(name).substr(0, name.find('/'));
    std::string_view suffix;
    std::optional<uint32_t>* slot;
    if (root.starts_with("_DL_VERSION_")) {
      suffix = root.substr(12);
      slot = &dl;
    } else if (root.starts_with("_MO_VERSION_")) {
      suffix = root.substr(12);
      slot = &mo;
    } else {
      continue;
    }
    if (suffix.empty()) return std::nullopt;
    uint64_t value = 0;
    for (char c : suffix) {
      if (c < '0' || c > '9') return std::nullopt;
      value = value * 10 + static_cast<uint64_t>(c - '0');
      if (value > 0xffffffffull) return std::nullopt;
    }
    if (*slot && **slot != value) return std::nullopt;
    *slot = static_cast<uint32_t>(value);
  }
  if (dl && *dl != 0) return dl;
  if (mo && *mo != 0) return mo;
  return std::nullopt;
}

// cadmpeg presentation.rs `saved_mode`.
std::optional<std::pair<uint32_t, size_t>> saved_mode(ByteSpan data, size_t at) {
  View view{data, at};
  CANDIDATE(tmpl, text(view));
  std::string_view t = tmpl.value;
  if (!t.starts_with("templatewidth=")) return std::nullopt;
  t.remove_prefix(14);
  size_t split = t.find(",templateheight=");
  if (split == std::string_view::npos) return std::nullopt;
  for (std::string_view v : {t.substr(0, split), t.substr(split + 16)}) {
    auto d = parse_f64(v);
    if (!d || !std::isfinite(*d) || !(*d > 0.0)) return std::nullopt;
  }
  for (int i = 0; i < 4; ++i) {
    CANDIDATE(skip, view.u32());
    (void)skip;
  }
  CANDIDATE(flag, view.u8());
  if (flag != 1) return std::nullopt;
  if (!numbers<9>(view) || !numbers<3>(view) || !numbers<1>(view)) return std::nullopt;
  if (!view.u8() || !view.u8()) return std::nullopt;
  if (!numbers<3>(view) || !numbers<1>(view)) return std::nullopt;
  if (!view.u8()) return std::nullopt;
  if (!text(view)) return std::nullopt;
  CANDIDATE(one, view.u32());
  if (one != 1) return std::nullopt;
  CANDIDATE(mode, view.u32());
  if (mode > 4) return std::nullopt;
  CANDIDATE(one2, view.u32());
  if (one2 != 1) return std::nullopt;
  CANDIDATE(zero, view.u16());
  if (zero != 0) return std::nullopt;
  CANDIDATE(count, view.u16());
  if (count == 0) return std::nullopt;
  return std::pair{mode, view.pos};
}

// cadmpeg presentation.rs `plane`.
std::optional<ReferencePlane> plane(ByteSpan data, size_t at) {
  View view{data, at};
  for (uint32_t expected : {168u, 6u, 0u, 5u}) {
    CANDIDATE(word, view.u32());
    if (word != expected) return std::nullopt;
  }
  CANDIDATE(corners, numbers<15>(view));
  if (corners[0] != corners[12] || corners[1] != corners[13] || corners[2] != corners[14])
    return std::nullopt;
  CANDIDATE(f1, font(view));
  (void)f1;
  CANDIDATE(f2, font(view));
  CANDIDATE(lead, view.u32());
  (void)lead;
  CANDIDATE(cmd, view.u32());
  if (cmd != 18) return std::nullopt;
  CANDIDATE(origin, numbers<3>(view));
  CANDIDATE(skip, numbers<2>(view));
  (void)skip;
  CANDIDATE(label, text(view));
  if (label.value.empty() || label.scalars > 256) return std::nullopt;
  CANDIDATE(one, view.u16());
  if (one != 1) return std::nullopt;
  CANDIDATE(count, view.u32());
  if (count != label.units || count > view.remaining() / 4) return std::nullopt;
  ReferencePlane p;
  p.advances.reserve(count);
  for (uint32_t i = 0; i < count; ++i) {
    CANDIDATE(f, view.f32());
    double advance = static_cast<double>(f);
    if (!std::isfinite(advance) || advance < 0.0) return std::nullopt;
    p.advances.push_back(advance * 1000.0);
  }
  CANDIDATE(one2, view.u32());
  if (one2 != 1) return std::nullopt;
  CANDIDATE(scale, numbers<1>(view));
  if (scale[0] <= 0.0) return std::nullopt;

  std::vector<double> c(corners.begin(), corners.end());
  for (double& v : c) v *= 1000.0;
  std::vector<double> o{origin[0] * 1000.0, origin[1] * 1000.0, origin[2] * 1000.0};
  // AttributeValue::vector rejects non-finite components (e.g. overflow).
  if (!all_finite(c) || !all_finite(o) || !all_finite(p.advances)) return std::nullopt;
  p.label = std::move(label.value);
  for (size_t i = 0; i < 5; ++i) p.corners.push_back(Vec3{c[3 * i], c[3 * i + 1], c[3 * i + 2]});
  p.origin = Vec3{o[0], o[1], o[2]};
  p.font = std::move(f2.first);
  p.point_size = f2.second;
  p.scale = scale[0];
  return p;
}

// cadmpeg presentation.rs `pen_color` (command 9: primary index, fallback
// index, COLORREF; negative primary selects fallback, -2 selects COLORREF).
std::optional<std::pair<PenColor, size_t>> pen_color(ByteSpan data, size_t at,
                                                     std::optional<uint32_t> version_opt) {
  if (!version_opt) return std::nullopt;
  uint32_t version = *version_opt;
  View view{data, std::min(at, data.size())};
  if (at > data.size()) return std::nullopt;
  CANDIDATE(size, view.u32());
  CANDIDATE(cmd, view.u32());
  if (size != 60 || cmd != 9) return std::nullopt;
  CANDIDATE(primary_u, view.u32());
  CANDIDATE(fallback_u, view.u32());
  CANDIDATE(packed, view.u32());
  if (!view.u32()) return std::nullopt;  // unclassified field
  if (version >= 3905) {
    if (!view.u16()) return std::nullopt;
    CANDIDATE(weight, view.f32());
    if (!std::isfinite(weight)) return std::nullopt;
  } else {
    if (!view.u32()) return std::nullopt;
  }
  if (!view.u32()) return std::nullopt;
  if (version >= 3238 && !view.u32()) return std::nullopt;
  if (version >= 6234 && !view.u32()) return std::nullopt;
  int32_t primary = static_cast<int32_t>(primary_u);
  int32_t fallback = static_cast<int32_t>(fallback_u);
  PenColor color;
  if (primary == -2) {
    if (packed > 0x00ffffff) return std::nullopt;
    color.kind = PenColor::Custom;
    color.rgb = colorref(packed);
  } else {
    int32_t index = primary < 0 ? fallback : primary;
    if (index < 0) {
      color.kind = PenColor::Unchanged;
    } else {
      color.kind = PenColor::Indexed;
      color.index = static_cast<uint32_t>(index);
    }
  }
  return std::pair{color, view.pos};
}

// cadmpeg presentation.rs `custom_pen_color` (command 12: COLORREF, R in the
// low byte; serialized length differs from the declared command size).
std::optional<std::pair<std::array<double, 3>, size_t>> custom_pen_color(
    ByteSpan data, size_t at, std::optional<uint32_t> version_opt) {
  if (!version_opt) return std::nullopt;
  uint32_t version = *version_opt;
  if (at > data.size()) return std::nullopt;
  View view{data, at};
  CANDIDATE(size, view.u32());
  if (size < 58 || size > 2106) return std::nullopt;
  CANDIDATE(cmd, view.u32());
  if (cmd != 12) return std::nullopt;
  CANDIDATE(packed, view.u32());
  if (packed > 0x00ffffff) return std::nullopt;
  if (version >= 3905) {
    if (!view.u16()) return std::nullopt;  // standard line-weight selector
    CANDIDATE(weight, view.f32());
    if (!std::isfinite(weight)) return std::nullopt;
  } else {
    if (!view.u32()) return std::nullopt;  // legacy RestoreType=3 line weight
  }
  if (version >= 3160 && !view.u32()) return std::nullopt;  // layer ID
  if (version >= 3238) {
    CANDIDATE(visible, view.u32());
    if (visible > 1) return std::nullopt;
  }
  // MFC CString: one-byte ANSI count or the ff fe ff Unicode marker.
  CANDIDATE(first, view.u8());
  bool unicode = false;
  uint32_t count;
  if (first < 255) {
    count = first;
  } else {
    CANDIDATE(shrt, view.u16());
    if (shrt == 0xfffe) {
      unicode = true;
      CANDIDATE(first2, view.u8());
      if (first2 < 255) {
        count = first2;
      } else {
        CANDIDATE(shrt2, view.u16());
        if (shrt2 == 0xffff) {
          CANDIDATE(far, view.u32());
          count = far;
        } else {
          count = shrt2;
        }
      }
    } else if (shrt == 0xffff) {
      CANDIDATE(far, view.u32());
      count = far;
    } else {
      count = shrt;
    }
  }
  if (count > 1024 || size != 56 + 2 * (count + 1)) return std::nullopt;
  CANDIDATE(name, view.take(static_cast<size_t>(count) * (unicode ? 2 : 1)));
  if (unicode && !decode_utf16(name, count)) return std::nullopt;
  return std::pair{colorref(packed), view.pos};
}

// cadmpeg presentation.rs `append` (one `Contents/DisplayLists` payload).
void append(ByteSpan data, std::optional<uint32_t> version, std::optional<int64_t>& mode_out,
            std::vector<ReferencePlane>& planes) {
  static constexpr std::string_view kEndToken = "uoTempBodyTessData_c";
  auto end_opt = find_from(data, kEndToken, 0, data.size());
  if (!end_opt) return;
  const size_t end = *end_opt;

  static constexpr uint8_t kTemplate[] = {'t', 0, 'e', 0, 'm', 0, 'p', 0, 'l', 0, 'a', 0, 't', 0,
                                          'e', 0, 'w', 0, 'i', 0, 'd', 0, 't', 0, 'h', 0, '=', 0};
  std::optional<std::pair<uint32_t, size_t>> scene;
  const uint8_t* base = data.data();
  for (size_t offset = 0; offset < end;) {
    const void* hit = std::memchr(base + offset, 0xff, end - offset);
    if (!hit) break;
    offset = static_cast<size_t>(static_cast<const uint8_t*>(hit) - base);
    const size_t at = offset++;
    if (data.size() - at < 3 || data[at + 1] != 254 || data[at + 2] != 255) continue;
    if (!starts_with_at(data, at + 4, ByteSpan(kTemplate, sizeof kTemplate))) continue;
    if (auto value = saved_mode(data, at)) {
      if (scene) return;  // ambiguous scene header: nothing is emitted
      scene = value;
    }
  }
  if (!scene) return;
  const auto [mode, start] = *scene;
  if (!mode_out) mode_out = mode;

  ByteSpan head = data.first(end);
  std::optional<uint32_t> color_index;
  std::optional<std::array<double, 3>> custom_color;
  size_t skip_until = start;
  for (size_t offset = start; offset < end; ++offset) {
    if (offset < skip_until) {
      offset = skip_until - 1;
      continue;
    }
    auto word0 = u32_at(data, offset);
    if (!word0) break;  // no further reads can match within the payload
    auto word1 = u32_at(data, offset + 4);
    if (*word0 == 60 && word1 == 9u) {
      if (auto pen = pen_color(head, offset, version)) {
        switch (pen->first.kind) {
          case PenColor::Indexed:
            color_index = pen->first.index;
            custom_color.reset();
            break;
          case PenColor::Custom:
            color_index.reset();
            custom_color = pen->first.rgb;
            break;
          case PenColor::Unchanged:
            break;
        }
        skip_until = pen->second;
      } else {
        color_index.reset();
        custom_color.reset();
      }
      continue;
    }
    if (word1 == 12u && *word0 >= 58 && *word0 <= 2106) {
      // An unsupported custom pen must not inherit the preceding pen.
      color_index.reset();
      custom_color.reset();
      if (auto pen = custom_pen_color(head, offset, version)) {
        custom_color = pen->first;
        skip_until = pen->second;
      }
      continue;
    }
    if (*word0 != 168) continue;
    if (auto p = plane(data, offset)) {
      p->source_offset = offset;
      if (color_index) {
        p->color_index = static_cast<int64_t>(*color_index);
      } else if (custom_color) {
        p->custom = true;
        p->custom_rgb = custom_color;
      }
      planes.push_back(std::move(*p));
    }
  }
}

}  // namespace swglb::presentation

namespace swglb {

// cadmpeg metadata.rs `attributes` (presentation::archive_version once, then
// presentation::append for every section in order).
void read_presentation(const Container& container, Model& model) {
  // cadmpeg presentation.rs `archive_version`: CFB directory paths (including
  // empty storages) or native section/directory names.
  const std::optional<uint32_t> version = presentation::display_version(container.entry_names);
  for (const auto& section : container.sections) {
    if (section.name != std::optional<std::string>("Contents/DisplayLists")) continue;
    size_t first = model.reference_planes.size();
    presentation::append(section.payload, version, model.saved_display_mode,
                         model.reference_planes);
    for (size_t i = first; i < model.reference_planes.size(); ++i) {
      auto& plane = model.reference_planes[i];
      plane.id = "sldprt:metadata:saved_reference_plane#" + std::to_string(section.ordinal) + ":" +
                 std::to_string(plane.source_offset);
    }
  }
}

}  // namespace swglb
