// SPDX-License-Identifier: Apache-2.0
// Display-face and document appearances. Ported from cadmpeg-codec-sldprt
// appearance.rs (with display-colour and appearance-ownership
// extensions) and decode.rs (`ensure_display_appearance`, the
// `moVisualProperties_c` material appearances of `build_geometry_ir`),
// cadmpeg 5af95421, Apache-2.0. Annotations, loss notes and unresolved
// diagnostics are not ported.
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <set>
#include <map>

#include "display_lists_internal.h"
#include "errors.h"

namespace swglb {
namespace {

constexpr std::string_view kVisualPropertiesClass = "moVisualProperties_c";

// layout.rs `display_lists_inline_visual_properties_prefix`.
constexpr size_t kInlinePackedColor = 2;

// layout.rs `visual_states_feature_appearance_prefix`.
namespace feature_visual {
constexpr size_t LEN = 36, FEATURE_SOURCE_ID = 4, FEATURE_TIMESTAMP = 8, SELECTOR_ONE_A = 12,
                 SELECTOR_ONE_B = 16, SELECTOR_TWO = 20, INSTANCE_PREFIX = 24, MARKER = 30,
                 PACKED_COLOR = 32;
constexpr uint32_t SELECTOR_ONE_A_VALUE = 1, SELECTOR_ONE_B_VALUE = 1, SELECTOR_TWO_VALUE = 2;
constexpr uint8_t INSTANCE_PREFIX_VALUE[6] = {0x07, 0x80, 0x01, 0x00, 0x00, 0x00};
constexpr uint8_t MARKER_VALUE[2] = {0x09, 0x80};
}  // namespace feature_visual

// appearance.rs `packed_rgb`.
Color packed_rgb(uint32_t packed) {
  return Color::from_rgba8(packed & 0xff, (packed >> 8) & 0xff, (packed >> 16) & 0xff, 255);
}

std::string hex8(uint32_t v) {
  char buf[9];
  std::snprintf(buf, sizeof buf, "%08x", v);
  return buf;
}

// appearance.rs `definition_at`.
std::optional<DisplayAppearance> definition_at(ByteSpan bytes, size_t packed_offset,
                                               size_t record_offset) {
  auto packed = u32_at(bytes, packed_offset);
  if (!packed) return std::nullopt;
  size_t name_header = packed_offset + 16;
  static constexpr uint8_t kMarker[3] = {0xff, 0xfe, 0xff};
  if (!starts_with_at(bytes, name_header, ByteSpan(kMarker, 3))) return std::nullopt;
  auto count = u8_at(bytes, name_header + 3);
  if (!count) return std::nullopt;
  size_t start = name_header + 4;
  auto raw = slice(bytes, start, start + size_t{*count} * 2);
  if (!raw) return std::nullopt;
  auto decoded = decode_utf16le_strict(*raw, *count);
  if (!decoded) return std::nullopt;
  auto trimmed = trim_unicode(*decoded);
  DisplayAppearance d;
  d.name = trimmed.empty() ? std::string("SolidWorks appearance") : to_utf8(trimmed);
  d.color = packed_rgb(*packed);
  d.packed_hex = hex8(*packed);
  d.record_offset = record_offset;
  return d;
}

// appearance.rs `inline_definitions`.
std::vector<DisplayAppearance> inline_definitions(ByteSpan data, size_t start, size_t end) {
  std::vector<DisplayAppearance> out;
  if (start > end || end > data.size()) return out;
  for (size_t at = start; at + 2 <= end; ++at) {
    // tag = u16 at `at`; high byte must be 0x80, low byte neither 0 nor 0xff.
    if (data[at + 1] != 0x80) continue;
    uint8_t index = data[at];
    if (index == 0 || index == 0xff) continue;
    if (auto d = definition_at(data, at + kInlinePackedColor, at)) out.push_back(std::move(*d));
  }
  return out;
}

// appearance.rs `body_property_slot`.
std::optional<size_t> body_property_slot(ByteSpan data, size_t start) {
  if (u16_at(data, start) != uint16_t{0} || u32_at(data, start + 2) != 1u ||
      u32_at(data, start + 6) != 6u)
    return std::nullopt;
  for (size_t i = 0; i < 3; ++i)
    if (!u32_at(data, start + 10 + 4 * i)) return std::nullopt;
  auto text = read_framed_string(data, start + 22);
  if (!text) return std::nullopt;
  return text->end;
}

// appearance.rs `display_component_headers`: (tag offset, offset after name).
std::vector<std::pair<size_t, size_t>> display_component_headers(
    ByteSpan data, const std::vector<DisplayFace>& faces) {
  std::vector<std::pair<size_t, size_t>> headers;
  if (!contains(data, "uoTempAssemblySHDData_c")) return headers;
  const uint8_t* base = data.data();
  size_t start = 0;
  for (size_t index = 0; index <= faces.size(); ++index) {
    size_t end = index < faces.size() ? faces[index].table.start : data.size();
    for (size_t at = start; at < end && at < data.size();) {
      const void* hit = std::memchr(base + at, 0xff, std::min(end, data.size()) - at);
      if (!hit) break;
      at = static_cast<size_t>(static_cast<const uint8_t*>(hit) - base);
      size_t here = at++;
      if (here < 2 || !starts_with_at(data, here, std::string_view("\xff\xfe\xff", 3)) ||
          !(*u16_at(data, here - 2) & 0x8000))
        continue;
      auto name = read_framed_string(data, here);
      if (!name) continue;
      bool has_at = name->value.find(U'@') != std::u32string::npos;
      auto next = u8_at(data, name->end);
      if (has_at && next && (*next == 0 || *next == 1)) headers.emplace_back(here - 2, name->end);
    }
    start = index < faces.size() ? faces[index].table.end : end;
  }
  return headers;
}

enum class TargetKind { Body, Face, Component };
struct Assignment {
  TargetKind kind;
  size_t first = 0, last = 0;  ///< Face index range [first, last) (Face: first only).
  DisplayAppearance definition;
};

/// Faces are sorted by table start and do not overlap, so table ends are
/// ordered too; [first, last) of the faces whose table lies in a window.
std::pair<size_t, size_t> face_window(const std::vector<DisplayFace>& faces, size_t min_start,
                                      bool strict_start, size_t max_end) {
  auto it = std::lower_bound(faces.begin(), faces.end(), min_start,
                             [&](const DisplayFace& f, size_t v) {
                               return strict_start ? f.table.start <= v : f.table.start < v;
                             });
  size_t first = static_cast<size_t>(it - faces.begin()), last = first;
  while (last < faces.size() && faces[last].table.end <= max_end) ++last;
  return {first, last};
}

/// Smallest value in sorted `v` that is >= x (strict: > x), or nullopt.
std::optional<size_t> next_at_or_after(const std::vector<size_t>& v, size_t x, bool strict) {
  auto it = strict ? std::upper_bound(v.begin(), v.end(), x) : std::lower_bound(v.begin(), v.end(), x);
  if (it == v.end()) return std::nullopt;
  return *it;
}

// appearance.rs `display_assignments`. Boundary searches use sorted offset
// vectors instead of rescanning every class/body/header per face.
std::vector<Assignment> display_assignments(ByteSpan data, const std::vector<DisplayFace>& faces) {
  std::vector<ClassInterval> classes = class_intervals(data);
  auto headers = display_component_headers(data, faces);
  std::vector<std::pair<size_t, size_t>> bodies;
  // The body-slot tags found in face metadata do not depend on the
  // `uoBodyPropInfo_c` class being examined (its 10-byte opening is fixed),
  // so they are collected once.
  bool tags_done = false;
  std::map<uint16_t, std::vector<std::pair<size_t, size_t>>> tags;
  static constexpr uint8_t kPrefix[10] = {0, 0, 1, 0, 0, 0, 6, 0, 0, 0};
  for (const auto& c : classes) {
    if (c.name != "uoBodyPropInfo_c") continue;
    bodies.emplace_back(c.class_offset, c.content.start);
    if (u16_at(data, c.content.start) != uint16_t{0} || u32_at(data, c.content.start + 2) != 1u ||
        u32_at(data, c.content.start + 6) != 6u)
      continue;
    if (!tags_done) {
      tags_done = true;
      for (const auto& face : faces) {
        size_t stop = face.metadata.end >= 12 ? face.metadata.end - 12 : 0;
        for (size_t at = face.metadata.start; at < stop; ++at) {
          if (!(data[at + 1] & 0x80)) continue;
          uint16_t tag = *u16_at(data, at);
          if (tag == 0xffff || std::memcmp(data.data() + at + 2, kPrefix, 10) != 0) continue;
          if (body_property_slot(data, at + 2)) tags[tag].emplace_back(at, at + 2);
        }
      }
    }
    if (tags.size() == 1)
      for (const auto& p : tags.begin()->second) bodies.push_back(p);
  }
  std::sort(bodies.begin(), bodies.end());
  bodies.erase(std::unique(bodies.begin(), bodies.end()), bodies.end());

  std::vector<Assignment> assignments;
  auto root = std::find_if(classes.begin(), classes.end(),
                           [](const ClassInterval& c) { return c.name == "uoTempAssemblySHDData_c"; });
  if (root != classes.end()) {
    const ClassInterval* only = nullptr;
    size_t defaults = 0;
    for (const auto& c : classes)
      if (c.name == kVisualPropertiesClass && c.class_offset < root->class_offset) {
        ++defaults;
        only = &c;
      }
    if (defaults == 1)
      if (auto d = definition_at(data, only->content.start, only->class_offset))
        assignments.push_back({TargetKind::Body, 0, faces.size(), std::move(*d)});
  }

  std::vector<size_t> header_offsets, body_offsets;
  for (const auto& h : headers) header_offsets.push_back(h.first);
  for (const auto& b : bodies) body_offsets.push_back(b.first);
  std::vector<size_t> face_bounds;  // class, body and header offsets
  for (const auto& c : classes) face_bounds.push_back(c.class_offset);
  face_bounds.insert(face_bounds.end(), body_offsets.begin(), body_offsets.end());
  face_bounds.insert(face_bounds.end(), header_offsets.begin(), header_offsets.end());
  std::sort(face_bounds.begin(), face_bounds.end());
  for (size_t i = 0; i < faces.size(); ++i) {
    const auto& face = faces[i];
    size_t end = face.metadata.end;
    if (auto b = next_at_or_after(face_bounds, face.metadata.start, false)) end = std::min(end, *b);
    auto definitions = inline_definitions(data, face.metadata.start, end);
    if (definitions.size() == 1)
      assignments.push_back({TargetKind::Face, i, i + 1, std::move(definitions[0])});
  }

  std::vector<size_t> body_bounds;  // face table starts, header and body offsets
  for (const auto& f : faces) body_bounds.push_back(f.table.start);
  body_bounds.insert(body_bounds.end(), header_offsets.begin(), header_offsets.end());
  body_bounds.insert(body_bounds.end(), body_offsets.begin(), body_offsets.end());
  std::sort(body_bounds.begin(), body_bounds.end());
  size_t previous = 0;
  for (const auto& [offset, content] : bodies) {
    size_t start = body_property_slot(data, content).value_or(content);
    size_t end = next_at_or_after(body_bounds, content, true).value_or(data.size());
    auto definitions = inline_definitions(data, start, end);
    size_t lower = 0;
    auto h = std::lower_bound(header_offsets.begin(), header_offsets.end(), offset);
    if (h != header_offsets.begin()) lower = *std::prev(h);
    lower = std::max(lower, previous);
    auto [first, last] = face_window(faces, lower, false, offset);
    if (definitions.size() == 1 && first < last)
      assignments.push_back({TargetKind::Body, first, last, std::move(definitions[0])});
    previous = offset;
  }

  for (size_t index = 0; index < headers.size(); ++index) {
    auto [offset, content] = headers[index];
    size_t end = index + 1 < headers.size() ? headers[index + 1].first : data.size();
    auto [first, last] = face_window(faces, offset, true, end);
    if (first >= last) continue;
    auto definitions = inline_definitions(data, content, faces[first].table.start);
    if (definitions.size() == 1)
      assignments.push_back({TargetKind::Component, first, last, std::move(definitions[0])});
  }
  return assignments;
}

struct FeatureAppearanceAssignment {
  uint32_t feature_source_id = 0;
  uint32_t feature_timestamp = 0;
  uint32_t packed_color = 0;
  size_t record_offset = 0;
};

// appearance.rs `feature_assignments`.
std::vector<FeatureAppearanceAssignment> feature_assignments(const Container& container) {
  std::vector<FeatureAppearanceAssignment> out;
  for (const auto& section : container.sections) {
    if (!section.name ||
        (*section.name != "ThirdPtyStore/VisualStates" &&
         *section.name != "ThirdPtyStore/VisualStates__ZLB"))
      continue;
    ByteSpan bytes(section.payload);
    auto classes = class_intervals(bytes);
    for (size_t marker : find_all(bytes, ByteSpan(feature_visual::MARKER_VALUE, 2))) {
      if (marker < feature_visual::MARKER) continue;
      size_t record_offset = marker - feature_visual::MARKER;
      auto record = slice(bytes, record_offset, record_offset + feature_visual::LEN);
      if (!record) continue;
      bool in_feature = std::any_of(classes.begin(), classes.end(), [&](const ClassInterval& c) {
        return c.name == "moCompFeature_c" && c.content.start <= record_offset &&
               record_offset + feature_visual::LEN <= c.content.end;
      });
      if (!in_feature ||
          u32_at(*record, feature_visual::SELECTOR_ONE_A) != feature_visual::SELECTOR_ONE_A_VALUE ||
          u32_at(*record, feature_visual::SELECTOR_ONE_B) != feature_visual::SELECTOR_ONE_B_VALUE ||
          u32_at(*record, feature_visual::SELECTOR_TWO) != feature_visual::SELECTOR_TWO_VALUE ||
          !starts_with_at(*record, feature_visual::INSTANCE_PREFIX,
                          ByteSpan(feature_visual::INSTANCE_PREFIX_VALUE, 6)))
        continue;
      uint32_t source = *u32_at(*record, feature_visual::FEATURE_SOURCE_ID);
      if (source == 0 || source == UINT32_MAX) continue;  // FeatureSourceId
      uint32_t timestamp = *u32_at(*record, feature_visual::FEATURE_TIMESTAMP);
      if (timestamp == 0 || timestamp == UINT32_MAX) continue;
      uint32_t packed = *u32_at(*record, feature_visual::PACKED_COLOR);
      if (packed >> 24 != 0) continue;
      out.push_back({source, timestamp, packed, record_offset});
    }
  }
  return out;
}

}  // namespace

// appearance.rs `resolve_display_appearances`: body, feature, face, then
// component precedence. Conflicting feature records stay unresolved (cadmpeg
// reports them as `unresolved`; they are dropped here).
SectionAppearances resolve_display_appearances(const Container& container,
                                               const DisplaySection& display,
                                               const std::set<uint32_t>& eligible_features) {
  const Section& section = container.sections[display.section_index];
  ByteSpan data(section.payload);
  const auto& faces = display.faces;
  auto native = display_assignments(data, faces);
  SectionAppearances out;
  for (const auto& a : native)
    if (a.kind == TargetKind::Body)
      for (size_t i = a.first; i < a.last; ++i) out.by_face[i] = a.definition;

  if (!eligible_features.empty()) {
    std::map<uint32_t, std::vector<size_t>> faces_by_source;
    for (size_t i = 0; i < faces.size(); ++i)
      if (auto source = face_feature_source_id(faces[i])) faces_by_source[*source].push_back(i);
    if (!faces_by_source.empty()) {
      std::map<uint32_t, std::vector<FeatureAppearanceAssignment>> feature_by_source;
      for (auto& a : feature_assignments(container))
        if (eligible_features.count(a.feature_source_id))
          feature_by_source[a.feature_source_id].push_back(a);
      for (const auto& [source, indexes] : faces_by_source) {
        auto it = feature_by_source.find(source);
        if (it == feature_by_source.end() || it->second.empty()) continue;
        const auto& first = it->second.front();
        bool agreed = std::all_of(it->second.begin(), it->second.end(), [&](const auto& c) {
          return c.feature_timestamp == first.feature_timestamp &&
                 c.packed_color == first.packed_color;
        });
        if (!agreed) continue;
        out.matched_feature_sources.insert(source);
        DisplayAppearance d;
        d.name = "SolidWorks feature appearance";
        d.color = packed_rgb(first.packed_color);
        d.packed_hex = hex8(first.packed_color);
        d.record_offset = first.record_offset;
        for (size_t i : indexes) out.by_face[i] = d;
      }
    }
  }
  for (const auto& a : native)
    if (a.kind == TargetKind::Face) out.by_face[a.first] = a.definition;
  for (const auto& a : native)
    if (a.kind == TargetKind::Component)
      for (size_t i = a.first; i < a.last; ++i) out.by_face[i] = a.definition;
  return out;
}

// decode.rs `ensure_display_appearance` (ScopedAppearanceOrigin::DisplayList).
std::string ensure_display_appearance(Model& model, const DisplayAppearance& definition,
                                      size_t section_ordinal) {
  for (const auto& a : model.appearances)
    if (a.base_color && *a.base_color == definition.color && a.name &&
        *a.name == definition.name)
      return a.id;
  Appearance appearance;
  appearance.id = "sldprt:appearance:displaylist#" + std::to_string(section_ordinal) + ":" +
                  std::to_string(definition.record_offset);
  appearance.name = definition.name;
  appearance.schema = "moVisualProperties_c";
  appearance.base_color = definition.color;
  model.appearances.push_back(appearance);
  return appearance.id;
}

std::optional<DisplayAppearance> appearance_definition_at(ByteSpan bytes, size_t packed_offset,
                                                          size_t record_offset) {
  return definition_at(bytes, packed_offset, record_offset);
}

// decode.rs `ensure_display_appearance` (ScopedAppearanceOrigin::VisualState).
std::string ensure_visual_state_appearance(Model& model, const DisplayAppearance& definition,
                                           size_t section_ordinal) {
  for (const auto& a : model.appearances)
    if (a.base_color && *a.base_color == definition.color && a.name &&
        *a.name == definition.name)
      return a.id;
  Appearance appearance;
  appearance.id = "sldprt:appearance:visualstate#" + std::to_string(section_ordinal) + ":" +
                  std::to_string(definition.record_offset);
  appearance.name = definition.name;
  appearance.schema = "SolidWorks.VisualStates";
  appearance.base_color = definition.color;
  model.appearances.push_back(appearance);
  return appearance.id;
}

// decode.rs: entity-53 face colours, before the material definitions.
void add_brep_face_appearances(const BrepResult& brep, Model& model) {
  std::set<std::string> appearances, bindings;
  for (const auto& a : model.appearances) appearances.insert(a.id);
  for (const auto& b : model.appearance_bindings) bindings.insert(b.id);
  for (const auto& fc : brep.face_colors) {
    std::optional<std::string> site = fc.site;
    if (!site && fc.target)
      if (size_t at = fc.target->find('@'); at != std::string::npos) site = fc.target->substr(at + 1);
    std::string tail = site ? "@" + *site : std::string();
    std::string id = "sldprt:appearance:entity53#" + std::to_string(fc.color_attr) + tail;
    if (appearances.insert(id).second) {
      Appearance appearance;
      appearance.id = id;
      appearance.schema = "entity-53";
      appearance.base_color = fc.color;
      model.appearances.push_back(std::move(appearance));
    }
    if (!fc.target) continue;
    std::string binding_id = "sldprt:appearance:binding#face:" + std::to_string(fc.face_attr) + ":" +
                             std::to_string(fc.color_attr) + tail;
    if (!bindings.insert(binding_id).second) continue;
    AppearanceBinding binding;
    binding.id = binding_id;
    binding.target_kind = "face";
    binding.target_id = *fc.target;
    binding.appearance = id;
    binding.source_entity_id = std::to_string(fc.face_attr);
    binding.object_type = "Face";
    model.appearance_bindings.push_back(std::move(binding));
  }
}

// decode.rs: the native appearance plan after tessellation ownership.
void apply_visual_state_plan(const BrepResult& brep, Model& model) {
  // decode.rs `retain_unresolved_appearance` for the plan's unresolved details.
  for (const auto& u : brep.visual_unresolved) {
    if (!u.packed_hex) continue;
    uint32_t packed = static_cast<uint32_t>(std::stoul(*u.packed_hex, nullptr, 16));
    std::string key = std::to_string(u.appearance_index) + ":" + std::to_string(u.source_offset);
    std::string id = "sldprt:appearance:unresolved#" + key;
    if (std::none_of(model.appearances.begin(), model.appearances.end(),
                     [&](const Appearance& a) { return a.id == id; })) {
      Appearance appearance;
      appearance.id = id;
      appearance.name = "Unresolved appearance";
      appearance.schema = "unresolved";
      appearance.base_color = packed_rgb(packed);
      model.appearances.push_back(std::move(appearance));
    }
    std::string binding_id = "sldprt:appearance:unresolved-binding#" + key;
    if (std::any_of(model.appearance_bindings.begin(), model.appearance_bindings.end(),
                    [&](const AppearanceBinding& b) { return b.id == binding_id; }))
      continue;
    AppearanceBinding binding;
    binding.id = binding_id;
    binding.target_kind = "source";
    binding.target_id = u.identity;
    binding.appearance = id;
    binding.source_entity_id = u.identity;
    binding.object_type = u.owner_class;
    binding.channels["owner_class"] = u.owner_class;
    binding.channels["identity"] = u.identity;
    binding.channels["appearance_index"] = std::to_string(u.appearance_index);
    binding.channels["reason"] = u.reason;
    binding.channels["source_offset"] = std::to_string(u.source_offset);
    binding.channels["packed_color_hex"] = *u.packed_hex;
    model.appearance_bindings.push_back(std::move(binding));
  }
  std::map<std::string, size_t> face_at;
  for (size_t i = 0; i < model.faces.size(); ++i) face_at.emplace(model.faces[i].id, i);
  for (size_t face_index = 0; face_index < brep.visual_plan.size(); ++face_index) {
    const VisualStateChoice& choice = brep.visual_plan[face_index];
    if (!choice.definition) continue;
    auto it = face_at.find(choice.face);
    if (it == face_at.end()) continue;
    const DisplayAppearance& definition = *choice.definition;
    {
      const Face& face = model.faces[it->second];
      if (face.color && choice.rank < 4) continue;
      if (face.color && *face.color != definition.color && choice.rank == 4) continue;
    }
    std::string appearance = ensure_visual_state_appearance(model, definition, choice.ordinal);
    model.faces[it->second].color = definition.color;
    AppearanceBinding binding;
    binding.id = "sldprt:appearance:native-binding#" + std::to_string(choice.ordinal) + ":" +
                 std::to_string(definition.record_offset) + ":" + std::to_string(face_index);
    binding.target_kind = "face";
    binding.target_id = choice.face;
    binding.appearance = appearance;
    binding.source_entity_id = choice.source_name + ":" + std::to_string(definition.record_offset);
    binding.object_type = "Native scoped face";
    binding.channels["packed_color_hex"] = definition.packed_hex;
    model.appearance_bindings.push_back(std::move(binding));
  }
}

// decode.rs `build_geometry_ir`: the `appearance::definitions` material
// appearances (`sldprt:appearance:material#<index>`). `build_metadata_ir`
// adds none. The entity-53 face colours (and their face bindings) belong to
// the B-rep stage; native visual-state appearances follow the display meshes.
void add_document_appearances(const Container& container, bool with_brep, Model& model) {
  if (!with_brep) return;
  size_t index = 0;
  for (const auto& section : container.sections) {
    ByteSpan bytes(section.payload);
    // appearance.rs `definitions`.
    for (size_t offset : find_all(bytes, kVisualPropertiesClass)) {
      auto d = definition_at(bytes, offset + kVisualPropertiesClass.size(), offset);
      if (!d) continue;
      Appearance appearance;
      appearance.id = "sldprt:appearance:material#" + std::to_string(index++);
      appearance.name = d->name;
      appearance.schema = "moVisualProperties_c";
      appearance.base_color = d->color;
      model.appearances.push_back(std::move(appearance));
    }
  }
}

}  // namespace swglb
