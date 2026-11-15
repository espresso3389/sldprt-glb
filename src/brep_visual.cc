// SPDX-License-Identifier: Apache-2.0
// Ported from cadmpeg-codec-sldprt `appearance/visual_states.rs` and the
// face-colour application of `decode.rs` (cadmpeg 5af95421, Apache-2.0).
//
// Deviation: rank-3 (`moCompFeature_c`) targets need cadmpeg's feature
// history (`feature_faces`: one history feature with the source id, its
// model feature and its input-lane object). They are approximated by the
// faces whose FACE_ID_2001 payload carries the (source, timestamp) pair,
// with no uncertain members.
#include "brep_visual.h"

#include <algorithm>
#include <cstdio>
#include <map>
#include <optional>

#include "display_lists_internal.h"

namespace swglb {
namespace {

// layout.rs `visual_states_feature_appearance_prefix`.
namespace record {
constexpr size_t LEN = 36, FEATURE_SOURCE_ID = 4, FEATURE_TIMESTAMP = 8, SELECTOR_ONE_A = 12,
                 SELECTOR_ONE_B = 16, SELECTOR_TWO = 20, INSTANCE_PREFIX = 24, MARKER = 30,
                 PACKED_COLOR = 32;
constexpr uint8_t INSTANCE_PREFIX_VALUE[6] = {0x07, 0x80, 0x01, 0x00, 0x00, 0x00};
constexpr uint8_t MARKER_VALUE[2] = {0x09, 0x80};
}  // namespace record

// layout.rs `visual_component_body_prefix`.
namespace body_prefix {
constexpr size_t LEN = 17, CLASS_TOKEN = 0, ZERO_PADDING = 2, IDENTITY = 9, IDENTITY_COPY = 13;
}

Color packed_rgb(uint32_t packed) {
  return Color::from_rgba8(packed & 0xff, (packed >> 8) & 0xff, (packed >> 16) & 0xff, 255);
}

using Choice = VisualStateChoice;

std::optional<Color> color_of(const Choice& c) {
  return c.definition ? std::optional<Color>(c.definition->color) : std::nullopt;
}

void merge(std::map<std::string, Choice>& choices, const std::string& face, const Choice& incoming) {
  auto it = choices.find(face);
  if (it == choices.end()) {
    choices.emplace(face, incoming);
    return;
  }
  Choice& previous = it->second;
  if (incoming.rank > previous.rank) previous = incoming;
  else if (incoming.rank == previous.rank && color_of(previous) != color_of(incoming))
    previous.definition.reset();
}

/// framed_string: `ff fe ff`, u8 count, UTF-16LE units.
std::optional<std::u32string> framed_string(ByteSpan bytes, size_t at) {
  if (at + 4 > bytes.size() || bytes[at] != 0xff || bytes[at + 1] != 0xfe || bytes[at + 2] != 0xff)
    return std::nullopt;
  size_t count = bytes[at + 3];
  if (at + 4 + count * 2 > bytes.size()) return std::nullopt;
  return decode_utf16le_strict(bytes.subspan(at + 4, count * 2), count);
}

/// cadmpeg `selected_bodies`.
std::optional<std::set<std::string>> selected_bodies(ByteSpan bytes,
                                                     const std::vector<ClassInterval>& classes,
                                                     const Model& model) {
  std::vector<std::string> keys;
  for (const auto& c : classes) {
    if (c.name != "visStateKeyData_c") continue;
    ByteSpan part = bytes.subspan(c.content.start, c.content.end - c.content.start);
    for (size_t at = 0; at + 3 <= part.size(); ++at) {
      if (part[at] != 0xff || part[at + 1] != 0xfe || part[at + 2] != 0xff) continue;
      if (auto key = framed_string(part, at); key && !key->empty()) keys.push_back(to_utf8(*key));
    }
  }
  if (keys.size() != 1) return std::nullopt;
  const std::string& key = keys[0];
  size_t us = key.find('_');
  if (us == std::string::npos || us == 0 || us + 1 >= key.size()) return std::nullopt;
  const Configuration* match = nullptr;
  size_t matches = 0;
  for (const auto& cfg : model.configurations) {
    if (!cfg.name) continue;
    bool hit = false;
    for (const std::string& prefix : {*cfg.name + "_", "<" + *cfg.name + ">_"})
      hit |= key.size() > prefix.size() && key.compare(0, prefix.size(), prefix) == 0;
    if (hit) ++matches, match = &cfg;
  }
  if (matches != 1 || !match->active) return std::nullopt;
  // `bodies: None` (unresolved membership) yields no scope; an empty list
  // is an empty scope. The model keeps no distinction, so an empty body
  // list is treated as unresolved.
  if (match->bodies.empty()) return std::nullopt;
  return std::set<std::string>(match->bodies.begin(), match->bodies.end());
}

std::set<std::string> faces_of_bodies(const Model& model, const std::set<std::string>& bodies) {
  std::set<std::string> regions, shells, out;
  for (const auto& b : model.bodies)
    if (bodies.count(b.id)) regions.insert(b.regions.begin(), b.regions.end());
  for (const auto& r : model.regions)
    if (regions.count(r.id)) shells.insert(r.shells.begin(), r.shells.end());
  for (const auto& f : model.faces)
    if (shells.count(f.shell)) out.insert(f.id);
  return out;
}

std::optional<uint32_t> body_identity(ByteSpan bytes) {
  if (bytes.size() < body_prefix::LEN) return std::nullopt;
  uint16_t token = *u16_at(bytes, body_prefix::CLASS_TOKEN);
  uint32_t identity = *u32_at(bytes, body_prefix::IDENTITY);
  for (size_t i = 0; i < 7; ++i)
    if (bytes[body_prefix::ZERO_PADDING + i] != 0) return std::nullopt;
  if (token <= 0x8000 || token == 0xffff || identity == 0 || identity == UINT32_MAX ||
      u32_at(bytes, body_prefix::IDENTITY_COPY) != identity)
    return std::nullopt;
  return identity;
}

std::optional<DisplayAppearance> record_definition(ByteSpan bytes, size_t at) {
  if (at + record::LEN > bytes.size()) return std::nullopt;
  ByteSpan v = bytes.subspan(at, record::LEN);
  if (u32_at(v, record::SELECTOR_ONE_A) != 1u || u32_at(v, record::SELECTOR_ONE_B) != 1u ||
      u32_at(v, record::SELECTOR_TWO) != 2u ||
      !std::equal(v.begin() + record::INSTANCE_PREFIX, v.begin() + record::MARKER,
                  record::INSTANCE_PREFIX_VALUE) ||
      !std::equal(v.begin() + record::MARKER, v.begin() + record::PACKED_COLOR,
                  record::MARKER_VALUE))
    return std::nullopt;
  uint32_t packed = *u32_at(v, record::PACKED_COLOR);
  if (packed >> 24 != 0) return std::nullopt;
  DisplayAppearance d;
  d.name = "SolidWorks scoped appearance";
  d.color = packed_rgb(packed);
  char hex[9];
  std::snprintf(hex, sizeof hex, "%08x", packed);
  d.packed_hex = hex;
  d.record_offset = at;
  return d;
}

}  // namespace

std::set<uint32_t> resolve_visual_states(const Container& container, const NativePayloads& payloads,
                                         const Model& model, std::vector<VisualStateChoice>& out,
                                         std::vector<UnresolvedAppearance>& unresolved) {
  std::set<uint32_t> feature_sources;
  std::map<std::string, Choice> plan;
  std::set<std::pair<uint32_t, uint32_t>> eligible;
  for (const auto& [face, v] : payloads.faces)
    if (v.size() > 2) eligible.emplace(v[1], v[2]);
  for (const auto& section : container.sections) {
    if (!section.name || (*section.name != "ThirdPtyStore/VisualStates" &&
                          *section.name != "ThirdPtyStore/VisualStates__ZLB"))
      continue;
    ByteSpan bytes(section.payload);
    auto classes = class_intervals(bytes);
    auto bodies = selected_bodies(bytes, classes, model);
    if (!bodies) continue;
    std::set<std::string> scope = faces_of_bodies(model, *bodies);
    auto merge_scope = [&](const std::set<std::string>& targets, uint8_t rank,
                           std::optional<DisplayAppearance> definition) {
      Choice c;
      c.rank = rank;
      c.ordinal = section.ordinal;
      c.source_name = section.source_stream;
      c.definition = std::move(definition);
      for (const auto& f : targets)
        if (scope.count(f)) merge(plan, f, c);
    };
    for (size_t index = 0; index < classes.size(); ++index) {
      const ClassInterval& cls = classes[index];
      if (cls.name == "moVisualProperties_c") {
        std::vector<std::string_view> previous;
        for (size_t k = index; k > 0 && previous.size() < 4; --k)
          previous.push_back(classes[k - 1].name);
        auto starts = [&](std::initializer_list<std::string_view> want) {
          if (previous.size() < want.size()) return false;
          return std::equal(want.begin(), want.end(), previous.begin());
        };
        bool chain =
            starts({"visStateVisualPropertiesAttribute_c", "moUnitComponent_c", "moCompHolder_c"}) ||
            starts({"visStateVisualStateAttribute_c", "visStateVisualPropertiesAttribute_c",
                    "moUnitComponent_c", "moCompHolder_c"}) ||
            starts({"moCompHolder_c", "visStateVisualStateAttribute_c",
                    "visStateVisualPropertiesAttribute_c", "moUnitComponent_c"});
        const ClassInterval* attribute = nullptr;
        for (size_t k = index; k > 0; --k)
          if (classes[k - 1].name == "visStateVisualPropertiesAttribute_c") {
            attribute = &classes[k - 1];
            break;
          }
        bool attribute_valid = false;
        if (attribute && attribute->content.end <= bytes.size()) {
          ByteSpan p = bytes.subspan(attribute->content.start,
                                     attribute->content.end - attribute->content.start);
          static constexpr uint8_t a4[4] = {1, 0, 0, 0}, a6[6] = {1, 0, 0, 0, 0, 0};
          attribute_valid = (p.size() == 4 && std::equal(p.begin(), p.end(), a4)) ||
                            (p.size() == 6 && std::equal(p.begin(), p.end(), a6));
        }
        if (chain && attribute_valid)
          if (auto d = appearance_definition_at(bytes, cls.content.start, cls.class_offset))
            merge_scope(scope, 1, std::move(d));
      }
      uint8_t rank;
      if (cls.name == "moCompHolder_c") rank = 1;
      else if (cls.name == "moCompSolidBody_c") rank = 2;
      else if (cls.name == "moCompFeature_c") rank = 3;
      else if (cls.name == "moCompFace_c" ||
               (cls.name.size() >= 11 && cls.name.compare(cls.name.size() - 11, 11, "SurfIdRep_c") == 0))
        rank = 4;
      else continue;
      if (cls.content.end < record::LEN - 1) continue;
      size_t last = cls.content.end - (record::LEN - 1);
      for (size_t at = cls.content.start; at < last; ++at) {
        auto definition = record_definition(bytes, at);
        if (!definition) continue;
        std::optional<std::set<std::string>> targets;
        if (rank == 1) {
          targets = scope;
        } else if (rank == 2) {
          if (auto value = body_identity(bytes.subspan(cls.content.start, cls.content.end - cls.content.start))) {
            std::set<std::string> candidates;
            for (const auto& [body, v] : payloads.bodies)
              if (v.size() == 1 && v[0] == *value && bodies->count(body)) candidates.insert(body);
            if (candidates.size() == 1) targets = faces_of_bodies(model, candidates);
          }
        } else if (rank == 3) {
          uint32_t source = *u32_at(bytes, at + record::FEATURE_SOURCE_ID);
          uint32_t timestamp = *u32_at(bytes, at + record::FEATURE_TIMESTAMP);
          if (eligible.count({source, timestamp})) {
            std::set<std::string> faces;
            for (const auto& [face, v] : payloads.faces)
              if (v.size() > 2 && v[1] == source && v[2] == timestamp) faces.insert(face);
            if (!faces.empty()) {
              if (source != 0 && source != UINT32_MAX) feature_sources.insert(source);
              targets = std::move(faces);
            }
          }
        }
        if (targets) {
          merge_scope(*targets, rank, definition);
        } else {
          UnresolvedAppearance u;
          u.owner_class = cls.name;
          u.identity = rank == 2   ? std::string("body")
                       : rank == 3 ? std::to_string(*u32_at(bytes, at + record::FEATURE_SOURCE_ID))
                                   : std::string("face");
          u.reason = "appearance target does not resolve to in-scope faces";
          u.source_offset = at;
          u.packed_hex = definition->packed_hex;
          unresolved.push_back(std::move(u));
          merge_scope(scope, rank, std::nullopt);
        }
      }
    }
  }
  for (auto& [id, choice] : plan) {
    choice.face = id;
    out.push_back(std::move(choice));
  }
  return feature_sources;
}

}  // namespace swglb
