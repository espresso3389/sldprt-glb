// SPDX-License-Identifier: Apache-2.0
// Decode pipeline. Ported from cadmpeg-codec-sldprt decode.rs (`decode`,
// `decode_scan`, `push_display_meshes`) at cadmpeg 5af95421 (Apache-2.0).
#include "engine.h"

#include <algorithm>
#include <set>

#include "errors.h"
#include "stages.h"

namespace swglb {
namespace {

struct DisplayPush {
  std::vector<FaceKeyBinding> face_keys;
  std::vector<PersistentFaceBinding> persistent;
};

DisplayPush push_display_meshes(const Container& container, std::vector<DisplaySection> sections,
                                Model& model, const std::set<uint32_t>& eligible) {
  DisplayPush push;
  for (auto& display : sections) {
    if (display.faces.empty()) continue;
    const Section& section = container.sections[display.section_index];
    ByteSpan payload(section.payload);
    SectionAppearances resolved = resolve_display_appearances(container, display, eligible);
    bool any_references = false;
    for (const auto& face : display.faces) any_references |= !face.surface_references.empty();
    for (size_t index = 0; index < display.faces.size(); ++index) {
      std::string id = "sldprt:displaylist:record#" + std::to_string(section.ordinal) + ":" +
                       std::to_string(index);
      if (auto key = display_face_key(payload, display.faces, index))
        push.face_keys.push_back({id, *key});
      if (any_references) {
        bool ambiguous = false;
        if (auto identity = persistent_join(payload, display.faces, index, &ambiguous))
          push.persistent.push_back({id, *identity});
      }
      if (auto it = resolved.by_face.find(index); it != resolved.by_face.end()) {
        std::string appearance = ensure_display_appearance(model, it->second, section.ordinal);
        AppearanceBinding binding;
        binding.id = "sldprt:appearance:binding#display:" + std::to_string(section.ordinal) + ":" +
                     std::to_string(index);
        binding.target_kind = "tessellation";
        binding.target_id = id;
        binding.appearance = appearance;
        binding.source_entity_id = section.source_stream + "::DisplayFace[" +
                                   std::to_string(index) + "]";
        binding.object_type = "DisplayFace";
        binding.channels["packed_color_hex"] = it->second.packed_hex;
        model.appearance_bindings.push_back(std::move(binding));
      }
      DisplayFace& face = display.faces[index];
      Tessellation mesh;
      mesh.id = std::move(id);
      mesh.strips = std::move(face.strips);
      mesh.has_normals = face.has_normals;
      mesh.section_index = display.section_index;
      mesh.table_index = index;
      mesh.table = face.table;
      mesh.metadata = face.metadata;
      model.tessellations.push_back(std::move(mesh));
    }
  }
  return push;
}

/// cadmpeg `Model::finalize`: every arena is ordered by entity id (byte-wise
/// string order). the reference writer depends on it: tessellation order decides the
/// primitive/colour grouping order, edge order the boundary line order and
/// attribute order the reference-plane node order.
template <class T>
void sort_by_id(std::vector<T>& items) {
  std::stable_sort(items.begin(), items.end(),
                   [](const T& a, const T& b) { return a.id < b.id; });
}

void finalize(Model& model) {
  sort_by_id(model.tessellations);
  sort_by_id(model.appearances);
  sort_by_id(model.appearance_bindings);
  sort_by_id(model.bodies);
  sort_by_id(model.regions);
  sort_by_id(model.shells);
  sort_by_id(model.faces);
  sort_by_id(model.loops);
  sort_by_id(model.coedges);
  sort_by_id(model.edges);
  sort_by_id(model.vertices);
  sort_by_id(model.points);
  sort_by_id(model.surfaces);
  sort_by_id(model.curves);
  sort_by_id(model.configurations);
  sort_by_id(model.occurrences);
  sort_by_id(model.reference_planes);
}

}  // namespace

Model decode_document(const Container& container, const DecodeOptions& options) {
  Model model;
  AssemblyPlanPtr assembly = read_assembly(container);
  std::set<std::string> placed_virtual;
  if (assembly) placed_virtual = virtual_site_keys(*assembly);

  BrepResult brep;
  if (options.brep) brep = decode_brep(container, placed_virtual, model);

  add_brep_face_appearances(brep, model);
  add_document_appearances(container, brep.decoded, model);
  std::vector<DisplaySection> sections = collect_display_sections(container);
  if (brep.decoded) {
    DisplayPush push =
        push_display_meshes(container, std::move(sections), model, brep.eligible_features);
    assign_tessellation_owners(model, brep, push.face_keys, push.persistent);
  } else {
    DisplayPush push = push_display_meshes(container, std::move(sections), model, {});
    assign_tessellation_owners(model, brep, {}, push.persistent);
  }
  apply_visual_state_plan(brep, model);
  read_presentation(container, model);
  if (assembly) apply_assembly(*assembly, container, model);
  finalize(model);
  return model;
}

}  // namespace swglb
