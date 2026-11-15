// SPDX-License-Identifier: Apache-2.0
// Engine-facing entry points of each decode stage. Each stage owns its own
// .cc file (and may add an internal header); these signatures are the
// contract used by engine.cc. See docs/DESIGN.md.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "container.h"
#include "model.h"

namespace swglb {

// ---------------------------------------------------------------------------
// display_lists.cc — `uoTempFaceTessData_c` descriptor tables
// (cadmpeg tessellation.rs: section_display_faces, class_intervals,
// display_face_key, persistent_surface_references, persistent_join_verdict).
// ---------------------------------------------------------------------------

/// Persistent B-rep face identity (cadmpeg brep.rs `PersistentFaceIdentity`).
struct PersistentFaceIdentity {
  uint32_t feature_source_id = 0;
  uint32_t local_id = 0;
  std::vector<uint32_t> trailing_fields;
  auto operator<=>(const PersistentFaceIdentity&) const = default;
};

/// One decoded descriptor table. Implementations may extend this struct.
struct DisplayFace {
  std::vector<std::vector<MeshVertex>> strips;
  bool has_normals = true;
  ByteRange table;
  ByteRange metadata;
  struct SurfaceReference;  // defined in display_lists.cc
  std::vector<std::shared_ptr<const SurfaceReference>> surface_references;
  /// feature_source_id of each surface reference (same order).
  std::vector<uint32_t> surface_reference_features;
};

struct DisplaySection {
  size_t section_index = 0;  ///< Index into Container::sections.
  std::vector<DisplayFace> faces;
};

/// Every section with at least one display face, in section order
/// (cadmpeg decode.rs `collect_display_sections`).
std::vector<DisplaySection> collect_display_sections(const Container& container);

/// Face key at metadata `M+0x64` (cadmpeg `display_face_key`).
std::optional<uint32_t> display_face_key(ByteSpan payload, const std::vector<DisplayFace>& faces,
                                         size_t index);

/// Persistent join of one table (cadmpeg `persistent_join_verdict`):
/// the admitted identity, or nullopt; `ambiguous_reused_tag` is set when the
/// join was withheld because of conflicting reused class tags.
std::optional<PersistentFaceIdentity> persistent_join(ByteSpan payload,
                                                      const std::vector<DisplayFace>& faces,
                                                      size_t index, bool* ambiguous_reused_tag);

// ---------------------------------------------------------------------------
// appearance.cc — display appearances (cadmpeg appearance.rs +
// display-colour / appearance-ownership extensions, decode.rs
// ensure_display_appearance / document default moVisualProperties_c).
// ---------------------------------------------------------------------------

/// One resolved display-face appearance definition.
struct DisplayAppearance {
  Color color;
  std::string packed_hex;  ///< e.g. "00404040"
  size_t record_offset = 0;
  std::string name;
};

struct SectionAppearances {
  std::map<size_t, DisplayAppearance> by_face;  ///< table index -> definition
  std::set<uint32_t> matched_feature_sources;
};

/// cadmpeg `appearance::resolve_display_appearances` for one display section.
/// `eligible_features` are the feature source ids admitted by the native
/// appearance plan (B-rep stage).
SectionAppearances resolve_display_appearances(const Container& container,
                                               const DisplaySection& display,
                                               const std::set<uint32_t>& eligible_features);

/// The appearance id for `definition`, adding the Appearance to `model` on
/// first use (cadmpeg `ensure_display_appearance`, DisplayList origin).
std::string ensure_display_appearance(Model& model, const DisplayAppearance& definition,
                                      size_t section_ordinal);
/// appearance.rs `definition_at`: the packed colour at `packed_offset` and the
/// framed name 16 bytes after it.
std::optional<DisplayAppearance> appearance_definition_at(ByteSpan bytes, size_t packed_offset,
                                                          size_t record_offset);
/// The same for a native visual-state definition (ScopedAppearanceOrigin::
/// VisualState: `sldprt:appearance:visualstate#...`, "SolidWorks.VisualStates").
std::string ensure_visual_state_appearance(Model& model, const DisplayAppearance& definition,
                                           size_t section_ordinal);

/// Appearances that cadmpeg adds before the display meshes (document
/// `moVisualProperties_c` definitions, entity-53 records, ...), for the
/// geometry path (`with_brep`) or the metadata-only path. Must reproduce the
/// `appearances` that the reference CADIR lists.
void add_document_appearances(const Container& container, bool with_brep, Model& model);

// ---------------------------------------------------------------------------
// brep.cc / ownership.cc — Parasolid B-rep via libopenparasolid
// ---------------------------------------------------------------------------
/// One entity-53 face colour (cadmpeg `OwnedFaceColor`): the colour ATTRIBUTE
/// record, its REAL_VALUES record and the emitted face it heads, if any.
struct BrepFaceColor {
  uint32_t face_attr = 0;   ///< ATTRIBUTE record attr
  uint32_t color_attr = 0;  ///< REAL_VALUES record attr
  Color color;
  std::optional<std::string> target;  ///< emitted face id (site-qualified)
  std::optional<std::string> site;    ///< site key of a qualified site
};

/// One face of the native visual-state plan (cadmpeg `visual_states::Choice`),
/// in face id order.
struct VisualStateChoice {
  std::string face;
  uint8_t rank = 0;
  size_t ordinal = 0;                           ///< VisualStates section ordinal
  std::string source_name;                      ///< VisualStates section stream name
  std::optional<DisplayAppearance> definition;  ///< nullopt: unresolved
};

/// A native appearance record whose target does not resolve (cadmpeg
/// `UnresolvedAppearanceBinding`), retained as an `unresolved` appearance.
struct UnresolvedAppearance {
  std::string owner_class;
  std::string identity;
  size_t appearance_index = 0;
  std::string reason;
  size_t source_offset = 0;
  std::optional<std::string> packed_hex;
};

struct BrepResult {
  bool decoded = false;  ///< false: no active body stream (metadata-only path)
  std::vector<std::pair<std::string, uint32_t>> face_keys;  ///< face id -> key
  std::vector<std::pair<std::string, PersistentFaceIdentity>> face_identities;
  std::set<uint32_t> eligible_features;  ///< native appearance plan feature sources
  std::vector<BrepFaceColor> face_colors;      ///< entity-53 colours of every site
  std::vector<VisualStateChoice> visual_plan;  ///< native appearance plan
  std::vector<UnresolvedAppearance> visual_unresolved;  ///< plan `unresolved_details`
};

/// decode.rs: the entity-53 appearances and face bindings of `brep.face_colors`
/// (before the material definitions).
void add_brep_face_appearances(const BrepResult& brep, Model& model);
/// decode.rs: the native visual-state plan after tessellation ownership: face
/// colours, visual-state appearances and native bindings.
void apply_visual_state_plan(const BrepResult& brep, Model& model);

/// Decode the active Parasolid partitions into `model` (bodies ... curves,
/// configurations, face colours). `placed_virtual` are virtual component
/// site keys placed by the assembly.
BrepResult decode_brep(const Container& container, const std::set<std::string>& placed_virtual,
                       Model& model);

struct FaceKeyBinding {
  std::string tessellation;
  uint32_t key;
};
struct PersistentFaceBinding {
  std::string tessellation;
  PersistentFaceIdentity identity;
};

/// assign_face_key_owners, assign_persistent_owners, then
/// assign_unique_surface_owners (cadmpeg tessellation.rs).
void assign_tessellation_owners(Model& model, const BrepResult& brep,
                                const std::vector<FaceKeyBinding>& face_keys,
                                const std::vector<PersistentFaceBinding>& persistent);

// ---------------------------------------------------------------------------
// presentation.cc — scene mode, reference planes, pens (with
// scene-mode and reference-plane pen extensions).
// ---------------------------------------------------------------------------
void read_presentation(const Container& container, Model& model);

// ---------------------------------------------------------------------------
// assembly.cc — COMPINSTANCETREE, placement, visibility, shared meshes
// (assembly.rs with placement and shared-mesh extensions).
// ---------------------------------------------------------------------------
struct AssemblyPlan;  // defined in assembly.cc
struct AssemblyPlanDeleter {
  void operator()(AssemblyPlan*) const;
};
using AssemblyPlanPtr = std::unique_ptr<AssemblyPlan, AssemblyPlanDeleter>;

/// nullptr for a part (no instance tree).
AssemblyPlanPtr read_assembly(const Container& container);
std::set<std::string> virtual_site_keys(const AssemblyPlan& plan);
/// Place tessellations, set source_object visibility, expand shared meshes,
/// copy appearance bindings and add occurrences.
void apply_assembly(const AssemblyPlan& plan, const Container& container, Model& model);

}  // namespace swglb
