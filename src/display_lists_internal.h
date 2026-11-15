// SPDX-License-Identifier: Apache-2.0
// Internal interface of the DisplayLists descriptor-table reader
// (display_lists.cc), shared with appearance.cc and the unit tests.
// Ported from cadmpeg-codec-sldprt tessellation.rs / framed_string.rs and
// cadmpeg-core decode/utf16.rs (cadmpeg 5af95421, Apache-2.0).
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "bytes.h"
#include "container.h"
#include "stages.h"

namespace swglb {

/// One `ff ff 01 00 <u16 len> <name>` class declaration and the bytes up to
/// the next declaration (cadmpeg tessellation.rs `ClassInterval`; the scene
/// `source_ids` are not needed for GLB output and are not collected).
struct ClassInterval {
  std::string name;
  size_t class_offset = 0;
  ByteRange content;
};

/// cadmpeg tessellation.rs `class_intervals`.
std::vector<ClassInterval> class_intervals(ByteSpan payload);

/// Lazily computed per-section index shared by the surface references of
/// one section, so `persistent_join` computes the section's class intervals
/// once (cadmpeg computes them once per section in `push_display_meshes`).
struct DisplaySectionIndex {
  const std::vector<ClassInterval>& classes(ByteSpan payload) const {
    if (!classes_) classes_ = class_intervals(payload);
    return *classes_;
  }

 private:
  mutable std::optional<std::vector<ClassInterval>> classes_;
};

/// cadmpeg tessellation.rs `PersistentSurfaceReference`: `complete` is the
/// `Complete(identity)` variant; otherwise `SourceOnly` keeps
/// `identity.feature_source_id` / `identity.local_id` with an empty tail.
struct DisplayFace::SurfaceReference {
  bool complete = false;
  PersistentFaceIdentity identity;
  std::shared_ptr<const DisplaySectionIndex> section;
};

// ---------------------------------------------------------------------------
// Text helpers
// ---------------------------------------------------------------------------

/// cadmpeg-core `utf16le_text` (Surrogates::Reject, no NUL trimming):
/// nullopt for a truncated window or an unpaired surrogate.
std::optional<std::u32string> decode_utf16le_strict(ByteSpan raw, size_t units);
/// Rust `str::trim` (Unicode White_Space).
std::u32string_view trim_unicode(std::u32string_view text);
std::string to_utf8(std::u32string_view text);

/// cadmpeg framed_string.rs `read_framed_string`: the decoded text and the
/// offset after it. Absent marker and failed reads both yield nullopt (every
/// caller treats them alike).
struct FramedText {
  std::u32string value;
  size_t end = 0;
};
std::optional<FramedText> read_framed_string(ByteSpan data, size_t at);

// ---------------------------------------------------------------------------
// Descriptor tables (exposed for the unit tests)
// ---------------------------------------------------------------------------

/// One parsed table: strips and whether it carries normals.
struct ParsedTable {
  std::vector<std::vector<MeshVertex>> strips;
  bool has_normals = true;
  size_t end = 0;
};

/// cadmpeg `parse_table`: nullopt is a probe miss; a lane disagreement inside
/// a recognised table throws DecodeError.
std::optional<ParsedTable> parse_table(ByteSpan bytes, size_t at);
/// cadmpeg `descriptor_table_offset`.
size_t descriptor_table_offset(ByteSpan payload, size_t at);
/// cadmpeg `section_display_faces`.
std::vector<DisplayFace> section_display_faces(ByteSpan payload);

using SurfaceReferencePtr = std::shared_ptr<const DisplayFace::SurfaceReference>;

/// cadmpeg `SurfaceTokenStatus` for a rejected string.
enum class SurfaceTokenStatus { Complete, OpaqueTail, Invalid };
/// cadmpeg `classify_surface_token`; on failure returns nullptr and sets
/// `status` (when given) to Invalid.
SurfaceReferencePtr classify_surface_token(std::u32string_view text,
                                           SurfaceTokenStatus* status = nullptr);
/// cadmpeg `persistent_surface_references` over `range`.
std::vector<SurfaceReferencePtr> persistent_surface_references(
    ByteSpan payload, ByteRange range, std::shared_ptr<const DisplaySectionIndex> section = {});

enum class SurfaceTokenWithhold { None, Conflicting, OpaqueTail, Invalid };
/// cadmpeg `agree_surface_tokens`: the agreed identity, or nullopt with the
/// withhold reason in `reason`.
std::optional<PersistentFaceIdentity> agree_surface_tokens(
    const std::vector<SurfaceReferencePtr>& references, SurfaceTokenWithhold* reason = nullptr);

/// cadmpeg `ReusedTagVerdict`.
struct ReusedTagVerdict {
  enum Kind { Absent, Admit, Withhold } kind = Absent;
  uint16_t tag = 0;
};
/// cadmpeg `admit_reused_face_tag`. `classes` must be in payload order (as
/// `class_intervals` returns them).
ReusedTagVerdict admit_reused_face_tag(ByteSpan payload, const DisplayFace& face,
                                       size_t previous_table_end,
                                       const std::vector<ClassInterval>& classes);

/// Triangle and strip counts of a face's mesh (cadmpeg `Mesh::triangle_count`
/// / `strip_count`).
size_t face_triangle_count(const DisplayFace& face);
inline size_t face_strip_count(const DisplayFace& face) { return face.strips.size(); }

/// cadmpeg `DisplayFace::feature_source_id`: the source id only when every
/// reference agrees on it.
std::optional<uint32_t> face_feature_source_id(const DisplayFace& face);

}  // namespace swglb
