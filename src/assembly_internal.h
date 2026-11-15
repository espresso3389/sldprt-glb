// SPDX-License-Identifier: Apache-2.0
// Internal interface of the assembly stage (assembly.cc), exposed for
// tests/assembly_test.cc. Ported from cadmpeg-codec-sldprt `assembly.rs`,
// `assembly/native.rs` and `assembly/display.rs` (cadmpeg 5af95421 with
// assembly-placement and shared-mesh extensions, Apache-2.0).
#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "container.h"
#include "model.h"

namespace swglb {
namespace assembly {

/// cadmpeg `Transform`: three affine rows for column vectors, millimetres.
struct Transform {
  double rows[3][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
  static Transform identity() { return {}; }
  /// cadmpeg `Transform::is_proper_rigid` (EPSILON 1e-9).
  bool is_proper_rigid() const;
  /// cadmpeg `Transform::compose`: `this * right`; nullopt when not finite.
  std::optional<Transform> compose(const Transform& right) const;
  /// cadmpeg `Transform::apply_point` / `apply_vector`; nullopt on overflow.
  std::optional<Vec3> apply_point(const Vec3& p) const;
  std::optional<Vec3> apply_vector(const Vec3& v) const;
};

/// cadmpeg `PrototypeReference` as the shared-mesh expansion compares it.
struct Prototype {
  enum class Kind { Local, External, Unresolved };
  Kind kind = Kind::Unresolved;
  std::string definition;             ///< Local: product definition id.
  std::string document;               ///< External: path; "" = ExternalDocument::Missing.
  std::optional<std::string> object;  ///< External: nonblank configuration.
  bool operator==(const Prototype&) const = default;
};

/// One projected occurrence (cadmpeg `NativePlacement` / `Occurrence`).
struct PlanOccurrence {
  std::string id;                     ///< "sldprt:assembly:occurrence#<key>"
  std::optional<std::string> parent;  ///< Parent occurrence id; nullopt = root.
  Transform transform;
  std::optional<std::string> name;
  std::optional<bool> visible;
  Prototype prototype;
};

/// cadmpeg display.rs `Placement` without the instance list.
struct Placement {
  std::string path;
  Transform transform;
  bool visible = true;
};

/// cadmpeg `world`: the complete instance path, world transform and
/// effective visibility of `occurrence`. Throws DecodeError for an absent or
/// cyclic parent, a non-rigid placement, or a missing name.
Placement world(const PlanOccurrence& occurrence, const std::vector<PlanOccurrence>& occurrences);

/// Projection of the component tree text (cadmpeg `native_project` +
/// `NativeAssembly::into_projection`, without loss notes). `virtual_sections`
/// maps each virtual component section key to its site key, or nullopt when
/// sections share the key.
struct Projection {
  std::vector<PlanOccurrence> occurrences;
  std::vector<std::string> virtual_sites;  ///< Site keys of virtual definitions.
};
Projection project(std::string_view text,
                   const std::map<std::string, std::optional<std::string>>& virtual_sections);

/// cadmpeg `placement`: an `swTransform` value as a proper rigid placement.
std::optional<Transform> placement(std::string_view text);

/// Result of cadmpeg display.rs `read`, keyed by tessellation id.
struct DisplayPlan {
  std::vector<Placement> owners;                          ///< One per owning path.
  std::vector<std::vector<Placement>> instances;          ///< Shared copies per owner.
  std::unordered_map<std::string, size_t> table_owner;    ///< tessellation id -> owner
  std::vector<Placement> worlds;                          ///< `world` of each occurrence.
};

/// cadmpeg display.rs `read`. Table byte ranges come from the tessellations
/// already in `model` (one per descriptor table of each section).
DisplayPlan read_display(const std::vector<PlanOccurrence>& occurrences,
                         const Container& container, const Model& model);

/// cadmpeg display.rs `apply`: place meshes, append shared copies and their
/// copied appearance bindings.
void apply_display(const DisplayPlan& plan, Model& model);

}  // namespace assembly

struct AssemblyPlan {
  assembly::Projection projection;
};

}  // namespace swglb
