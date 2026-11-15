// SPDX-License-Identifier: Apache-2.0
// Field-level access to one decoded Parasolid transmit stream
// (libopenparasolid SchemaParser + ModelBuilder).
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "bytes.h"
#include "openparasolid/model.h"
#include "openparasolid/schema.h"

namespace swglb::xt {

// XT node types (Parasolid schema 13006).
enum : uint16_t {
  kBody = 12,
  kShell = 13,
  kFace = 14,
  kLoop = 15,
  kEdge = 16,
  kFin = 17,
  kVertex = 18,
  kRegion = 19,
  kPoint = 29,
  kLine = 30,
  kCircle = 31,
  kEllipse = 32,
  kIntersection = 38,
  kPlane = 50,
  kCylinder = 51,
  kCone = 52,
  kSphere = 53,
  kTorus = 54,
  kBlendedEdge = 56,
  kBlendBound = 59,
  kOffsetSurf = 60,
  kSweptSurf = 67,
  kSpunSurf = 68,
  kAttDefId = 79,
  kAttribDef = 80,
  kAttribute = 81,
  kIntValues = 82,
  kRealValues = 83,
  kCharValues = 84,
  kBSurface = 124,
  kTrimmedCurve = 133,
  kBCurve = 134,
  kSpCurve = 137,
};

/// One parsed stream. Node references are raw stream indices; cadmpeg's
/// entity "attr" is index + 1.
class Stream {
 public:
  /// nullptr when the stream does not parse.
  static std::unique_ptr<Stream> parse(ByteSpan payload);
  /// Attach a parsed deltas journal as an extra source stream (its nodes are
  /// reachable through part_nodes() only). False when `payload` does not
  /// parse or carries no delta nodes.
  bool apply_deltas(ByteSpan payload);

  const openparasolid::DecodedNode* node(uint32_t index) const;
  uint16_t type(uint32_t index) const {
    const auto* n = node(index);
    return n ? n->node_type : 0;
  }
  bool is(uint32_t index, uint16_t t) const { return index != 0 && type(index) == t; }
  /// The partition's node view (one node per index).
  const std::vector<const openparasolid::DecodedNode*>& nodes() const { return merged_; }
  /// Number of source streams (partition first, then each deltas stream).
  size_t parts() const { return parts_.size(); }
  /// The node occurrences of one source stream in stream order.
  const std::vector<openparasolid::DecodedNode>& part_nodes(size_t part) const;
  /// The payload of one source stream; node offsets are relative to it.
  ByteSpan part_bytes(size_t part) const;
  bool part_is_deltas(size_t part) const;

  /// The field named `name`, or nullptr.
  const openparasolid::DecodedField* field(const openparasolid::DecodedNode& n,
                                           std::string_view name) const;
  /// First value of an integer/pointer/char field (0 when absent or unset).
  int64_t integer(const openparasolid::DecodedNode& n, std::string_view name) const;
  uint32_t ptr(uint32_t index, std::string_view name) const {
    const auto* n = node(index);
    if (!n) return 0;
    int64_t v = integer(*n, name);
    return v > 0 && v <= 0xffffffffLL ? static_cast<uint32_t>(v) : 0;
  }
  std::vector<int64_t> integers(const openparasolid::DecodedNode& n, std::string_view name) const;
  std::vector<double> reals(const openparasolid::DecodedNode& n, std::string_view name) const;
  std::optional<double> real(const openparasolid::DecodedNode& n, std::string_view name) const;
  /// Every real value after the `sense` field, in field order (cadmpeg
  /// analytic value order).
  std::vector<double> values_after_sense(const openparasolid::DecodedNode& n) const;
  std::string chars(const openparasolid::DecodedNode& n, std::string_view name) const;

  /// Typed geometry of a node, when ModelBuilder produced it.
  const openparasolid::GeometryObject* geometry(uint32_t index) const;
  bool typed() const { return model_ != nullptr; }
  /// The ModelBuilder model of the partition, or nullptr.
  const openparasolid::Model* model() const { return model_.get(); }

 private:
  struct Part {
    openparasolid::SchemaParseReport report;
    bool deltas = false;
    ByteSpan bytes;  ///< the stream payload (owned by the caller)
    /// Per schema: field name -> definition index.
    std::vector<std::unordered_map<std::string, uint64_t>> names;
  };
  const Part* part_of(const openparasolid::DecodedNode& n) const;
  void rebuild();

  std::vector<std::unique_ptr<Part>> parts_;
  std::unique_ptr<openparasolid::Model> model_;
  std::unordered_map<uint32_t, const openparasolid::DecodedNode*> by_index_;
  std::vector<const openparasolid::DecodedNode*> merged_;
};

}  // namespace swglb::xt
