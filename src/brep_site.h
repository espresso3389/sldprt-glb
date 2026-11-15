// SPDX-License-Identifier: Apache-2.0
// One Parasolid stream site decoded into CADIR B-rep arenas (cadmpeg
// brep/graph.rs `decode_graph`), through libopenparasolid.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "brep_streams.h"
#include "bytes.h"
#include "model.h"
#include "stages.h"

namespace swglb {

struct SiteBrep {
  std::vector<Body> bodies;
  std::vector<Region> regions;
  std::vector<Shell> shells;
  std::vector<Face> faces;
  std::vector<Loop> loops;
  std::vector<Coedge> coedges;
  std::vector<Edge> edges;
  std::vector<Vertex> vertices;
  std::vector<Point> points;
  std::vector<Surface> surfaces;
  std::vector<Curve> curves;

  std::vector<std::pair<std::string, uint32_t>> face_keys;
  std::vector<std::pair<std::string, PersistentFaceIdentity>> face_identities;
  /// Native FACE_ID_2001 payloads of emitted faces (visual-state scope).
  std::vector<std::pair<std::string, std::vector<uint32_t>>> face_payloads;
  /// Native ATOM_ID_2001 payloads of emitted bodies.
  std::vector<std::pair<std::string, std::vector<uint32_t>>> body_payloads;
  /// Entity-53 face colours (cadmpeg `face_colors`), attrs in attr space.
  std::vector<BrepFaceColor> face_colors;
  /// Framed topology records (cadmpeg `source_entity_records`).
  size_t entity_records = 0;

  bool has_geometry() const { return !faces.empty() || !surfaces.empty() || !points.empty(); }
};

/// Decode the streams of one site (partition snapshots; deltas are not
/// applied). nullopt when no stream parses.
std::optional<SiteBrep> decode_site(const std::vector<const PsStream*>& streams);

/// Append "@<site>" to every identity of `brep` (cadmpeg `qualify_ids`).
void qualify_site(SiteBrep& brep, const std::string& site);

/// Append `from` to `into`.
void merge_site(SiteBrep& into, SiteBrep&& from);

}  // namespace swglb
