// SPDX-License-Identifier: Apache-2.0
// Embedded Parasolid stream extraction from SOLIDWORKS sections.
// Ported from cadmpeg-codec-sldprt `parasolid.rs` (cadmpeg 5af95421,
// Apache-2.0).
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "bytes.h"

namespace swglb {

/// One located and header-validated Parasolid stream.
struct PsStream {
  size_t offset = 0;        ///< Direct-stream or wrapper offset in the section payload.
  Bytes payload;            ///< Complete stream bytes starting at `PS\0\0`.
  std::string description;  ///< Stream header description (lossy UTF-8).
  std::string schema;       ///< `SCH_...` token.
  size_t body_offset = 0;   ///< End of the schema token.
};

/// Every stream carried by one section payload (cadmpeg
/// `extract_streams_in_section`), in payload order.
std::vector<PsStream> extract_parasolid_streams(ByteSpan payload, std::string_view section);

/// Header of a stream that starts within its first 64 bytes, or nullopt.
std::optional<PsStream> parasolid_stream_header(ByteSpan payload);

/// Description names a partition or deltas body stream.
bool is_body_stream(const PsStream& stream);
bool is_deltas_stream(const PsStream& stream);

/// ASCII case-insensitive substring test.
bool icontains(std::string_view haystack, std::string_view needle);

}  // namespace swglb
