// SPDX-License-Identifier: Apache-2.0
// SOLIDWORKS outer container: native compressed blocks and Compound File
// Binary (CFB) streams. Ported from cadmpeg-codec-sldprt `container.rs`
// (cadmpeg 5af95421, Apache-2.0).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "bytes.h"

namespace swglb {

/// One named payload of the source document.
///
/// `ordinal` matches cadmpeg's `Section::ordinal()`: the marker offset of a
/// native block, or the CFB directory id of a compound stream. It is part of
/// the deterministic tessellation identifiers.
struct Section {
  std::optional<std::string> name;  ///< Decoded stream name; absent for anonymous blocks.
  std::string source_stream;        ///< Name, or "block@<offset>" / "compound@<id>".
  size_t ordinal = 0;
  bool compound = false;
  Bytes payload;  ///< Inflated semantic bytes.

  /// "block@<offset>" or "compound@<id>" (cadmpeg `site_key`).
  std::string site_key() const;
};

struct Container {
  bool compound = false;
  uint32_t version = 0;           ///< Native outer version (4); 0 for CFB.
  std::vector<Section> sections;  ///< Native blocks in file order, then CFB streams.
  /// Every storage and stream path seen in a CFB directory (including empty
  /// storages), or every native block/directory name. Used for
  /// `_DL_VERSION_` / `_MO_VERSION_` resolution.
  std::vector<std::string> entry_names;

  const Section* find(std::string_view name) const;  ///< Case-insensitive name match.
};

/// Error raised for a container that cannot be read at all.
struct ContainerError {
  std::string message;
};

/// Read a complete SLDPRT/SLDASM image. Returns nullopt and sets `error`
/// when the container framing is unreadable.
std::optional<Container> read_container(ByteSpan image, ContainerError* error);

/// Raw DEFLATE inflate with an exact expected output size.
std::optional<Bytes> inflate_raw_exact(ByteSpan compressed, size_t expected);
/// zlib member inflate with an exact expected output size; also returns the
/// number of compressed bytes consumed.
std::optional<std::pair<Bytes, size_t>> inflate_zlib_exact(ByteSpan compressed, size_t expected);

uint32_t crc32_of(ByteSpan data);

}  // namespace swglb
