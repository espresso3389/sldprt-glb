// SPDX-License-Identifier: Apache-2.0
// Internal pieces of presentation.cc, exposed for tests/presentation_test.cc.
// Ported from cadmpeg-codec-sldprt `metadata/presentation.rs` (with
// scene-mode, reference-plane pen and legacy pen extensions).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "bytes.h"
#include "model.h"

namespace swglb::presentation {

/// cadmpeg `display_version`: `_DL_VERSION_<n>` / `_MO_VERSION_<n>` root names.
std::optional<uint32_t> display_version(const std::vector<std::string>& names);

/// cadmpeg `saved_mode`: (mode, end position) of the scene header at `at`.
std::optional<std::pair<uint32_t, size_t>> saved_mode(ByteSpan data, size_t at);

/// cadmpeg `plane`: one command-6 reference plane at `at` (pen fields unset).
std::optional<ReferencePlane> plane(ByteSpan data, size_t at);

/// cadmpeg `PenColor`.
struct PenColor {
  enum Kind { Indexed, Custom, Unchanged } kind = Unchanged;
  uint32_t index = 0;
  std::array<double, 3> rgb{};
  bool operator==(const PenColor&) const = default;
};

/// cadmpeg `pen_color`: command 9 at `at`; (colour, end position).
std::optional<std::pair<PenColor, size_t>> pen_color(ByteSpan data, size_t at,
                                                     std::optional<uint32_t> version);

/// cadmpeg `custom_pen_color`: command 12 at `at`; (rgb, end position).
std::optional<std::pair<std::array<double, 3>, size_t>> custom_pen_color(
    ByteSpan data, size_t at, std::optional<uint32_t> version);

/// cadmpeg `append` for one `Contents/DisplayLists` payload. Appends to
/// `planes`; sets `mode` when the scene header is found.
void append(ByteSpan data, std::optional<uint32_t> version, std::optional<int64_t>& mode,
            std::vector<ReferencePlane>& planes);

}  // namespace swglb::presentation
