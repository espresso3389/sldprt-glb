// SPDX-License-Identifier: Apache-2.0
// Bounds-checked little-endian readers over an in-memory byte image.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace swglb {

using Bytes = std::vector<uint8_t>;
using ByteSpan = std::span<const uint8_t>;

inline std::optional<uint8_t> u8_at(ByteSpan b, size_t at) {
  if (at >= b.size()) return std::nullopt;
  return b[at];
}

inline std::optional<uint16_t> u16_at(ByteSpan b, size_t at) {
  if (at > b.size() || b.size() - at < 2) return std::nullopt;
  return static_cast<uint16_t>(b[at] | (b[at + 1] << 8));
}

inline std::optional<uint32_t> u32_at(ByteSpan b, size_t at) {
  if (at > b.size() || b.size() - at < 4) return std::nullopt;
  return static_cast<uint32_t>(b[at]) | (static_cast<uint32_t>(b[at + 1]) << 8) |
         (static_cast<uint32_t>(b[at + 2]) << 16) | (static_cast<uint32_t>(b[at + 3]) << 24);
}

inline std::optional<int32_t> i32_at(ByteSpan b, size_t at) {
  auto v = u32_at(b, at);
  if (!v) return std::nullopt;
  return static_cast<int32_t>(*v);
}

inline std::optional<uint32_t> u32be_at(ByteSpan b, size_t at) {
  if (at > b.size() || b.size() - at < 4) return std::nullopt;
  return (static_cast<uint32_t>(b[at]) << 24) | (static_cast<uint32_t>(b[at + 1]) << 16) |
         (static_cast<uint32_t>(b[at + 2]) << 8) | static_cast<uint32_t>(b[at + 3]);
}

inline std::optional<uint64_t> u64_at(ByteSpan b, size_t at) {
  auto lo = u32_at(b, at);
  auto hi = u32_at(b, at + 4);
  if (!lo || !hi) return std::nullopt;
  return static_cast<uint64_t>(*lo) | (static_cast<uint64_t>(*hi) << 32);
}

inline std::optional<float> f32_at(ByteSpan b, size_t at) {
  auto v = u32_at(b, at);
  if (!v) return std::nullopt;
  float f;
  uint32_t raw = *v;
  std::memcpy(&f, &raw, 4);
  return f;
}

inline std::optional<double> f64_at(ByteSpan b, size_t at) {
  auto v = u64_at(b, at);
  if (!v) return std::nullopt;
  double d;
  uint64_t raw = *v;
  std::memcpy(&d, &raw, 8);
  return d;
}

/// Sub-span [start, end), or nullopt when it does not lie inside `b`.
inline std::optional<ByteSpan> slice(ByteSpan b, size_t start, size_t end) {
  if (start > end || end > b.size()) return std::nullopt;
  return b.subspan(start, end - start);
}

inline bool starts_with_at(ByteSpan b, size_t at, ByteSpan needle) {
  if (at > b.size() || b.size() - at < needle.size()) return false;
  return std::equal(needle.begin(), needle.end(), b.begin() + static_cast<std::ptrdiff_t>(at));
}

inline bool starts_with_at(ByteSpan b, size_t at, std::string_view needle) {
  return starts_with_at(
      b, at, ByteSpan(reinterpret_cast<const uint8_t*>(needle.data()), needle.size()));
}

/// Every offset at which `needle` occurs in `hay` (overlapping allowed).
std::vector<size_t> find_all(ByteSpan hay, ByteSpan needle);
inline std::vector<size_t> find_all(ByteSpan hay, std::string_view needle) {
  return find_all(hay, ByteSpan(reinterpret_cast<const uint8_t*>(needle.data()), needle.size()));
}

/// First offset >= from at which `needle` occurs before `limit`, or nullopt.
std::optional<size_t> find_from(ByteSpan hay, ByteSpan needle, size_t from, size_t limit);
inline std::optional<size_t> find_from(ByteSpan hay, std::string_view needle, size_t from,
                                       size_t limit) {
  return find_from(hay, ByteSpan(reinterpret_cast<const uint8_t*>(needle.data()), needle.size()),
                   from, limit);
}

inline bool contains(ByteSpan hay, std::string_view needle) {
  return find_from(hay, needle, 0, hay.size()).has_value();
}

/// ASCII case-insensitive equality.
bool iequals(std::string_view a, std::string_view b);
bool iends_with(std::string_view s, std::string_view suffix);
bool istarts_with(std::string_view s, std::string_view prefix);

}  // namespace swglb
