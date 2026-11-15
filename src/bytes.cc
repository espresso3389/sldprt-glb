// SPDX-License-Identifier: Apache-2.0
#include "bytes.h"

#include <algorithm>
#include <functional>

namespace swglb {

std::vector<size_t> find_all(ByteSpan hay, ByteSpan needle) {
  std::vector<size_t> out;
  if (needle.empty() || hay.size() < needle.size()) return out;
  std::boyer_moore_horspool_searcher searcher(needle.begin(), needle.end());
  auto it = hay.begin();
  while (true) {
    auto [first, last] = searcher(it, hay.end());
    if (first == hay.end()) break;
    out.push_back(static_cast<size_t>(first - hay.begin()));
    it = first + 1;
  }
  return out;
}

std::optional<size_t> find_from(ByteSpan hay, ByteSpan needle, size_t from, size_t limit) {
  limit = std::min(limit, hay.size());
  if (needle.empty() || from > limit || limit - from < needle.size()) return std::nullopt;
  auto begin = hay.begin() + static_cast<std::ptrdiff_t>(from);
  auto end = hay.begin() + static_cast<std::ptrdiff_t>(limit);
  auto it = std::search(begin, end, needle.begin(), needle.end());
  if (it == end) return std::nullopt;
  return static_cast<size_t>(it - hay.begin());
}

static char lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + 32) : c; }

bool iequals(std::string_view a, std::string_view b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i)
    if (lower(a[i]) != lower(b[i])) return false;
  return true;
}

bool iends_with(std::string_view s, std::string_view suffix) {
  return s.size() >= suffix.size() && iequals(s.substr(s.size() - suffix.size()), suffix);
}

bool istarts_with(std::string_view s, std::string_view prefix) {
  return s.size() >= prefix.size() && iequals(s.substr(0, prefix.size()), prefix);
}

}  // namespace swglb
