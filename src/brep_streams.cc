// SPDX-License-Identifier: Apache-2.0
// Ported from cadmpeg-codec-sldprt `parasolid.rs` (cadmpeg 5af95421,
// Apache-2.0). The `ThirdPty/WELDMENT_STORAGE` direct reader is not ported:
// those streams are not partition/deltas body streams.
#include "brep_streams.h"

#include <zlib.h>

#include <array>
#include <cstring>

#include "container.h"

namespace swglb {
namespace {

constexpr std::array<uint8_t, 16> kWrappedMagic = {0x23, 0x1d, 0xd5, 0x71, 0xda, 0x81, 0x48, 0xa2,
                                                   0xa8, 0x58, 0x98, 0xb2, 0x1b, 0x89, 0xef, 0x99};
constexpr size_t kFrameHeaderLen = 8;  // u32 uncompressed, u32 member size
constexpr size_t kChainMagicAt = 4;    // chain length precedes the magic
constexpr size_t kMaxFrame = 512u * 1024 * 1024;
constexpr size_t kProbeWorkPerByte = 16;

bool has_magic_at(ByteSpan b, size_t at) {
  return starts_with_at(b, at, ByteSpan(kWrappedMagic));
}

bool has_wrapped_prefix(ByteSpan payload) {
  return has_magic_at(payload, 0) || has_magic_at(payload, kChainMagicAt);
}

bool schema_char(uint8_t c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

// cadmpeg-parasolid `find_u8_length_prefixed_schema_token`.
std::optional<std::pair<size_t, size_t>> length_prefixed_schema(ByteSpan prologue) {
  for (size_t off = 0; off + 4 <= prologue.size(); ++off) {
    if (std::memcmp(prologue.data() + off, "SCH_", 4) != 0) continue;
    if (off == 0) continue;
    size_t length = prologue[off - 1];
    size_t end = off + length;
    if (end > prologue.size() || length <= 4) continue;
    bool ok = true;
    for (size_t i = off; i < end; ++i) ok &= schema_char(prologue[i]);
    if (ok) return std::make_pair(off, end);
  }
  return std::nullopt;
}

std::optional<Bytes> inflate_frame(ByteSpan member, size_t expected) {
  if (expected == 0 || expected > kMaxFrame) return std::nullopt;
  auto r = inflate_zlib_exact(member, expected);
  if (!r || r->second != member.size()) return std::nullopt;
  return std::move(r->first);
}

std::optional<PsStream> extracted(size_t offset, Bytes payload) {
  if (payload.size() < 4 || std::memcmp(payload.data(), "PS\0\0", 4) != 0) return std::nullopt;
  auto header = parasolid_stream_header(payload);
  if (!header) return std::nullopt;
  header->offset = offset;
  header->payload = std::move(payload);
  return header;
}

std::optional<PsStream> single_wrapped(ByteSpan payload, size_t magic_at) {
  size_t frame_at = magic_at + kWrappedMagic.size();
  auto uncompressed = u32_at(payload, frame_at);
  auto member_size = u32_at(payload, frame_at + 4);
  if (!uncompressed || !member_size) return std::nullopt;
  size_t start = frame_at + kFrameHeaderLen;
  auto member = slice(payload, start, start + *member_size);
  if (!member) return std::nullopt;
  auto inflated = inflate_frame(*member, *uncompressed);
  if (!inflated) return std::nullopt;
  return extracted(magic_at, std::move(*inflated));
}

std::optional<PsStream> chained_wrapped(ByteSpan payload, size_t magic_at) {
  if (magic_at < kChainMagicAt) return std::nullopt;
  size_t chain_len_at = magic_at - kChainMagicAt;
  auto chain_len = u32_at(payload, chain_len_at);
  if (!chain_len || *chain_len < kWrappedMagic.size() + kFrameHeaderLen) return std::nullopt;
  size_t section_end = magic_at + *chain_len;
  if (section_end > payload.size()) return std::nullopt;
  size_t frame_at = magic_at + kWrappedMagic.size();
  std::vector<Bytes> frames;
  while (frame_at < section_end) {
    bool zeros = true;
    for (size_t i = frame_at; i < section_end && zeros; ++i) zeros = payload[i] == 0;
    if (zeros) break;
    if (section_end - frame_at < kFrameHeaderLen) return std::nullopt;
    uint32_t uncompressed = *u32_at(payload, frame_at);
    uint32_t member_size = *u32_at(payload, frame_at + 4);
    if (uncompressed == 0 || member_size == 0) return std::nullopt;
    size_t start = frame_at + kFrameHeaderLen;
    size_t end = start + member_size;
    if (end > section_end) return std::nullopt;
    auto frame = inflate_frame(payload.subspan(start, member_size), uncompressed);
    if (!frame) return std::nullopt;
    frames.push_back(std::move(*frame));
    frame_at = end;
  }
  if (frames.empty()) return std::nullopt;
  Bytes stream;
  if (frames.size() == 1) {
    stream = std::move(frames[0]);
  } else {
    size_t total = 0;
    for (const auto& f : frames) total += f.size();
    stream.reserve(total);
    for (const auto& f : frames) stream.insert(stream.end(), f.begin(), f.end());
  }
  return extracted(chain_len_at, std::move(stream));
}

// One zlib member of unknown size starting at `in` (cadmpeg
// `inflate_zlib_member`, ExpandSpec::Unknown).
std::optional<Bytes> inflate_unknown(ByteSpan in) {
  z_stream zs{};
  if (inflateInit2(&zs, 15) != Z_OK) return std::nullopt;
  Bytes out;
  std::array<uint8_t, 65536> chunk;
  zs.next_in = const_cast<Bytef*>(in.data());
  zs.avail_in = static_cast<uInt>(std::min<size_t>(in.size(), 0x7fffffff));
  int rc = Z_OK;
  while (rc == Z_OK) {
    zs.next_out = chunk.data();
    zs.avail_out = static_cast<uInt>(chunk.size());
    rc = inflate(&zs, Z_NO_FLUSH);
    size_t produced = chunk.size() - zs.avail_out;
    out.insert(out.end(), chunk.begin(), chunk.begin() + static_cast<std::ptrdiff_t>(produced));
    if (out.size() > kMaxFrame) rc = Z_MEM_ERROR;
    if (rc == Z_OK && produced == 0 && zs.avail_in == 0) rc = Z_BUF_ERROR;
  }
  inflateEnd(&zs);
  if (rc != Z_STREAM_END) return std::nullopt;
  return out;
}

bool payload_present(const std::vector<PsStream>& streams, const Bytes& payload) {
  for (const auto& s : streams)
    if (s.payload == payload) return true;
  return false;
}

}  // namespace

bool icontains(std::string_view haystack, std::string_view needle) {
  if (needle.empty()) return true;
  if (haystack.size() < needle.size()) return false;
  for (size_t i = 0; i + needle.size() <= haystack.size(); ++i)
    if (iequals(haystack.substr(i, needle.size()), needle)) return true;
  return false;
}

std::optional<PsStream> parasolid_stream_header(ByteSpan payload) {
  size_t window = std::min<size_t>(payload.size(), 64);
  std::optional<size_t> sig;
  for (size_t i = 0; i + 4 <= window; ++i)
    if (std::memcmp(payload.data() + i, "PS\0\0", 4) == 0) {
      sig = i;
      break;
    }
  if (!sig) return std::nullopt;
  size_t desc_len_at = *sig + 4;
  if (desc_len_at + 2 > payload.size()) return std::nullopt;
  size_t desc_len = (static_cast<size_t>(payload[desc_len_at]) << 8) | payload[desc_len_at + 1];
  size_t desc_start = desc_len_at + 2;
  size_t desc_end = desc_start + desc_len;
  if (desc_end > payload.size()) return std::nullopt;
  size_t window_end = std::min(desc_end + 64, payload.size());
  auto token = length_prefixed_schema(payload.subspan(desc_end, window_end - desc_end));
  if (!token) return std::nullopt;
  PsStream out;
  out.description.assign(reinterpret_cast<const char*>(payload.data() + desc_start), desc_len);
  out.schema.assign(reinterpret_cast<const char*>(payload.data() + desc_end + token->first),
                    token->second - token->first);
  out.body_offset = desc_end + token->second;
  return out;
}

bool is_body_stream(const PsStream& stream) {
  return icontains(stream.description, "partition") || icontains(stream.description, "deltas");
}

bool is_deltas_stream(const PsStream& stream) { return icontains(stream.description, "deltas"); }

std::vector<PsStream> extract_parasolid_streams(ByteSpan payload, std::string_view section) {
  std::vector<PsStream> out;
  bool wrapped_prefix = has_wrapped_prefix(payload);
  if (!wrapped_prefix) {
    std::vector<std::pair<size_t, PsStream>> starts;
    for (size_t at : find_all(payload, std::string_view("PS\0\0", 4))) {
      if (auto header = parasolid_stream_header(payload.subspan(at)))
        starts.emplace_back(at, std::move(*header));
    }
    for (size_t i = 0; i < starts.size(); ++i) {
      size_t start = starts[i].first;
      size_t end = i + 1 < starts.size() ? starts[i + 1].first : payload.size();
      PsStream s = std::move(starts[i].second);
      s.offset = start;
      s.payload.assign(payload.begin() + static_cast<std::ptrdiff_t>(start),
                       payload.begin() + static_cast<std::ptrdiff_t>(end));
      out.push_back(std::move(s));
    }
    if (!out.empty()) return out;
  }
  if (payload.size() < kWrappedMagic.size()) return out;
  bool has_magic = false;
  for (size_t at : find_all(payload, ByteSpan(kWrappedMagic))) {
    has_magic = true;
    auto stream = at == 0 ? single_wrapped(payload, at) : chained_wrapped(payload, at);
    if (stream && !payload_present(out, stream->payload)) out.push_back(std::move(*stream));
  }
  if (!has_magic) return out;
  if (!out.empty()) {
    std::stable_sort(out.begin(), out.end(),
                     [](const PsStream& a, const PsStream& b) { return a.offset < b.offset; });
    return out;
  }
  if (wrapped_prefix) return out;
  // Older nested wrappers: the first zlib member inflating to `PS\0\0`.
  size_t budget = payload.size() * kProbeWorkPerByte;
  size_t work = 0;
  for (size_t i = 0; i + 2 <= payload.size(); ++i) {
    if (payload[i] != 0x78 || !(payload[i + 1] == 0x01 || payload[i + 1] == 0x9c ||
                                payload[i + 1] == 0xda))
      continue;
    work += payload.size() - i;
    if (work > budget) break;
    auto inner = inflate_unknown(payload.subspan(i));
    if (!inner || inner->size() < 4 || std::memcmp(inner->data(), "PS\0\0", 4) != 0) continue;
    auto header = parasolid_stream_header(*inner);
    if (!header) continue;
    header->offset = i;
    header->payload = std::move(*inner);
    if (!payload_present(out, header->payload)) out.push_back(std::move(*header));
  }
  (void)section;
  return out;
}

}  // namespace swglb
