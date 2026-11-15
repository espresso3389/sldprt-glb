// SPDX-License-Identifier: Apache-2.0
// Ported from cadmpeg-codec-sldprt `container.rs` and cadmpeg-container
// `compound.rs` (cadmpeg 5af95421, Apache-2.0).
#include "container.h"

#include <zlib.h>

#include <array>
#include <cstring>
#include <functional>

namespace swglb {
namespace {

constexpr std::array<uint8_t, 6> kMarker = {0x14, 0x00, 0x06, 0x00, 0x08, 0x00};
constexpr std::array<uint8_t, 8> kCfbMagic = {0xd0, 0xcf, 0x11, 0xe0, 0xa1, 0xb1, 0x1a, 0xe1};
constexpr std::array<uint8_t, 16> kZlbMagic = {0x23, 0x1d, 0xd5, 0x71, 0xda, 0x81, 0x48, 0xa2,
                                               0xa8, 0x58, 0x98, 0xb2, 0x1b, 0x89, 0xef, 0x99};
constexpr uint32_t kNativeVersion = 4;
constexpr size_t kMaxUncompressed = 512u * 1024 * 1024;

// block_frame_header (offsets from the 6-byte marker).
constexpr size_t kBlockLen = 26, kBlockTypeId = 6, kBlockCrc = 10, kBlockComp = 14,
                 kBlockUncomp = 18, kBlockPre = 22;
// cache_cell_header.
constexpr size_t kCacheLen = 26, kCacheTwoL = 10, kCacheHalfL = 14, kCacheL = 18,
                 kCacheNameLen = 22;
// tail_directory_entry.
constexpr size_t kDirLen = 40, kDirZero10 = 10, kDirZero18 = 18, kDirNameLen = 22;

std::optional<std::string> nibble_swap_name(ByteSpan raw) {
  std::string out;
  out.reserve(raw.size());
  for (uint8_t b : raw) {
    uint8_t c = static_cast<uint8_t>((b << 4) | (b >> 4));
    if (c < 0x20 || c >= 0x7f) return std::nullopt;
    out.push_back(static_cast<char>(c));
  }
  return out;
}

struct Frame {
  uint32_t crc, comp, uncomp, pre;
  size_t payload_start, payload_end;
};

std::optional<Frame> read_frame(ByteSpan bytes, size_t off) {
  auto crc = u32_at(bytes, off + kBlockCrc);
  auto comp = u32_at(bytes, off + kBlockComp);
  auto uncomp = u32_at(bytes, off + kBlockUncomp);
  auto pre = u32_at(bytes, off + kBlockPre);
  if (!u32_at(bytes, off + kBlockTypeId) || !crc || !comp || !uncomp || !pre) return std::nullopt;
  if (*comp == 0) return std::nullopt;
  size_t start = off + kBlockLen + *pre;
  size_t end = start + *comp;
  if (start < off || end < start || end > bytes.size()) return std::nullopt;
  return Frame{*crc, *comp, *uncomp, *pre, start, end};
}

bool is_cache_cell(ByteSpan bytes, size_t off) {
  auto two_l = u32_at(bytes, off + kCacheTwoL);
  auto half_l = u32_at(bytes, off + kCacheHalfL);
  auto l = u32_at(bytes, off + kCacheL);
  auto name_len = u32_at(bytes, off + kCacheNameLen);
  if (!two_l || !half_l || !l || !name_len) return false;
  if (*l == 0 || static_cast<uint64_t>(*l) * 2 != *two_l || *half_l != *l / 2) return false;
  if (*name_len == 0 || *name_len >= 500) return false;
  auto raw = slice(bytes, off + kCacheLen, off + kCacheLen + *name_len);
  return raw && nibble_swap_name(*raw).has_value();
}

std::optional<std::string> directory_entry_name(ByteSpan bytes, size_t off) {
  auto zero_a = u32_at(bytes, off + kDirZero10);
  auto zero_b = u32_at(bytes, off + kDirZero18);
  auto name_len = u32_at(bytes, off + kDirNameLen);
  if (!zero_a || !zero_b || !name_len || *zero_a != 0 || *zero_b != 0) return std::nullopt;
  if (*name_len == 0 || *name_len >= 500) return std::nullopt;
  size_t start = off + kDirLen;
  auto raw = slice(bytes, start, start + *name_len);
  if (!raw || !slice(bytes, start + *name_len, start + *name_len + 6)) return std::nullopt;
  return nibble_swap_name(*raw);
}

// ---------------------------------------------------------------------------
// Compound File Binary
// ---------------------------------------------------------------------------
constexpr uint32_t kFreeSect = 0xffffffff, kEndOfChain = 0xfffffffe, kNoStream = 0xffffffff;

struct CfbEntry {
  std::string name;
  uint8_t type = 0;  // 1 storage, 2 stream, 5 root
  uint32_t left = kNoStream, right = kNoStream, child = kNoStream;
  uint32_t start = 0;
  uint64_t size = 0;
};

struct CfbReader {
  ByteSpan image;
  size_t sector_size = 512;
  std::vector<uint32_t> fat, mini_fat;
  std::vector<CfbEntry> dir;
  Bytes mini_stream;

  std::optional<size_t> sector_offset(uint32_t sector) const {
    size_t off = (static_cast<size_t>(sector) + 1) * sector_size;
    if (off + sector_size > image.size() + sector_size) return std::nullopt;
    return off;
  }

  std::optional<Bytes> read_chain(uint32_t start, std::optional<uint64_t> size) const {
    Bytes out;
    uint32_t sector = start;
    size_t guard = 0;
    while (sector != kEndOfChain) {
      if (sector >= fat.size() || ++guard > fat.size() + 1) return std::nullopt;
      auto off = sector_offset(sector);
      if (!off) return std::nullopt;
      size_t end = std::min(*off + sector_size, image.size());
      if (*off > end) return std::nullopt;
      out.insert(out.end(), image.begin() + static_cast<std::ptrdiff_t>(*off),
                 image.begin() + static_cast<std::ptrdiff_t>(end));
      sector = fat[sector];
    }
    if (size) {
      if (out.size() < *size) return std::nullopt;
      out.resize(static_cast<size_t>(*size));
    }
    return out;
  }

  std::optional<Bytes> read_mini(uint32_t start, uint64_t size) const {
    Bytes out;
    uint32_t sector = start;
    size_t guard = 0;
    while (sector != kEndOfChain) {
      if (sector >= mini_fat.size() || ++guard > mini_fat.size() + 1) return std::nullopt;
      size_t off = static_cast<size_t>(sector) * 64;
      if (off + 64 > mini_stream.size()) return std::nullopt;
      out.insert(out.end(), mini_stream.begin() + static_cast<std::ptrdiff_t>(off),
                 mini_stream.begin() + static_cast<std::ptrdiff_t>(off + 64));
      sector = mini_fat[sector];
    }
    if (out.size() < size) return std::nullopt;
    out.resize(static_cast<size_t>(size));
    return out;
  }

  bool load(std::string* error) {
    auto major = u16_at(image, 26);
    auto shift = u16_at(image, 30);
    if (!major || !shift || (*shift != 9 && *shift != 12)) {
      *error = "unsupported CFB header";
      return false;
    }
    sector_size = size_t{1} << *shift;
    uint32_t fat_count = *u32_at(image, 44);
    uint32_t dir_start = *u32_at(image, 48);
    uint32_t mini_start = *u32_at(image, 60);
    uint32_t mini_count = *u32_at(image, 64);
    uint32_t difat_start = *u32_at(image, 68);
    uint32_t difat_count = *u32_at(image, 72);
    std::vector<uint32_t> fat_sectors;
    for (size_t i = 0; i < 109 && fat_sectors.size() < fat_count; ++i) {
      auto s = u32_at(image, 76 + i * 4);
      if (!s) return false;
      if (*s != kFreeSect) fat_sectors.push_back(*s);
    }
    uint32_t difat = difat_start;
    for (uint32_t n = 0; n < difat_count && difat != kEndOfChain && difat != kFreeSect; ++n) {
      size_t off = (static_cast<size_t>(difat) + 1) * sector_size;
      size_t per = sector_size / 4 - 1;
      for (size_t i = 0; i < per && fat_sectors.size() < fat_count; ++i) {
        auto s = u32_at(image, off + i * 4);
        if (!s) return false;
        if (*s != kFreeSect) fat_sectors.push_back(*s);
      }
      auto next = u32_at(image, off + per * 4);
      if (!next) return false;
      difat = *next;
    }
    for (uint32_t s : fat_sectors) {
      size_t off = (static_cast<size_t>(s) + 1) * sector_size;
      for (size_t i = 0; i < sector_size / 4; ++i) {
        auto v = u32_at(image, off + i * 4);
        if (!v) {
          *error = "CFB FAT sector out of range";
          return false;
        }
        fat.push_back(*v);
      }
    }
    auto dir_bytes = read_chain(dir_start, std::nullopt);
    if (!dir_bytes) {
      *error = "CFB directory chain is invalid";
      return false;
    }
    ByteSpan d(*dir_bytes);
    for (size_t off = 0; off + 128 <= d.size(); off += 128) {
      CfbEntry e;
      uint16_t name_len = *u16_at(d, off + 64);
      std::u16string wide;
      for (size_t i = 0; i + 2 <= name_len && i < 64; i += 2) {
        char16_t c = static_cast<char16_t>(*u16_at(d, off + i));
        if (c == 0) break;
        wide.push_back(c);
      }
      for (char16_t c : wide) {
        // Names in SOLIDWORKS documents are ASCII; keep other code units as '?'.
        e.name.push_back(c < 0x80 ? static_cast<char>(c) : '?');
      }
      e.type = d[off + 66];
      e.left = *u32_at(d, off + 68);
      e.right = *u32_at(d, off + 72);
      e.child = *u32_at(d, off + 76);
      e.start = *u32_at(d, off + 116);
      e.size = *u64_at(d, off + 120);
      if (sector_size == 512) e.size &= 0xffffffffu;
      dir.push_back(std::move(e));
    }
    if (dir.empty() || dir[0].type != 5) {
      *error = "CFB root entry missing";
      return false;
    }
    if (dir[0].size > 0) {
      auto ms = read_chain(dir[0].start, dir[0].size);
      if (!ms) {
        *error = "CFB mini stream invalid";
        return false;
      }
      mini_stream = std::move(*ms);
    }
    if (mini_count > 0 && mini_start != kEndOfChain) {
      auto mf = read_chain(mini_start, std::nullopt);
      if (!mf) {
        *error = "CFB mini FAT invalid";
        return false;
      }
      for (size_t i = 0; i + 4 <= mf->size(); i += 4) mini_fat.push_back(*u32_at(*mf, i));
    }
    return true;
  }
};

std::optional<Bytes> unwrap_zlb(ByteSpan payload) {
  if (payload.size() < 24 || !std::equal(kZlbMagic.begin(), kZlbMagic.end(), payload.begin()))
    return std::nullopt;
  uint32_t uncompressed = *u32_at(payload, 16);
  uint32_t member = *u32_at(payload, 20);
  if (uncompressed == 0 || member == 0) return std::nullopt;
  auto body = slice(payload, 24, 24 + static_cast<size_t>(member));
  if (!body) return std::nullopt;
  auto inflated = inflate_zlib_exact(*body, uncompressed);
  if (!inflated || inflated->second != member) return std::nullopt;
  return std::move(inflated->first);
}

std::optional<Container> read_compound(ByteSpan image, ContainerError* error) {
  CfbReader reader;
  reader.image = image;
  std::string message;
  if (!reader.load(&message)) {
    if (error) error->message = message;
    return std::nullopt;
  }
  Container out;
  out.compound = true;
  std::vector<bool> seen(reader.dir.size(), false);
  // Mirrors cadmpeg's walk: pop, push right, emit (recurse into storages),
  // push left.
  std::function<bool(uint32_t, const std::string*)> walk = [&](uint32_t root,
                                                              const std::string* parent) {
    if (root == kNoStream) return true;
    std::vector<uint32_t> pending{root};
    while (!pending.empty()) {
      uint32_t id = pending.back();
      pending.pop_back();
      if (id >= reader.dir.size() || seen[id]) return false;
      seen[id] = true;
      const CfbEntry& e = reader.dir[id];
      if (e.right != kNoStream) pending.push_back(e.right);
      std::string path = parent ? *parent + "/" + e.name : e.name;
      if (e.type == 1) {
        out.entry_names.push_back(path);
        if (!walk(e.child, &path)) return false;
      } else if (e.type == 2) {
        out.entry_names.push_back(path);
        std::optional<Bytes> data = Bytes{};
        if (e.size > 0) {
          data = e.size < 4096 ? reader.read_mini(e.start, e.size)
                               : reader.read_chain(e.start, e.size);
        }
        if (!data) return false;
        Section s;
        s.name = path;
        s.source_stream = path;
        s.ordinal = id;
        s.compound = true;
        if (auto decoded = unwrap_zlb(*data)) {
          s.payload = std::move(*decoded);
        } else {
          s.payload = std::move(*data);
        }
        out.sections.push_back(std::move(s));
      } else {
        return false;
      }
      if (e.left != kNoStream) pending.push_back(e.left);
    }
    return true;
  };
  if (!walk(reader.dir[0].child, nullptr)) {
    if (error) error->message = "CFB directory tree is invalid";
    return std::nullopt;
  }
  return out;
}

}  // namespace

std::string Section::site_key() const {
  return (compound ? "compound@" : "block@") + std::to_string(ordinal);
}

const Section* Container::find(std::string_view name) const {
  for (const auto& s : sections)
    if (s.name && iequals(*s.name, name)) return &s;
  return nullptr;
}

uint32_t crc32_of(ByteSpan data) {
  uLong crc = ::crc32(0L, Z_NULL, 0);
  size_t done = 0;
  while (done < data.size()) {
    uInt chunk = static_cast<uInt>(std::min<size_t>(data.size() - done, 1u << 30));
    crc = ::crc32(crc, data.data() + done, chunk);
    done += chunk;
  }
  return static_cast<uint32_t>(crc);
}

static std::optional<std::pair<Bytes, size_t>> inflate_impl(ByteSpan in, size_t expected,
                                                            int window_bits) {
  if (expected > kMaxUncompressed) return std::nullopt;
  Bytes out(expected);
  z_stream zs{};
  if (inflateInit2(&zs, window_bits) != Z_OK) return std::nullopt;
  zs.next_in = const_cast<Bytef*>(in.data());
  zs.avail_in = static_cast<uInt>(in.size());
  zs.next_out = out.data();
  zs.avail_out = static_cast<uInt>(out.size());
  int rc = inflate(&zs, Z_FINISH);
  size_t produced = zs.total_out;
  size_t consumed = zs.total_in;
  if (rc == Z_BUF_ERROR && zs.avail_out == 0) {
    // Output is full; the stream must end exactly here.
    uint8_t probe;
    zs.next_out = &probe;
    zs.avail_out = 1;
    rc = inflate(&zs, Z_FINISH);
    if (zs.total_out != expected) rc = Z_DATA_ERROR;
    consumed = zs.total_in;
  }
  inflateEnd(&zs);
  if (rc != Z_STREAM_END || produced != expected) return std::nullopt;
  return std::make_pair(std::move(out), consumed);
}

std::optional<Bytes> inflate_raw_exact(ByteSpan compressed, size_t expected) {
  auto r = inflate_impl(compressed, expected, -15);
  if (!r) return std::nullopt;
  return std::move(r->first);
}

std::optional<std::pair<Bytes, size_t>> inflate_zlib_exact(ByteSpan compressed, size_t expected) {
  return inflate_impl(compressed, expected, 15);
}

std::optional<Container> read_container(ByteSpan image, ContainerError* error) {
  if (image.size() >= 8 && std::equal(kCfbMagic.begin(), kCfbMagic.end(), image.begin()))
    return read_compound(image, error);
  if (image.size() < 8 + kMarker.size()) {
    if (error) error->message = "file too short for a SOLIDWORKS container";
    return std::nullopt;
  }
  auto version = u32be_at(image, 4);
  if (!version || *version != kNativeVersion) {
    if (error) error->message = "not a native SOLIDWORKS container";
    return std::nullopt;
  }
  Container out;
  out.version = *version;
  ByteSpan marker(kMarker);
  size_t i = 8;
  std::optional<size_t> invalid_local;
  while (i + kMarker.size() <= image.size()) {
    if (!starts_with_at(image, i, marker)) {
      // Skip quickly to the next possible marker byte.
      auto next = find_from(image, marker, i + 1, image.size());
      if (!next) break;
      i = *next;
      continue;
    }
    if (auto frame = read_frame(image, i); frame && frame->uncomp <= kMaxUncompressed) {
      auto payload = slice(image, frame->payload_start, frame->payload_end);
      std::optional<Bytes> inflated = inflate_raw_exact(*payload, frame->uncomp);
      if (inflated && inflated->size() == frame->uncomp && crc32_of(*inflated) == frame->crc) {
        Section s;
        auto preamble = slice(image, i + kBlockLen, frame->payload_start);
        if (preamble) s.name = nibble_swap_name(*preamble);
        s.source_stream = s.name ? *s.name : "block@" + std::to_string(i);
        s.ordinal = i;
        s.payload = std::move(*inflated);
        if (s.name) out.entry_names.push_back(*s.name);
        out.sections.push_back(std::move(s));
        i = frame->payload_end;
        continue;
      }
    }
    if (is_cache_cell(image, i)) {
      // Cache cells are retained without interpretation.
    } else if (auto name = directory_entry_name(image, i)) {
      out.entry_names.push_back(*name);
    } else if (auto frame = read_frame(image, i);
               frame && frame->uncomp <= kMaxUncompressed &&
               slice(image, i + kBlockLen, frame->payload_start)
                   .and_then([](ByteSpan p) { return nibble_swap_name(p); })
                   .has_value()) {
      if (!invalid_local || i < *invalid_local) invalid_local = i;
    }
    ++i;
  }
  if (invalid_local && out.sections.empty()) {
    if (error)
      error->message = "declared native local member at " + std::to_string(*invalid_local) +
                       " failed DEFLATE, size or CRC validation";
    return std::nullopt;
  }
  return out;
}

}  // namespace swglb
