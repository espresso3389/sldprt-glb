// SPDX-License-Identifier: Apache-2.0
// Assembly product structure, placement, visibility and shared display
// meshes. Ported from cadmpeg-codec-sldprt `assembly.rs`, `assembly/native.rs`
// and `assembly/display.rs` at cadmpeg 5af95421 with assembly-placement and shared-mesh
// extensions (Apache-2.0). Loss notes, product definitions, budgets and the virtual
// component body bookkeeping are not ported: they do not affect
// tessellations, appearance bindings or occurrences.
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_set>

#include "assembly_internal.h"
#include "errors.h"
#include "stages.h"

namespace swglb {
namespace assembly {
namespace {

// ---------------------------------------------------------------------------
// Text helpers
// ---------------------------------------------------------------------------

/// Next code point of valid UTF-8 text (all text here is validated or
/// produced by our own UTF-16 conversion).
uint32_t next_code_point(std::string_view s, size_t& i) {
  auto b = static_cast<unsigned char>(s[i]);
  int extra = b < 0x80 ? 0 : b < 0xE0 ? 1 : b < 0xF0 ? 2 : 3;
  uint32_t cp = extra == 0 ? b : extra == 1 ? (b & 0x1F) : extra == 2 ? (b & 0x0F) : (b & 0x07);
  ++i;
  for (int k = 0; k < extra && i < s.size(); ++k, ++i)
    cp = (cp << 6) | (static_cast<unsigned char>(s[i]) & 0x3F);
  return cp;
}

/// Rust `char::is_whitespace` (Unicode White_Space).
bool is_unicode_whitespace(uint32_t c) {
  return (c >= 0x09 && c <= 0x0D) || c == 0x20 || c == 0x85 || c == 0xA0 || c == 0x1680 ||
         (c >= 0x2000 && c <= 0x200A) || c == 0x2028 || c == 0x2029 || c == 0x202F ||
         c == 0x205F || c == 0x3000;
}

bool has_whitespace(std::string_view s) {
  for (size_t i = 0; i < s.size();)
    if (is_unicode_whitespace(next_code_point(s, i))) return true;
  return false;
}

/// Rust `str::trim().is_empty()` / `NonBlankString::new(..).is_none()`.
bool is_blank(std::string_view s) {
  for (size_t i = 0; i < s.size();)
    if (!is_unicode_whitespace(next_code_point(s, i))) return false;
  return true;
}

/// cadmpeg ids.rs `valid_key_text`: nonempty, no `#`, no Unicode whitespace.
bool valid_key(std::string_view s) {
  return !s.empty() && s.find('#') == std::string_view::npos && !has_whitespace(s);
}

std::string ascii_lower(std::string_view s) {
  std::string out(s);
  for (char& c : out)
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  return out;
}

bool valid_utf8(std::string_view s) {
  size_t i = 0;
  while (i < s.size()) {
    auto b = static_cast<unsigned char>(s[i]);
    if (b < 0x80) { ++i; continue; }
    int extra;
    uint32_t cp, min;
    if ((b & 0xE0) == 0xC0) { extra = 1; cp = b & 0x1F; min = 0x80; }
    else if ((b & 0xF0) == 0xE0) { extra = 2; cp = b & 0x0F; min = 0x800; }
    else if ((b & 0xF8) == 0xF0) { extra = 3; cp = b & 0x07; min = 0x10000; }
    else return false;
    if (s.size() - i <= static_cast<size_t>(extra)) return false;
    for (int k = 1; k <= extra; ++k) {
      auto c = static_cast<unsigned char>(s[i + k]);
      if ((c & 0xC0) != 0x80) return false;
      cp = (cp << 6) | (c & 0x3F);
    }
    if (cp < min || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) return false;
    i += extra + 1;
  }
  return true;
}

void push_utf8(std::string& out, uint32_t cp) {
  if (cp < 0x80) {
    out += static_cast<char>(cp);
  } else if (cp < 0x800) {
    out += static_cast<char>(0xC0 | (cp >> 6));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else if (cp < 0x10000) {
    out += static_cast<char>(0xE0 | (cp >> 12));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else {
    out += static_cast<char>(0xF0 | (cp >> 18));
    out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  }
}

/// UTF-16 code units to UTF-8. `lossy` replaces unpaired surrogates with
/// U+FFFD (Rust `String::from_utf16_lossy`); otherwise they fail.
std::optional<std::string> utf16_to_utf8(const std::vector<uint16_t>& units, bool lossy) {
  std::string out;
  out.reserve(units.size());
  for (size_t i = 0; i < units.size(); ++i) {
    uint32_t u = units[i];
    if (u >= 0xD800 && u <= 0xDBFF && i + 1 < units.size() && units[i + 1] >= 0xDC00 &&
        units[i + 1] <= 0xDFFF) {
      push_utf8(out, 0x10000 + ((u - 0xD800) << 10) + (units[i + 1] - 0xDC00));
      ++i;
    } else if (u >= 0xD800 && u <= 0xDFFF) {
      if (!lossy) return std::nullopt;
      push_utf8(out, 0xFFFD);
    } else {
      push_utf8(out, u);
    }
  }
  return out;
}

/// cadmpeg container.rs `xml_text`.
std::optional<std::string> xml_text(ByteSpan bytes) {
  if (!bytes.empty() && bytes[0] == 0x86) bytes = bytes.subspan(1);
  if (bytes.size() >= 2 && bytes[0] == 0xff && bytes[1] == 0xfe) {
    std::vector<uint16_t> units;
    units.reserve((bytes.size() - 2) / 2);
    for (size_t at = 2; at + 2 <= bytes.size(); at += 2)
      units.push_back(static_cast<uint16_t>(bytes[at] | (bytes[at + 1] << 8)));
    return utf16_to_utf8(units, true);
  }
  std::string_view text(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  if (!valid_utf8(text)) return std::nullopt;
  return std::string(text);
}

constexpr std::string_view kTreeSection = "swXmlContents/COMPINSTANCETREE";
constexpr std::string_view kWrappedSuffix = "__ZLB";
constexpr std::string_view kEmbeddedPrefix = "ImportedComp/";
constexpr std::string_view kVirtualPrefix = "VirtualComp/";
constexpr std::string_view kVirtualSuffix = "-sldprt";
constexpr double kMetresToMillimetres = 1000.0;
constexpr double kEpsHomogeneousColumn = 1e-9;
/// cadmpeg DecodePolicy `max_recursion_depth` (service policy).
constexpr int kMaxNesting = 128;

/// cadmpeg container.rs `unwrapped_name`.
std::string_view unwrapped_name(std::string_view name) {
  if (name.size() >= kWrappedSuffix.size() &&
      iequals(name.substr(name.size() - kWrappedSuffix.size()), kWrappedSuffix))
    return name.substr(0, name.size() - kWrappedSuffix.size());
  return name;
}

/// cadmpeg container.rs `is_embedded_document`.
bool is_embedded_document(std::string_view name) {
  name = unwrapped_name(name);
  return name.starts_with(kEmbeddedPrefix) || iends_with(name, ".sldprt") ||
         iends_with(name, ".sldasm");
}

/// cadmpeg container.rs `virtual_component_key`.
std::optional<std::string> virtual_component_key(std::string_view raw) {
  std::string name(unwrapped_name(raw));
  std::replace(name.begin(), name.end(), '\\', '/');
  if (name.size() < kVirtualPrefix.size() ||
      !iequals(std::string_view(name).substr(0, kVirtualPrefix.size()), kVirtualPrefix))
    return std::nullopt;
  std::string key = ascii_lower(std::string_view(name).substr(kVirtualPrefix.size()));
  if (key.size() > kVirtualSuffix.size() && key.ends_with(kVirtualSuffix)) return key;
  return std::nullopt;
}

/// cadmpeg assembly.rs `is_mate_list`.
bool is_mate_list(std::string_view raw) {
  std::string name = ascii_lower(unwrapped_name(raw));
  std::replace(name.begin(), name.end(), '\\', '/');
  constexpr std::string_view prefix = "contents/config-", suffix = "-mateslist";
  if (!name.starts_with(prefix)) return false;
  std::string_view rest = std::string_view(name).substr(prefix.size());
  if (!rest.ends_with(suffix)) return false;
  std::string_view index = rest.substr(0, rest.size() - suffix.size());
  return !index.empty() &&
         std::all_of(index.begin(), index.end(), [](char c) { return c >= '0' && c <= '9'; });
}

/// cadmpeg native.rs `document_stem`.
std::optional<std::string> document_stem(std::string_view name) {
  size_t slash = name.find_last_of("\\/");
  std::string_view file = slash == std::string_view::npos ? name : name.substr(slash + 1);
  size_t dot = file.rfind('.');
  std::string_view stem = dot == std::string_view::npos ? file : file.substr(0, dot);
  if (is_blank(stem)) return std::nullopt;
  return std::string(stem);
}

/// Rust `str::parse::<u32>()`.
bool parses_u32(std::string_view s) {
  if (!s.empty() && s[0] == '+') s.remove_prefix(1);
  if (s.empty()) return false;
  uint32_t v;
  auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  return ec == std::errc() && p == s.data() + s.size();
}

/// Rust `str::parse::<f64>()` restricted to finite values.
std::optional<double> parse_finite(std::string_view s) {
  if (!s.empty() && s[0] == '+') {
    s.remove_prefix(1);
    if (!s.empty() && (s[0] == '+' || s[0] == '-')) return std::nullopt;
  }
  if (s.empty()) return std::nullopt;
  double v;
  auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (ec != std::errc() || p != s.data() + s.size() || !std::isfinite(v)) return std::nullopt;
  return v;
}

// ---------------------------------------------------------------------------
// Minimal XML reader (stands in for roxmltree with default options: no DTD,
// predefined and character entity references only). Only what the component
// tree walk needs is retained: element local names, attributes by qualified
// name, and element children. Any well-formedness error fails the parse.
// ---------------------------------------------------------------------------
struct XmlElement {
  std::string name;  ///< Local name.
  std::vector<std::pair<std::string, std::string>> attributes;  ///< qualified name -> value
  std::vector<size_t> children;  ///< Element children (indices).
  const std::string* attribute(std::string_view key) const {
    for (const auto& [k, v] : attributes)
      if (k == key) return &v;
    return nullptr;
  }
};

class XmlReader {
 public:
  explicit XmlReader(std::string_view text) : s_(text) {}

  /// Parse the document; elements in document order (root at index 0).
  std::optional<std::vector<XmlElement>> parse() {
    if (s_.starts_with("\xEF\xBB\xBF")) pos_ = 3;
    if (!misc(true)) return std::nullopt;
    if (!peek('<')) return std::nullopt;
    if (!element()) return std::nullopt;
    if (!misc(false)) return std::nullopt;
    if (pos_ != s_.size()) return std::nullopt;
    return std::move(elements_);
  }

 private:
  bool peek(char c) const { return pos_ < s_.size() && s_[pos_] == c; }
  bool starts(std::string_view t) const { return s_.substr(pos_).starts_with(t); }
  static bool space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }
  void skip_space() {
    while (pos_ < s_.size() && space(s_[pos_])) ++pos_;
  }
  bool skip_past(std::string_view end) {
    size_t at = s_.find(end, pos_);
    if (at == std::string_view::npos) return false;
    pos_ = at + end.size();
    return true;
  }

  /// Prolog / epilog: whitespace, comments, processing instructions.
  bool misc(bool prolog) {
    for (;;) {
      skip_space();
      if (starts("<!--")) {
        pos_ += 4;
        if (!skip_past("-->")) return false;
      } else if (starts("<?")) {
        pos_ += 2;
        if (!skip_past("?>")) return false;
      } else if (starts("<!")) {
        return false;  // DOCTYPE (roxmltree default: DTD not allowed) or junk.
      } else {
        return pos_ == s_.size() || (prolog && peek('<'));
      }
    }
  }

  std::optional<std::string_view> name() {
    size_t start = pos_;
    while (pos_ < s_.size()) {
      char c = s_[pos_];
      if (space(c) || c == '/' || c == '>' || c == '=' || c == '<' || c == '"' || c == '\'' ||
          c == '&')
        break;
      ++pos_;
    }
    if (pos_ == start) return std::nullopt;
    std::string_view n = s_.substr(start, pos_ - start);
    if (n[0] == '-' || n[0] == '.' || (n[0] >= '0' && n[0] <= '9')) return std::nullopt;
    return n;
  }

  /// Decode one entity reference starting at `&`.
  bool entity(std::string& out) {
    size_t semi = s_.find(';', pos_);
    if (semi == std::string_view::npos) return false;
    std::string_view ref = s_.substr(pos_ + 1, semi - pos_ - 1);
    pos_ = semi + 1;
    if (ref == "lt") out += '<';
    else if (ref == "gt") out += '>';
    else if (ref == "amp") out += '&';
    else if (ref == "apos") out += '\'';
    else if (ref == "quot") out += '"';
    else if (ref.size() >= 2 && ref[0] == '#') {
      bool hex = ref[1] == 'x';
      std::string_view digits = ref.substr(hex ? 2 : 1);
      if (digits.empty()) return false;
      uint32_t cp = 0;
      auto [p, ec] =
          std::from_chars(digits.data(), digits.data() + digits.size(), cp, hex ? 16 : 10);
      if (ec != std::errc() || p != digits.data() + digits.size()) return false;
      if (cp == 0 || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) return false;
      push_utf8(out, cp);
    } else {
      return false;
    }
    return true;
  }

  std::optional<std::string> attribute_value() {
    if (!peek('"') && !peek('\'')) return std::nullopt;
    char quote = s_[pos_++];
    std::string out;
    while (pos_ < s_.size() && s_[pos_] != quote) {
      char c = s_[pos_];
      if (c == '<') return std::nullopt;
      if (c == '&') {
        if (!entity(out)) return std::nullopt;
        continue;
      }
      // Line-end normalisation, then attribute-value normalisation.
      if (c == '\r') {
        out += ' ';
        ++pos_;
        if (peek('\n')) ++pos_;
        continue;
      }
      out += (c == '\t' || c == '\n') ? ' ' : c;
      ++pos_;
    }
    if (!peek(quote)) return std::nullopt;
    ++pos_;
    return out;
  }

  static std::string local_name(std::string_view qualified) {
    size_t colon = qualified.find(':');
    return std::string(colon == std::string_view::npos ? qualified : qualified.substr(colon + 1));
  }

  /// One start tag; returns false on error. `open` is set when the element
  /// has content (not self-closing).
  bool start_tag(size_t index, std::string_view& qname, bool& open) {
    ++pos_;  // '<'
    auto n = name();
    if (!n) return false;
    qname = *n;
    elements_[index].name = local_name(*n);
    for (;;) {
      bool had_space = pos_ < s_.size() && space(s_[pos_]);
      skip_space();
      if (starts("/>")) {
        pos_ += 2;
        open = false;
        return true;
      }
      if (peek('>')) {
        ++pos_;
        open = true;
        return true;
      }
      if (!had_space) return false;
      auto key = name();
      if (!key) return false;
      skip_space();
      if (!peek('=')) return false;
      ++pos_;
      skip_space();
      auto value = attribute_value();
      if (!value) return false;
      std::string k(*key);
      for (const auto& [existing, _] : elements_[index].attributes)
        if (existing == k) return false;
      // Namespace declarations are not attributes in roxmltree.
      if (k == "xmlns" || k.starts_with("xmlns:")) continue;
      elements_[index].attributes.emplace_back(std::move(k), std::move(*value));
    }
  }

  /// The root element and its content, iteratively.
  bool element() {
    struct Open {
      size_t index;
      std::string_view qname;
    };
    std::vector<Open> stack;
    auto open_element = [&](std::optional<size_t> parent) -> bool {
      size_t index = elements_.size();
      elements_.emplace_back();
      if (parent) elements_[*parent].children.push_back(index);
      std::string_view qname;
      bool open = false;
      if (!start_tag(index, qname, open)) return false;
      if (open) stack.push_back({index, qname});
      return true;
    };
    if (!open_element(std::nullopt)) return false;
    while (!stack.empty()) {
      if (pos_ >= s_.size()) return false;
      if (starts("</")) {
        pos_ += 2;
        auto n = name();
        if (!n || *n != stack.back().qname) return false;
        skip_space();
        if (!peek('>')) return false;
        ++pos_;
        stack.pop_back();
      } else if (starts("<!--")) {
        pos_ += 4;
        if (!skip_past("-->")) return false;
      } else if (starts("<![CDATA[")) {
        pos_ += 9;
        if (!skip_past("]]>")) return false;
      } else if (starts("<?")) {
        pos_ += 2;
        if (!skip_past("?>")) return false;
      } else if (starts("<!")) {
        return false;
      } else if (peek('<')) {
        if (!open_element(stack.back().index)) return false;
      } else if (peek('&')) {
        std::string scratch;
        if (!entity(scratch)) return false;
      } else {
        if (starts("]]>")) return false;
        ++pos_;
      }
    }
    return true;
  }

  std::string_view s_;
  size_t pos_ = 0;
  std::vector<XmlElement> elements_;
};

// ---------------------------------------------------------------------------
// Exact-enough dot products (cadmpeg math/sum.rs `finite_dot`): the plain f64
// sum when it is not a cancellation, else an error-free expansion sum.
// ---------------------------------------------------------------------------
template <size_t N>
std::optional<double> finite_dot(const std::array<double, N>& a, const std::array<double, N>& b) {
  std::array<double, N> products;
  for (size_t i = 0; i < N; ++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i])) return std::nullopt;
    products[i] = a[i] * b[i];
  }
  // fast_dot
  bool fast = true;
  double scale = 0;
  for (size_t i = 0; i < N; ++i) {
    if (!std::isfinite(products[i])) fast = false;
    if (a[i] != 0 && b[i] != 0 && std::fabs(products[i]) < std::numeric_limits<double>::min())
      fast = false;
    scale = std::max(scale, std::fabs(products[i]));
  }
  if (fast) {
    if (scale == 0) return 0.0;
    double sum = -0.0;
    for (double p : products) sum += p;
    if (std::isfinite(sum) && std::fabs(sum) / scale >= std::numeric_limits<double>::epsilon())
      return sum;
  }
  // Exact path: Shewchuk grow-expansion over TwoProduct terms.
  std::vector<double> expansion;
  auto grow = [&](double x) {
    std::vector<double> next;
    next.reserve(expansion.size() + 1);
    double q = x;
    for (double e : expansion) {
      double s = q + e;
      double bv = s - q;
      double err = (q - (s - bv)) + (e - bv);
      if (err != 0) next.push_back(err);
      q = s;
    }
    if (q != 0) next.push_back(q);
    expansion.swap(next);
  };
  for (size_t i = 0; i < N; ++i) {
    double p = a[i] * b[i];
    if (!std::isfinite(p)) return std::nullopt;
    double e = std::fma(a[i], b[i], -p);
    grow(p);
    if (e != 0) grow(e);
  }
  double sum = 0;
  for (double e : expansion) sum += e;
  if (!std::isfinite(sum)) return std::nullopt;
  return sum == 0 ? 0.0 : sum;
}

}  // namespace

// ---------------------------------------------------------------------------
// Transform (cadmpeg-ir transform.rs)
// ---------------------------------------------------------------------------
bool Transform::is_proper_rigid() const {
  constexpr double eps = 1.0e-9;
  const double x[3] = {rows[0][0], rows[1][0], rows[2][0]};
  const double y[3] = {rows[0][1], rows[1][1], rows[2][1]};
  const double z[3] = {rows[0][2], rows[1][2], rows[2][2]};
  for (const auto& r : rows)
    for (double v : r)
      if (!std::isfinite(v)) return false;
  auto dot = [](const double* l, const double* r) { return l[0] * r[0] + l[1] * r[1] + l[2] * r[2]; };
  const double cross[3] = {x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2],
                           x[0] * y[1] - x[1] * y[0]};
  return std::fabs(dot(x, x) - 1.0) <= eps && std::fabs(dot(y, y) - 1.0) <= eps &&
         std::fabs(dot(z, z) - 1.0) <= eps && std::fabs(dot(x, y)) <= eps &&
         std::fabs(dot(x, z)) <= eps && std::fabs(dot(y, z)) <= eps &&
         std::fabs(dot(cross, z) - 1.0) <= eps;
}

std::optional<Transform> Transform::compose(const Transform& right) const {
  auto full = [](const Transform& t, size_t r) -> std::array<double, 4> {
    if (r == 3) return {0, 0, 0, 1};
    return {t.rows[r][0], t.rows[r][1], t.rows[r][2], t.rows[r][3]};
  };
  Transform out;
  for (size_t r = 0; r < 3; ++r) {
    std::array<double, 4> left = full(*this, r);
    for (size_t c = 0; c < 4; ++c) {
      std::array<double, 4> column;
      for (size_t k = 0; k < 4; ++k) column[k] = full(right, k)[c];
      auto v = finite_dot(left, column);
      if (!v) return std::nullopt;
      out.rows[r][c] = *v;
    }
  }
  return out;
}

std::optional<Vec3> Transform::apply_point(const Vec3& p) const {
  std::array<double, 4> v{p.x, p.y, p.z, 1.0};
  double out[3];
  for (size_t r = 0; r < 3; ++r) {
    auto d = finite_dot(std::array<double, 4>{rows[r][0], rows[r][1], rows[r][2], rows[r][3]}, v);
    if (!d) return std::nullopt;
    out[r] = *d;
  }
  return Vec3{out[0], out[1], out[2]};
}

std::optional<Vec3> Transform::apply_vector(const Vec3& p) const {
  std::array<double, 3> v{p.x, p.y, p.z};
  double out[3];
  for (size_t r = 0; r < 3; ++r) {
    auto d = finite_dot(std::array<double, 3>{rows[r][0], rows[r][1], rows[r][2]}, v);
    if (!d) return std::nullopt;
    out[r] = *d;
  }
  return Vec3{out[0], out[1], out[2]};
}

// ---------------------------------------------------------------------------
// swTransform (assembly.rs `placement`, `placement_rows`)
// ---------------------------------------------------------------------------
namespace {
std::optional<Transform> placement_rows(std::string_view text) {
  double values[16];
  size_t count = 0;
  size_t i = 0;
  auto ascii_ws = [](char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\f' || c == '\r'; };
  while (i < text.size()) {
    while (i < text.size() && ascii_ws(text[i])) ++i;
    if (i >= text.size()) break;
    size_t start = i;
    while (i < text.size() && !ascii_ws(text[i])) ++i;
    if (count >= 16) return std::nullopt;
    auto v = parse_finite(text.substr(start, i - start));
    if (!v) return std::nullopt;
    values[count++] = *v;
  }
  if (count != 16) return std::nullopt;
  const double homogeneous[4] = {values[3], values[7], values[11], values[15] - 1.0};
  for (double h : homogeneous)
    if (std::fabs(h) > kEpsHomogeneousColumn) return std::nullopt;
  Transform t;
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) t.rows[r][c] = values[c * 4 + r];
    t.rows[r][3] = values[12 + r] * kMetresToMillimetres;
  }
  for (const auto& r : t.rows)  // Transform::affine
    for (double v : r)
      if (!std::isfinite(v)) return std::nullopt;
  return t;
}
}  // namespace

std::optional<Transform> placement(std::string_view text) {
  auto t = placement_rows(text);
  if (!t || !t->is_proper_rigid()) return std::nullopt;
  return t;
}

// ---------------------------------------------------------------------------
// Component tree walk (assembly.rs `native_project`, `Walk`)
// ---------------------------------------------------------------------------
namespace {

constexpr std::string_view kOccurrencePrefix = "sldprt:assembly:occurrence#";
constexpr std::string_view kDefinitionPrefix = "sldprt:assembly:model#";

struct TreeModel {
  const std::string* file = nullptr;
  const std::string* configuration = nullptr;
  std::vector<const XmlElement*> references;
};
struct TreeFile {
  const std::string* path = nullptr;
  const std::string* doc_type = nullptr;
};

struct Walk {
  const std::map<std::string, TreeFile>& files;
  const std::map<std::string, TreeModel>& models;
  const std::map<std::string, std::optional<std::string>>& virtual_sections;
  std::optional<std::string> holder_stem;
  std::set<std::string> definitions;
  std::unordered_set<std::string> occurrences;
  Projection& out;
  int depth = 0;

  const std::string* file_path(const std::string& model) const {
    auto m = models.find(model);
    if (m == models.end() || !m->second.file) return nullptr;
    auto f = files.find(*m->second.file);
    if (f == files.end()) return nullptr;
    return f->second.path;
  }

  std::optional<std::string> model_name(const std::string& model) const {
    const std::string* path = file_path(model);
    if (!path) return std::nullopt;
    return document_stem(*path);
  }

  bool is_assembly_model(const TreeModel& data) const {
    const std::string* doc_type = nullptr;
    if (data.file) {
      auto f = files.find(*data.file);
      if (f != files.end()) doc_type = f->second.doc_type;
    }
    if (doc_type) return iequals(*doc_type, "ASSEMBLY");
    return !data.references.empty();
  }

  /// Walk::virtual_site
  std::optional<std::string> virtual_site(const std::string& model,
                                          const XmlElement& reference) const {
    if (!holder_stem) return std::nullopt;
    std::optional<std::string> stem = model_name(model);
    if (!stem) {
      const std::string* n = reference.attribute("swName");
      if (!n) return std::nullopt;
      stem = *n;
    }
    for (size_t at = stem->find('^'); at != std::string::npos; at = stem->find('^', at + 1)) {
      std::string_view name = std::string_view(*stem).substr(0, at);
      if (name.empty()) continue;
      std::string_view assembly = std::string_view(*stem).substr(at + 1);
      if (!iequals(assembly, *holder_stem)) continue;
      auto it = virtual_sections.find(ascii_lower(name) + "-sldprt");
      if (it == virtual_sections.end() || !it->second) continue;
      return it->second;
    }
    return std::nullopt;
  }

  /// Walk::definition / virtual_definition: the definition id of `model`, or
  /// nullopt for an invalid identity key. `site` records a virtual definition.
  std::optional<std::string> definition(const std::string& model,
                                        const std::optional<std::string>& site) {
    if (!valid_key(model)) return std::nullopt;
    if (definitions.insert(model).second && site) out.virtual_sites.push_back(*site);
    return std::string(kDefinitionPrefix) + model;
  }

  /// Walk::children
  void children(const std::string& model, const std::string& parent,
                const std::optional<std::string>& parent_key, std::set<std::string>& path) {
    if (++depth > kMaxNesting) throw DecodeError("project SolidWorks component tree: nesting limit");
    auto m = models.find(model);
    std::vector<const XmlElement*> references;
    if (m != models.end()) references = m->second.references;
    for (const XmlElement* reference : references) {
      auto attr = [&](std::string_view k) { return reference->attribute(k); };
      auto is = [&](std::string_view k, std::string_view v) {
        const std::string* a = attr(k);
        return a && *a == v;
      };
      if (is("swSuppressed", "YES")) continue;
      const std::string* id = attr("id");
      const std::string* child = attr("swModelRef");
      if (!id || !child) continue;
      auto child_model = models.find(*child);
      if (child_model == models.end()) continue;
      const std::string* text = attr("swTransform");
      if (!text) continue;
      std::optional<Transform> transform = placement(*text);
      if (!transform) continue;  // unresolved or mirrored
      if (path.contains(*child)) continue;
      // NativePlacement::tree_occurrence_id
      if (!valid_key(*id)) continue;
      std::string key = parent_key ? *parent_key + "-" + *id : *id;
      std::string occurrence = std::string(kOccurrencePrefix) + key;
      if (occurrences.contains(occurrence)) continue;
      bool expands = is_assembly_model(child_model->second);
      Prototype prototype;
      if (expands) {
        auto def = definition(*child, std::nullopt);
        if (!def) continue;
        prototype.kind = Prototype::Kind::Local;
        prototype.definition = *def;
      } else if (is("swIsVirtualComponent", "YES")) {
        std::optional<std::string> def;
        if (auto site = virtual_site(*child, *reference)) def = definition(*child, site);
        if (def) {
          prototype.kind = Prototype::Kind::Local;
          prototype.definition = *def;
        } else {
          prototype.kind = Prototype::Kind::Unresolved;
        }
      } else {
        // NativePrototype::external
        const std::string* p = file_path(*child);
        prototype.kind = Prototype::Kind::External;
        prototype.document = p && !is_blank(*p) ? *p : std::string();
        const std::string* configuration = child_model->second.configuration;
        if (configuration && !is_blank(*configuration)) prototype.object = *configuration;
      }
      // instance_name
      std::string name = attr("swName") ? *attr("swName") : std::string();
      if (const std::string* number = attr("swReferenceNumber")) name += "-" + *number;
      std::optional<bool> visible;
      if (is("swHidden", "YES")) visible = false;
      else if (is("swHidden", "NO")) visible = true;
      occurrences.insert(occurrence);
      out.occurrences.push_back(
          PlanOccurrence{occurrence, parent, *transform, std::move(name), visible, prototype});
      if (expands) {
        path.insert(*child);
        children(*child, occurrence, key, path);
        path.erase(*child);
      }
    }
    --depth;
  }
};

}  // namespace

Projection project(std::string_view text,
                   const std::map<std::string, std::optional<std::string>>& virtual_sections) {
  Projection out;
  XmlReader reader(text);
  auto parsed = reader.parse();
  if (!parsed || parsed->empty()) return out;  // not well-formed: structure lost
  const std::vector<XmlElement>& elements = *parsed;
  if (elements[0].name != "swSolidWorks") return out;

  std::map<std::string, TreeFile> files;
  std::map<std::string, TreeModel> models;
  bool repeated = false;
  std::vector<const XmlElement*> configurations;
  for (const XmlElement& node : elements) {  // document order
    if (node.name == "swFile") {
      if (const std::string* id = node.attribute("id")) {
        TreeFile file{node.attribute("swPath"), node.attribute("swDocType")};
        auto [it, inserted] = files.insert_or_assign(*id, file);
        repeated |= !inserted;
      }
    } else if (node.name == "swModel") {
      if (const std::string* id = node.attribute("id")) {
        TreeModel model{node.attribute("swFileRef"), node.attribute("swConfigurationName"), {}};
        for (size_t child : node.children)
          if (elements[child].name == "swReference") model.references.push_back(&elements[child]);
        auto [it, inserted] = models.insert_or_assign(*id, std::move(model));
        repeated |= !inserted;
      }
    } else if (node.name == "swConfiguration") {
      configurations.push_back(&node);
    }
  }
  if (repeated) return out;

  std::vector<std::string> active;
  for (const XmlElement* node : configurations) {
    const std::string* recent = node->attribute("swMostRecentConfiguration");
    const std::string* ref = node->attribute("swModelRef");
    if (recent && *recent == "YES" && ref) active.push_back(*ref);
  }
  if (active.size() != 1) return out;
  const std::string root_model = active[0];
  if (!models.contains(root_model)) return out;

  Walk walk{files, models, virtual_sections, std::nullopt, {}, {}, out};
  walk.holder_stem = walk.model_name(root_model);
  auto root_definition = walk.definition(root_model, std::nullopt);
  if (!root_definition) return out;
  // NativePlacement::tree_root
  std::string root_id = std::string(kOccurrencePrefix) + "root";
  Prototype root_prototype;
  root_prototype.kind = Prototype::Kind::Local;
  root_prototype.definition = *root_definition;
  walk.occurrences.insert(root_id);
  out.occurrences.push_back(PlanOccurrence{root_id, std::nullopt, Transform::identity(),
                                           walk.model_name(root_model), std::nullopt,
                                           root_prototype});
  std::set<std::string> path{root_model};
  walk.children(root_model, root_id, std::nullopt, path);
  return out;
}

// ---------------------------------------------------------------------------
// Display placement (assembly/display.rs)
// ---------------------------------------------------------------------------
namespace {

using OccurrenceIndex = std::unordered_map<std::string_view, size_t>;

OccurrenceIndex index_occurrences(const std::vector<PlanOccurrence>& occurrences) {
  OccurrenceIndex index;
  index.reserve(occurrences.size());
  for (size_t i = 0; i < occurrences.size(); ++i) index.emplace(occurrences[i].id, i);  // first wins
  return index;
}

/// display.rs `world`.
Placement world_indexed(const PlanOccurrence& occurrence, const std::vector<PlanOccurrence>& rows,
                        const OccurrenceIndex& index) {
  std::vector<const PlanOccurrence*> chain{&occurrence};
  const PlanOccurrence* current = &occurrence;
  while (current->parent) {
    auto it = index.find(*current->parent);
    if (it == index.end()) throw DecodeError("assembly display parent is absent");
    current = &rows[it->second];
    for (const PlanOccurrence* o : chain)
      if (o->id == current->id) throw DecodeError("assembly display parent cycle");
    chain.push_back(current);
  }
  std::reverse(chain.begin(), chain.end());
  Placement out;
  std::vector<std::string> path;
  for (size_t i = 0; i < chain.size(); ++i) {
    const PlanOccurrence& item = *chain[i];
    if (!item.transform.is_proper_rigid())
      throw DecodeError("assembly display requires rigid occurrence placement");
    auto composed = out.transform.compose(item.transform);
    if (!composed) throw DecodeError("assembly display transform is not finite");
    out.transform = *composed;
    out.visible &= item.visible != std::optional<bool>(false);
    if (i != 0) {
      const PlanOccurrence& parent = *chain[i - 1];
      if (!parent.name) throw DecodeError("assembly display parent has no name");
      std::string_view document = *parent.name;
      if (i != 1) {
        size_t dash = document.rfind('-');
        if (dash != std::string_view::npos && parses_u32(document.substr(dash + 1)))
          document = document.substr(0, dash);
      }
      if (!item.name) throw DecodeError("assembly display occurrence has no name");
      path.push_back(*item.name + "@" + std::string(document));
    }
  }
  for (size_t i = 0; i < path.size(); ++i) {
    if (i) out.path += '/';
    out.path += path[i];
  }
  return out;
}

/// framed_string.rs `read_framed_string` at `at` (marker already checked).
/// Returns the text and the position after it, or nullopt for a failed read.
std::optional<std::pair<std::string, size_t>> read_framed_string(ByteSpan data, size_t at) {
  size_t pos = at + 3;
  auto first = u8_at(data, pos);
  if (!first) return std::nullopt;
  ++pos;
  uint64_t count = *first;
  if (*first == 0xff) {
    auto second = u16_at(data, pos);
    if (!second) return std::nullopt;
    pos += 2;
    count = *second;
    if (*second == 0xffff) {
      auto far = u32_at(data, pos);
      if (!far) return std::nullopt;
      pos += 4;
      count = *far;
    }
  }
  if (count > (data.size() - pos) / 2) return std::nullopt;
  std::vector<uint16_t> units(static_cast<size_t>(count));
  for (size_t k = 0; k < units.size(); ++k)
    units[k] = static_cast<uint16_t>(data[pos + 2 * k] | (data[pos + 2 * k + 1] << 8));
  pos += units.size() * 2;
  auto text = utf16_to_utf8(units, false);
  if (!text) return std::nullopt;
  return std::make_pair(std::move(*text), pos);
}

}  // namespace

Placement world(const PlanOccurrence& occurrence, const std::vector<PlanOccurrence>& occurrences) {
  return world_indexed(occurrence, occurrences, index_occurrences(occurrences));
}

DisplayPlan read_display(const std::vector<PlanOccurrence>& occurrences, const Container& container,
                         const Model& model) {
  DisplayPlan plan;
  if (occurrences.empty()) return plan;
  OccurrenceIndex index = index_occurrences(occurrences);
  std::vector<Placement>& worlds = plan.worlds;
  worlds.reserve(occurrences.size());
  std::map<std::string, size_t> paths;  // path -> owner index in plan.owners
  for (const PlanOccurrence& occurrence : occurrences) {
    worlds.push_back(world_indexed(occurrence, occurrences, index));
    const Placement& placement = worlds.back();
    if (placement.path.empty()) continue;
    if (!paths.emplace(placement.path, plan.owners.size()).second)
      throw DecodeError("assembly display occurrence path is ambiguous");
    plan.owners.push_back(placement);
  }
  plan.instances.resize(plan.owners.size());

  // Descriptor tables per section, from the tessellations pushed for them.
  std::unordered_map<size_t, std::vector<std::pair<size_t, ByteRange>>> tables;
  for (const Tessellation& t : model.tessellations)
    tables[t.section_index].emplace_back(t.table_index, t.table);

  static constexpr std::string_view kShdClass = "uoTempAssemblySHDData_c";
  std::set<std::string> saved;
  std::vector<bool> owns(plan.owners.size(), false);
  for (size_t s = 0; s < container.sections.size(); ++s) {
    const Section& section = container.sections[s];
    if (section.name != "Contents/DisplayLists") continue;
    ByteSpan data(section.payload);
    if (!contains(data, kShdClass)) continue;
    std::vector<std::pair<size_t, ByteRange>> faces;
    if (auto it = tables.find(s); it != tables.end()) faces = it->second;
    std::sort(faces.begin(), faces.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    size_t start = 0;
    std::optional<size_t> owner;
    for (size_t i = 0; i <= faces.size(); ++i) {
      size_t end = i < faces.size() ? faces[i].second.start : data.size();
      end = std::min(end, data.size());
      for (size_t at = start; at < end;) {
        const void* hit = std::memchr(data.data() + at, 0xff, end - at);
        if (!hit) break;
        at = static_cast<size_t>(static_cast<const uint8_t*>(hit) - data.data());
        size_t here = at++;
        if (here + 3 > data.size() || data[here + 1] != 0xfe || data[here + 2] != 0xff ||
            here < 2)
          continue;
        auto tag = u16_at(data, here - 2);
        if (!tag || !(*tag & 0x8000)) continue;
        auto name = read_framed_string(data, here);
        if (!name) continue;
        auto next = u8_at(data, name->second);
        if (name->first.find('@') == std::string::npos || !next || *next > 1) continue;
        auto found = paths.find(name->first);
        owner = found == paths.end() ? std::nullopt : std::optional<size_t>(found->second);
        if (owner) saved.insert(std::move(name->first));
      }
      if (i == faces.size()) break;
      if (!owner) throw DecodeError("assembly display mesh has no unique active occurrence");
      plan.table_owner["sldprt:displaylist:record#" + std::to_string(section.ordinal) + ":" +
                       std::to_string(faces[i].first)] = *owner;
      owns[*owner] = true;
      start = faces[i].second.end;
    }
  }

  // Shared-mesh expansion.
  std::unordered_set<std::string_view> parents;
  for (const PlanOccurrence& o : occurrences)
    if (o.parent) parents.insert(*o.parent);
  auto owner_of = [&](const std::string& path) -> std::optional<size_t> {
    auto it = paths.find(path);
    if (it == paths.end() || !owns[it->second]) return std::nullopt;
    return it->second;
  };
  for (size_t k = 0; k < occurrences.size(); ++k) {
    const PlanOccurrence& occurrence = occurrences[k];
    if (parents.contains(occurrence.id)) continue;
    const Placement& target = worlds[k];
    if (!saved.contains(target.path) || owner_of(target.path)) continue;
    if (occurrence.prototype.kind == Prototype::Kind::Unresolved)
      throw DecodeError("shared display prototype is unresolved");
    std::vector<size_t> donors;
    for (size_t c = 0; c < occurrences.size(); ++c) {
      if (!(occurrences[c].prototype == occurrence.prototype)) continue;
      if (auto donor = owner_of(worlds[c].path)) donors.push_back(*donor);
    }
    if (donors.size() != 1)
      throw DecodeError("shared display prototype has no unique mesh owner");
    plan.instances[donors[0]].push_back(target);
  }
  return plan;
}

namespace {
/// display.rs `place`.
void place(Tessellation& mesh, const Placement& placement) {
  for (auto& strip : mesh.strips) {
    for (MeshVertex& v : strip) {
      auto p = placement.transform.apply_point(v.position);
      if (!p) throw DecodeError("assembly position overflow");
      v.position = *p;
    }
  }
  if (mesh.has_normals) {
    for (auto& strip : mesh.strips) {
      for (MeshVertex& v : strip) {
        auto n = placement.transform.apply_vector(v.normal);
        if (!n) throw DecodeError("assembly normal overflow");
        v.normal = *n;
      }
    }
  }
  if (is_blank(placement.path)) throw DecodeError("assembly display path is empty");
  SourceObject source;
  source.object_id = placement.path;
  source.name = placement.path;
  source.visible = placement.visible;
  mesh.source_object = std::move(source);
}
}  // namespace

void apply_display(const DisplayPlan& plan, Model& model) {
  std::unordered_map<std::string_view, std::vector<size_t>> bindings_by_mesh;
  for (size_t b = 0; b < model.appearance_bindings.size(); ++b) {
    const AppearanceBinding& binding = model.appearance_bindings[b];
    if (binding.target_kind == "tessellation") bindings_by_mesh[binding.target_id].push_back(b);
  }
  std::vector<Tessellation> copies;
  std::vector<AppearanceBinding> copied_bindings;
  for (Tessellation& mesh : model.tessellations) {
    auto it = plan.table_owner.find(mesh.id);
    if (it == plan.table_owner.end()) continue;
    const std::vector<Placement>& instances = plan.instances[it->second];
    const std::vector<size_t>* bound = nullptr;
    if (auto b = bindings_by_mesh.find(mesh.id); b != bindings_by_mesh.end()) bound = &b->second;
    for (size_t i = 0; i < instances.size(); ++i) {
      Tessellation copy = mesh;
      std::string suffix = ":instance:" + std::to_string(i);
      copy.id = mesh.id + suffix;
      if (bound) {
        for (size_t b : *bound) {
          AppearanceBinding copied = model.appearance_bindings[b];
          copied.id += suffix;
          copied.target_id = copy.id;
          copied_bindings.push_back(std::move(copied));
        }
      }
      place(copy, instances[i]);
      copies.push_back(std::move(copy));
    }
    place(mesh, plan.owners[it->second]);
  }
  for (auto& c : copies) model.tessellations.push_back(std::move(c));
  for (auto& b : copied_bindings) model.appearance_bindings.push_back(std::move(b));
}

}  // namespace assembly

// ---------------------------------------------------------------------------
// Stage entry points (assembly.rs `read`, `AssemblyProjection::apply`)
// ---------------------------------------------------------------------------
void AssemblyPlanDeleter::operator()(AssemblyPlan* p) const { delete p; }

AssemblyPlanPtr read_assembly(const Container& container) {
  using namespace assembly;
  const Bytes* tree = nullptr;
  bool is_assembly = false, trees_differ = false;
  std::map<std::string, std::optional<std::string>> virtual_sections;
  for (const Section& section : container.sections) {
    if (!section.name) continue;
    const std::string& name = *section.name;
    if (is_embedded_document(name)) continue;
    if (auto key = virtual_component_key(name)) {
      auto [it, inserted] = virtual_sections.try_emplace(*key, section.site_key());
      if (!inserted) it->second = std::nullopt;
      continue;
    }
    std::string_view unwrapped = unwrapped_name(name);
    if (unwrapped == kTreeSection) {
      is_assembly = true;
      if (!tree) tree = &section.payload;
      else if (*tree != section.payload) trees_differ = true;
    } else if (is_mate_list(unwrapped)) {
      is_assembly = true;
    }
  }
  if (!is_assembly) return nullptr;
  AssemblyPlanPtr plan(new AssemblyPlan);
  if (tree && !trees_differ) {
    if (auto text = xml_text(ByteSpan(*tree))) plan->projection = project(*text, virtual_sections);
  }
  return plan;
}

std::set<std::string> virtual_site_keys(const AssemblyPlan& plan) {
  return {plan.projection.virtual_sites.begin(), plan.projection.virtual_sites.end()};
}

void apply_assembly(const AssemblyPlan& plan, const Container& container, Model& model) {
  using namespace assembly;
  const auto& occurrences = plan.projection.occurrences;
  DisplayPlan display = read_display(occurrences, container, model);
  apply_display(display, model);
  for (size_t i = 0; i < occurrences.size(); ++i) {
    const Placement& w = display.worlds[i];
    model.occurrences.push_back(Occurrence{occurrences[i].id, w.path, w.visible});
  }
  // `decoded.ir.model.finalize()`: arenas sorted by identity (stable).
  auto by_id = [](const auto& a, const auto& b) { return a.id < b.id; };
  std::stable_sort(model.tessellations.begin(), model.tessellations.end(), by_id);
  std::stable_sort(model.appearance_bindings.begin(), model.appearance_bindings.end(), by_id);
  std::stable_sort(model.occurrences.begin(), model.occurrences.end(), by_id);
}

}  // namespace swglb
