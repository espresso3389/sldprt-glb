// SPDX-License-Identifier: Apache-2.0
// Ported from cadmpeg-codec-sldprt `container.rs`,
// `container/configuration_bindings.rs` and `history/mod.rs` (cadmpeg
// 5af95421, Apache-2.0). The XML reader accepts the well-formed subset that
// roxmltree accepts for these envelopes (no external DTD entities).
#include "brep_config.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <set>

namespace swglb {
namespace {

// ---------------------------------------------------------------------------
// Text decoding
// ---------------------------------------------------------------------------
void put_utf8(std::string& out, uint32_t cp) {
  if (cp < 0x80) {
    out.push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out.push_back(static_cast<char>(0xc0 | (cp >> 6)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  } else if (cp < 0x10000) {
    out.push_back(static_cast<char>(0xe0 | (cp >> 12)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3f)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  } else {
    out.push_back(static_cast<char>(0xf0 | (cp >> 18)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3f)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3f)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
  }
}

bool valid_utf8(ByteSpan b) {
  size_t i = 0;
  while (i < b.size()) {
    uint8_t c = b[i];
    if (c < 0x80) {
      ++i;
      continue;
    }
    size_t n;
    uint32_t cp, min;
    if ((c & 0xe0) == 0xc0) n = 1, cp = c & 0x1f, min = 0x80;
    else if ((c & 0xf0) == 0xe0) n = 2, cp = c & 0x0f, min = 0x800;
    else if ((c & 0xf8) == 0xf0) n = 3, cp = c & 0x07, min = 0x10000;
    else return false;
    for (size_t k = 1; k <= n; ++k) {
      if (i + k >= b.size() || (b[i + k] & 0xc0) != 0x80) return false;
      cp = (cp << 6) | (b[i + k] & 0x3f);
    }
    if (cp < min || cp > 0x10ffff || (cp >= 0xd800 && cp <= 0xdfff)) return false;
    i += n + 1;
  }
  return true;
}

// ---------------------------------------------------------------------------
// XML reader
// ---------------------------------------------------------------------------
class XmlReader {
 public:
  explicit XmlReader(std::string_view s) : s_(s) {}

  std::optional<XmlElement> document() {
    if (s_.substr(0, 3) == "\xef\xbb\xbf") p_ = 3;
    if (!misc(true)) return std::nullopt;
    if (!at('<')) return std::nullopt;
    XmlElement root;
    if (!element(root, 0)) return std::nullopt;
    if (!misc(false)) return std::nullopt;
    if (p_ != s_.size()) return std::nullopt;
    return root;
  }

 private:
  std::string_view s_;
  size_t p_ = 0;

  bool at(char c) const { return p_ < s_.size() && s_[p_] == c; }
  bool starts(std::string_view t) const { return s_.substr(p_, t.size()) == t; }
  void ws() {
    while (p_ < s_.size() && (s_[p_] == ' ' || s_[p_] == '\t' || s_[p_] == '\r' || s_[p_] == '\n'))
      ++p_;
  }
  bool skip_past(std::string_view t) {
    size_t q = s_.find(t, p_);
    if (q == std::string_view::npos) return false;
    p_ = q + t.size();
    return true;
  }
  static bool name_char(char c) {
    unsigned char u = static_cast<unsigned char>(c);
    return u >= 0x80 || std::isalnum(u) || c == '_' || c == ':' || c == '-' || c == '.';
  }
  static bool name_start(char c) {
    unsigned char u = static_cast<unsigned char>(c);
    return u >= 0x80 || std::isalpha(u) || c == '_' || c == ':';
  }
  bool name(std::string& out) {
    if (p_ >= s_.size() || !name_start(s_[p_])) return false;
    size_t b = p_;
    while (p_ < s_.size() && name_char(s_[p_])) ++p_;
    out.assign(s_.substr(b, p_ - b));
    return true;
  }
  // Prolog/epilog: XML declaration, PIs, comments, DOCTYPE, whitespace.
  bool misc(bool prolog) {
    for (;;) {
      ws();
      if (starts("<?")) {
        if (!skip_past("?>")) return false;
      } else if (starts("<!--")) {
        if (!skip_past("-->")) return false;
      } else if (prolog && starts("<!DOCTYPE")) {
        int depth = 0;
        for (; p_ < s_.size(); ++p_) {
          if (s_[p_] == '[') ++depth;
          else if (s_[p_] == ']') --depth;
          else if (s_[p_] == '>' && depth == 0) break;
        }
        if (p_ >= s_.size()) return false;
        ++p_;
      } else {
        return true;
      }
    }
  }
  bool reference(std::string& out) {
    // at '&'
    size_t semi = s_.find(';', p_);
    if (semi == std::string_view::npos || semi - p_ > 12) return false;
    std::string_view ent = s_.substr(p_ + 1, semi - p_ - 1);
    p_ = semi + 1;
    if (ent == "lt") out.push_back('<');
    else if (ent == "gt") out.push_back('>');
    else if (ent == "amp") out.push_back('&');
    else if (ent == "quot") out.push_back('"');
    else if (ent == "apos") out.push_back('\'');
    else if (ent.size() >= 2 && ent[0] == '#') {
      uint32_t cp = 0;
      bool hex = ent[1] == 'x';
      std::string_view digits = ent.substr(hex ? 2 : 1);
      if (digits.empty()) return false;
      for (char c : digits) {
        int d;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (hex && c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (hex && c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else return false;
        cp = cp * (hex ? 16 : 10) + static_cast<uint32_t>(d);
        if (cp > 0x10ffff) return false;
      }
      if (cp == 0 || (cp >= 0xd800 && cp <= 0xdfff)) return false;
      put_utf8(out, cp);
    } else {
      return false;
    }
    return true;
  }
  bool attribute_value(std::string& out) {
    if (p_ >= s_.size() || (s_[p_] != '"' && s_[p_] != '\'')) return false;
    char quote = s_[p_++];
    for (;;) {
      if (p_ >= s_.size()) return false;
      char c = s_[p_];
      if (c == quote) {
        ++p_;
        return true;
      }
      if (c == '<') return false;
      if (c == '&') {
        if (!reference(out)) return false;
        continue;
      }
      // Attribute-value normalisation of whitespace characters.
      out.push_back(c == '\t' || c == '\n' || c == '\r' ? ' ' : c);
      ++p_;
    }
  }
  static std::string local(const std::string& n) {
    size_t c = n.find(':');
    return c == std::string::npos ? n : n.substr(c + 1);
  }
  bool element(XmlElement& e, int depth) {
    if (depth > 1024) return false;
    ++p_;  // '<'
    std::string raw;
    if (!name(raw)) return false;
    e.name = local(raw);
    std::set<std::string> seen;
    for (;;) {
      size_t before = p_;
      ws();
      if (starts("/>")) {
        p_ += 2;
        return true;
      }
      if (at('>')) {
        ++p_;
        break;
      }
      if (p_ == before) return false;  // attributes need separating whitespace
      std::string an, av;
      if (!name(an)) return false;
      ws();
      if (!at('=')) return false;
      ++p_;
      ws();
      if (!attribute_value(av)) return false;
      if (!seen.insert(an).second) return false;
      e.attributes.emplace_back(std::move(an), std::move(av));
    }
    // Content.
    for (;;) {
      if (p_ >= s_.size()) return false;
      if (starts("</")) {
        p_ += 2;
        std::string close;
        if (!name(close) || close != raw) return false;
        ws();
        if (!at('>')) return false;
        ++p_;
        return true;
      }
      if (starts("<!--")) {
        if (!skip_past("-->")) return false;
      } else if (starts("<![CDATA[")) {
        if (!skip_past("]]>")) return false;
      } else if (starts("<?")) {
        if (!skip_past("?>")) return false;
      } else if (at('<')) {
        e.children.emplace_back();
        if (!element(e.children.back(), depth + 1)) return false;
      } else if (at('&')) {
        std::string sink;
        if (!reference(sink)) return false;
      } else {
        ++p_;
      }
    }
  }
};

template <typename F>
void each_descendant(const XmlElement& e, F&& f) {
  f(e);
  for (const auto& c : e.children) each_descendant(c, f);
}

std::optional<std::string> attr(const XmlElement& e, std::string_view key) {
  if (const std::string* v = e.attribute(key)) return *v;
  return std::nullopt;
}

std::optional<size_t> decimal(const std::optional<std::string>& v) {
  if (!v || v->empty() || v->size() > 19) return std::nullopt;
  size_t out = 0;
  for (char c : *v) {
    if (c < '0' || c > '9') return std::nullopt;
    out = out * 10 + static_cast<size_t>(c - '0');
  }
  return out;
}

/// Rust `str::parse::<u32>()`.
std::optional<uint32_t> parse_u32(const std::string& v) {
  std::string_view s = v;
  if (!s.empty() && s[0] == '+') s.remove_prefix(1);
  if (s.empty()) return std::nullopt;
  uint64_t out = 0;
  for (char c : s) {
    if (c < '0' || c > '9') return std::nullopt;
    out = out * 10 + static_cast<uint64_t>(c - '0');
    if (out > 0xffffffffull) return std::nullopt;
  }
  return static_cast<uint32_t>(out);
}

bool is_features_manifest_name(const std::optional<std::string>& name) {
  if (!name) return false;
  std::string_view n = *name;
  size_t slash = n.rfind('/');
  if (slash != std::string_view::npos) n = n.substr(slash + 1);
  return iequals(n, "Features") || iequals(n, "Features__ZLB");
}

}  // namespace

const std::string* XmlElement::attribute(std::string_view key) const {
  for (const auto& [k, v] : attributes)
    if (k == key) return &v;
  return nullptr;
}

std::optional<std::string> xml_text(ByteSpan bytes) {
  if (!bytes.empty() && bytes[0] == 0x86) bytes = bytes.subspan(1);
  if (bytes.size() >= 2 && bytes[0] == 0xff && bytes[1] == 0xfe) {
    std::string out;
    out.reserve(bytes.size() / 2);
    size_t n = (bytes.size() - 2) / 2;
    for (size_t i = 0; i < n; ++i) {
      uint32_t u = bytes[2 + 2 * i] | (bytes[3 + 2 * i] << 8);
      if (u >= 0xd800 && u <= 0xdbff && i + 1 < n) {
        uint32_t l = bytes[4 + 2 * i] | (bytes[5 + 2 * i] << 8);
        if (l >= 0xdc00 && l <= 0xdfff) {
          put_utf8(out, 0x10000 + ((u - 0xd800) << 10) + (l - 0xdc00));
          ++i;
          continue;
        }
      }
      if (u >= 0xd800 && u <= 0xdfff) u = 0xfffd;
      put_utf8(out, u);
    }
    return out;
  }
  if (!valid_utf8(bytes)) return std::nullopt;
  return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

std::optional<XmlElement> parse_xml(std::string_view text) {
  // Cheap reject: an XML document starts with optional whitespace/BOM and '<'.
  size_t i = 0;
  if (text.substr(0, 3) == "\xef\xbb\xbf") i = 3;
  while (i < text.size() && (text[i] == ' ' || text[i] == '\t' || text[i] == '\r' || text[i] == '\n'))
    ++i;
  if (i >= text.size() || text[i] != '<') return std::nullopt;
  return XmlReader(text).document();
}

std::optional<size_t> configuration_index(std::string_view section) {
  for (size_t i = 0; i + 7 <= section.size(); ++i) {
    if (!iequals(section.substr(i, 7), "config-")) continue;
    size_t start = i + 7, n = 0;
    while (start + n < section.size() && section[start + n] >= '0' && section[start + n] <= '9') ++n;
    if (n == 0 || n > 19) return std::nullopt;
    size_t out = 0;
    for (size_t k = 0; k < n; ++k) out = out * 10 + static_cast<size_t>(section[start + k] - '0');
    return out;
  }
  return std::nullopt;
}

ConfigurationScan::ConfigurationScan(const Container& container) {
  std::set<std::string> names;
  for (const auto& section : container.sections) {
    auto text = xml_text(section.payload);
    if (!text) continue;
    auto root = parse_xml(*text);
    if (!root) continue;
    if (is_features_manifest_name(section.name) && root->name == "swSolidWorks") {
      Manifest m;
      each_descendant(*root, [&](const XmlElement& e) {
        if (e.name == "swConfiguration")
          m.configurations.push_back({attr(e, "swName"), attr(e, "swID"), attr(e, "swModelRef"),
                                      attr(e, "swMostRecentConfiguration")});
        else if (e.name == "swModel")
          m.models.push_back({attr(e, "id"), attr(e, "swConfigurationName"),
                              attr(e, "swConfigurationId")});
      });
      documents_.push_back(std::move(m));
    }
    if (root->name.find("Keywords") != std::string::npos) {
      each_descendant(*root, [&](const XmlElement& e) {
        if (e.name != "Configuration") return;
        keywords_.push_back({attr(e, "Name"), attr(e, "SourceIndex")});
        auto n = attr(e, "Name");
        auto idx = decimal(attr(e, "SourceIndex"));
        if (n && idx) source_indices_[*n].push_back(*idx);
      });
      // Native configurations: direct children (history/mod.rs).
      size_t ordinal = 0;
      for (const auto& c : root->children) {
        if (c.name != "Configuration") continue;
        NativeConfiguration nc;
        nc.id = "sldprt:model:configuration#" + std::to_string(section.ordinal) + ":" +
                std::to_string(ordinal);
        nc.ordinal = ordinal++;
        nc.name = attr(c, "Name").value_or("");
        if (const std::string* si = c.attribute("SourceIndex")) nc.source_index = parse_u32(*si);
        natives_.push_back(std::move(nc));
      }
    }
    if (root->name != "swSolidWorks") continue;
    each_descendant(*root, [&](const XmlElement& e) {
      if (e.name != "swModel") return;
      if (auto n = attr(e, "swConfigurationName")) names.insert(*n);
    });
    has_envelope_ = true;
  }
  configuration_names_.assign(names.begin(), names.end());
  finish();
  compute_active();
}

bool ConfigurationScan::has_manifest_rows() const {
  return std::any_of(documents_.begin(), documents_.end(),
                     [](const Manifest& d) { return !d.configurations.empty(); });
}

void ConfigurationScan::finish() {
  std::map<std::string, std::vector<const Keyword*>> keywords;
  std::map<size_t, size_t> explicit_counts;
  std::map<size_t, std::optional<std::string>> explicit_names;
  for (const auto& k : keywords_) {
    if (auto idx = decimal(k.source_index)) {
      ++explicit_counts[*idx];
      explicit_names[*idx] = k.name;
    }
    if (k.name) keywords[*k.name].push_back(&k);
  }
  std::map<std::string, std::vector<std::pair<size_t, const Row*>>> rows;
  std::map<size_t, size_t> counts;
  std::map<size_t, std::optional<std::string>> row_names;
  std::vector<std::map<std::string, std::vector<const ModelRow*>>> models;
  for (size_t di = 0; di < documents_.size(); ++di) {
    std::map<std::string, std::vector<const ModelRow*>> by_id;
    for (const auto& m : documents_[di].models)
      if (m.id) by_id[*m.id].push_back(&m);
    models.push_back(std::move(by_id));
    for (const auto& r : documents_[di].configurations) {
      if (r.name) rows[*r.name].emplace_back(di, &r);
      if (auto idx = decimal(r.index)) {
        ++counts[*idx];
        row_names[*idx] = r.name;
      }
    }
  }
  std::set<std::string> all;
  for (const auto& [n, _] : keywords) all.insert(n);
  for (const auto& [n, _] : rows) all.insert(n);
  auto count_of = [](const std::map<size_t, size_t>& m, size_t k) -> size_t {
    auto it = m.find(k);
    return it == m.end() ? 0 : it->second;
  };
  for (const std::string& name : all) {
    auto index = [&]() -> std::optional<size_t> {
      const Keyword* keyword = nullptr;
      if (auto it = keywords.find(name); it != keywords.end()) {
        if (it->second.size() != 1) return std::nullopt;
        keyword = it->second[0];
      }
      if (name.empty()) return std::nullopt;
      auto manifest_index = [&]() -> std::optional<size_t> {
        auto it = rows.find(name);
        if (it == rows.end() || it->second.size() != 1) return std::nullopt;
        auto [di, row] = it->second[0];
        auto idx = decimal(row->index);
        if (!idx) return std::nullopt;
        if (count_of(counts, *idx) != 1 || count_of(explicit_counts, *idx) > 1)
          return std::nullopt;
        if (auto en = explicit_names.find(*idx);
            en != explicit_names.end() && en->second != std::optional<std::string>(name))
          return std::nullopt;
        if (!row->model || row->model->empty()) return std::nullopt;
        auto mi = models[di].find(*row->model);
        if (mi == models[di].end() || mi->second.size() != 1) return std::nullopt;
        const ModelRow* model = mi->second[0];
        if (model->name != std::optional<std::string>(name) || model->index != row->index)
          return std::nullopt;
        return idx;
      }();
      if (!keyword || !keyword->source_index) return manifest_index;
      auto idx = decimal(keyword->source_index);
      if (!idx) return std::nullopt;
      if (count_of(explicit_counts, *idx) != 1 || count_of(counts, *idx) > 1) return std::nullopt;
      if (auto it = rows.find(name); it != rows.end() && it->second.size() > 1) return std::nullopt;
      if (auto rn = row_names.find(*idx); rn != row_names.end() && rn->second && *rn->second != name)
        return std::nullopt;
      if (manifest_index && *manifest_index != *idx) return std::nullopt;
      return idx;
    }();
    resolved_[name] = index;
  }
  for (const auto& [name, index] : resolved_) {
    if (!index) continue;
    auto it = names_by_index_.find(*index);
    if (it == names_by_index_.end()) names_by_index_.emplace(*index, name);
    else it->second.reset();
  }
}

std::pair<ManifestIndex, size_t> ConfigurationScan::index_for(const std::string& name) const {
  if (!has_manifest_rows()) return {ManifestIndex::kAbsent, 0};
  auto it = resolved_.find(name);
  if (it == resolved_.end() || !it->second) return {ManifestIndex::kUnresolved, 0};
  return {ManifestIndex::kResolved, *it->second};
}

std::optional<std::string> ConfigurationScan::manifest_name(size_t index) const {
  if (!has_manifest_rows()) return std::nullopt;
  auto it = names_by_index_.find(index);
  if (it == names_by_index_.end()) return std::nullopt;
  return it->second;
}

void ConfigurationScan::compute_active() {
  if (!has_manifest_rows()) {
    active_ = Active::kAbsent;
    return;
  }
  active_ = Active::kAmbiguous;
  const Manifest* doc = nullptr;
  const Row* row = nullptr;
  size_t yes = 0;
  for (const auto& d : documents_)
    for (const auto& r : d.configurations)
      if (r.most_recent == std::optional<std::string>("YES")) {
        if (yes++ == 0) doc = &d, row = &r;
      }
  if (yes != 1) return;
  auto idx = decimal(row->index);
  if (!idx) return;
  size_t same = 0;
  for (const auto& d : documents_)
    for (const auto& r : d.configurations) same += decimal(r.index) == idx;
  if (same != 1) return;
  auto it = names_by_index_.find(*idx);
  if (it == names_by_index_.end() || !it->second) return;
  const std::string& name = *it->second;
  if (row->name && *row->name != name) return;
  if (row->model) {
    if (row->model->empty()) return;
    const ModelRow* model = nullptr;
    size_t n = 0;
    for (const auto& m : doc->models)
      if (m.id == row->model) {
        if (n++ == 0) model = &m;
      }
    if (n != 1) return;
    if (model->name && *model->name != name) return;
    if (model->index && model->index != row->index) return;
  }
  active_ = Active::kUnique;
  active_index_ = *idx;
  active_row_name_ = name;
}

std::optional<std::string> ConfigurationScan::active_name() const {
  if (active_ == Active::kUnique) return active_row_name_;
  if (active_ == Active::kAbsent && configuration_names_.size() == 1)
    return configuration_names_.front();
  return std::nullopt;
}

std::optional<size_t> ConfigurationScan::active_index() const {
  if (has_manifest_rows()) {
    if (active_ == Active::kUnique) return active_index_;
    return std::nullopt;
  }
  auto name = active_name();
  if (!name) return std::nullopt;
  auto it = source_indices_.find(*name);
  if (it == source_indices_.end() || it->second.size() != 1) return std::nullopt;
  return it->second[0];
}

}  // namespace swglb
