// SPDX-License-Identifier: Apache-2.0
// Native configuration identities (Keywords `Configuration` records and the
// Features `swSolidWorks` manifest). Ported from cadmpeg-codec-sldprt
// `container.rs` / `container/configuration_bindings.rs` / `history/mod.rs`
// (cadmpeg 5af95421, Apache-2.0).
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "bytes.h"
#include "container.h"

namespace swglb {

/// One element of a parsed XML document (local names, decoded values).
struct XmlElement {
  std::string name;  ///< Local tag name (prefix stripped).
  std::vector<std::pair<std::string, std::string>> attributes;  ///< Raw names.
  std::vector<XmlElement> children;
  const std::string* attribute(std::string_view key) const;
};

/// cadmpeg `xml_text`: strip one leading 0x86, decode UTF-16LE after a BOM,
/// otherwise require UTF-8.
std::optional<std::string> xml_text(ByteSpan bytes);
/// Parse a well-formed document; nullopt for malformed XML.
std::optional<XmlElement> parse_xml(std::string_view text);

/// Digits after the first case-insensitive "config-" (cadmpeg
/// `configuration_index`).
std::optional<size_t> configuration_index(std::string_view section);

struct NativeConfiguration {
  std::string id;  ///< sldprt:model:configuration#<section ordinal>:<ordinal>
  size_t ordinal = 0;
  std::string name;
  std::optional<uint32_t> source_index;
};

enum class ManifestIndex { kAbsent, kUnresolved, kResolved };

class ConfigurationScan {
 public:
  explicit ConfigurationScan(const Container& container);

  const std::vector<NativeConfiguration>& natives() const { return natives_; }
  bool has_manifest_rows() const;
  std::pair<ManifestIndex, size_t> index_for(const std::string& name) const;
  /// `manifest_configuration_name` (requires manifest rows).
  std::optional<std::string> manifest_name(size_t index) const;
  std::optional<size_t> active_index() const;              ///< active_configuration_index
  std::optional<std::string> active_name() const;          ///< active_configuration_name_ref
  bool has_envelope() const { return has_envelope_; }      ///< a first swSolidWorks envelope

 private:
  struct Keyword {
    std::optional<std::string> name, source_index;
  };
  struct Row {
    std::optional<std::string> name, index, model, most_recent;
  };
  struct ModelRow {
    std::optional<std::string> id, name, index;
  };
  struct Manifest {
    std::vector<Row> configurations;
    std::vector<ModelRow> models;
  };
  enum class Active { kAbsent, kUnique, kAmbiguous };

  void finish();
  void compute_active();

  std::vector<NativeConfiguration> natives_;
  std::vector<Keyword> keywords_;
  std::vector<Manifest> documents_;
  std::map<std::string, std::optional<size_t>> resolved_;
  std::map<size_t, std::optional<std::string>> names_by_index_;
  std::map<std::string, std::vector<size_t>> source_indices_;
  std::vector<std::string> configuration_names_;  ///< sorted, unique
  bool has_envelope_ = false;
  Active active_ = Active::kAbsent;
  size_t active_index_ = 0;
  std::optional<std::string> active_row_name_;
};

}  // namespace swglb
