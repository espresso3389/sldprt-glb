// SPDX-License-Identifier: Apache-2.0
// Parasolid B-rep through libopenparasolid: site selection, imported-body
// merging and configuration binding. Ported from cadmpeg-codec-sldprt
// `decode.rs` (`active_body_streams`, `try_decode_brep`,
// `assign_configuration_bodies`, `mark_active_configuration`) and
// `container.rs` (`select_active_parasolid_site`), cadmpeg 5af95421,
// Apache-2.0.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>

#include "brep_config.h"
#include "brep_site.h"
#include "brep_streams.h"
#include "brep_visual.h"
#include "container.h"
#include "stages.h"

namespace swglb {
namespace {

std::string_view name_of(const Section& s) { return s.name ? std::string_view(*s.name) : ""; }

std::string_view unwrapped_name(std::string_view name) {
  if (name.size() >= 5 && iequals(name.substr(name.size() - 5), "__ZLB"))
    return name.substr(0, name.size() - 5);
  return name;
}

bool is_stock_bodies_section(std::string_view name) {
  size_t cut = name.find_last_of("/\\");
  if (cut != std::string_view::npos) name = name.substr(cut + 1);
  if (!istarts_with(name, "config-") || !iends_with(name, "-stockbodies")) return false;
  std::string_view index = name.substr(7, name.size() >= 19 ? name.size() - 19 : 0);
  if (name.size() < 7 + 12 + 1) return false;
  return std::all_of(index.begin(), index.end(), [](char c) { return c >= '0' && c <= '9'; });
}

bool is_embedded_document(std::string_view name) {
  name = unwrapped_name(name);
  return name.starts_with("ImportedComp/") || iends_with(name, ".sldprt") ||
         iends_with(name, ".sldasm");
}

bool is_virtual_component(std::string_view name) {
  std::string n(unwrapped_name(name));
  std::replace(n.begin(), n.end(), '\\', '/');
  if (n.size() < 12 || !iequals(std::string_view(n).substr(0, 12), "VirtualComp/")) return false;
  std::string_view key = std::string_view(n).substr(12);
  return key.size() > 7 && iends_with(key, "-sldprt");
}

bool is_local_bodies_section(std::string_view name) {
  std::string n(name);
  std::replace(n.begin(), n.end(), '\\', '/');
  size_t last = n.rfind('/');
  if (last == std::string::npos) return false;
  if (!iequals(std::string_view(n).substr(last + 1), "LocalBodies")) return false;
  std::string_view head = std::string_view(n).substr(0, last);
  size_t prev = head.rfind('/');
  std::string_view folder = prev == std::string_view::npos ? head : head.substr(prev + 1);
  return icontains(folder, "FeatureBodies");
}

struct StreamRef {
  size_t section;
  size_t stream;
};

class Streams {
 public:
  explicit Streams(const Container& c) : c_(c), ps_(c.sections.size()) {
    for (size_t i = 0; i < c.sections.size(); ++i)
      ps_[i] = extract_parasolid_streams(c.sections[i].payload, name_of(c.sections[i]));
  }
  const Section& section(const StreamRef& r) const { return c_.sections[r.section]; }
  const PsStream& stream(const StreamRef& r) const { return ps_[r.section][r.stream]; }
  const std::vector<PsStream>& of(size_t section) const { return ps_[section]; }
  size_t size() const { return ps_.size(); }

 private:
  const Container& c_;
  std::vector<std::vector<PsStream>> ps_;
};

/// cadmpeg `select_active_parasolid_site`.
std::optional<StreamRef> select_active_site(const Container& c, const Streams& st,
                                            const ConfigurationScan& cs) {
  auto active = cs.active_index();
  if (cs.has_manifest_rows() && !active) return std::nullopt;
  std::optional<StreamRef> selected;
  for (size_t si = 0; si < c.sections.size(); ++si) {
    std::string_view name = name_of(c.sections[si]);
    bool bad = icontains(name, "ghost") || icontains(name, "deltas") ||
               icontains(name, "resolvedfeatures");
    bool section_is_partition = icontains(name, "partition") && !bad;
    bool admissible = !bad && !is_stock_bodies_section(name) && !is_virtual_component(name);
    const auto& streams = st.of(si);
    size_t body_streams = std::count_if(streams.begin(), streams.end(),
                                        [](const PsStream& s) { return is_body_stream(s); });
    for (size_t k = 0; k < streams.size(); ++k) {
      const PsStream& s = streams[k];
      if (!is_body_stream(s)) continue;
      const std::string& d = s.description;
      if (!admissible || icontains(d, "ghost") || icontains(d, "deltas") ||
          !(icontains(d, "partition") || (body_streams == 1 && section_is_partition)))
        continue;
      if (active && (!c.sections[si].name || configuration_index(*c.sections[si].name) != active))
        continue;
      if (selected) return std::nullopt;
      selected = StreamRef{si, k};
    }
  }
  return selected;
}

/// cadmpeg `active_body_streams`.
std::vector<StreamRef> active_body_streams(const Container& c, const Streams& st) {
  std::vector<StreamRef> out;
  for (size_t si = 0; si < c.sections.size(); ++si) {
    std::string_view name = name_of(c.sections[si]);
    if (icontains(name, "ghost") || icontains(name, "resolvedfeatures") ||
        is_stock_bodies_section(name) || is_embedded_document(name) || is_virtual_component(name))
      continue;
    const auto& streams = st.of(si);
    for (size_t k = 0; k < streams.size(); ++k)
      if (is_body_stream(streams[k])) out.push_back({si, k});
  }
  std::stable_sort(out.begin(), out.end(), [&](const StreamRef& a, const StreamRef& b) {
    auto key = [&](const StreamRef& r) {
      return std::make_pair(!icontains(c.sections[r.section].source_stream, "partition"),
                            !icontains(st.stream(r).description, "partition"));
    };
    return key(a) < key(b);
  });
  return out;
}

std::vector<std::string> body_ids(const SiteBrep& b) {
  std::vector<std::string> out;
  for (const auto& body : b.bodies) out.push_back(body.id);
  return out;
}

struct Config {
  Configuration c;
  std::optional<uint32_t> source_index;
  bool has_bodies = false;
};

struct ActiveFacts {
  std::optional<std::string> name;   ///< sw_configuration_name
  std::optional<size_t> index;       ///< configuration index of active_parasolid_block
  bool unresolved = false;           ///< sldprt_active_partition_unresolved
};

/// cadmpeg `bind_active_configuration_partition`.
std::optional<std::pair<uint32_t, size_t>> bind_active_partition(std::vector<Config>& cfgs,
                                                                 const ActiveFacts& f) {
  if (!f.name || !f.index || *f.index > 0xffffffffu) return std::nullopt;
  uint32_t index = static_cast<uint32_t>(*f.index);
  std::vector<size_t> matches;
  for (size_t i = 0; i < cfgs.size(); ++i)
    if (cfgs[i].c.name == f.name) matches.push_back(i);
  if (matches.empty()) return std::nullopt;
  bool taken = std::any_of(cfgs.begin(), cfgs.end(),
                           [&](const Config& c) { return c.source_index == index; });
  if (matches.size() > 1 || cfgs[matches[0]].source_index || taken) return std::nullopt;
  cfgs[matches[0]].source_index = index;
  return std::make_pair(index, matches[0]);
}

/// cadmpeg `mark_active_configuration`.
void mark_active(std::vector<Config>& cfgs, const ActiveFacts& f) {
  auto unique = [&](auto pred) -> std::optional<size_t> {
    std::optional<size_t> found;
    for (size_t i = 0; i < cfgs.size(); ++i)
      if (pred(cfgs[i])) {
        if (found) return std::nullopt;
        found = i;
      }
    return found;
  };
  std::optional<size_t> selected;
  if (f.name) {
    selected = unique([&](const Config& c) { return c.c.name == f.name; });
  } else if (f.index) {
    selected = unique([&](const Config& c) {
      return c.source_index && static_cast<size_t>(*c.source_index) == *f.index;
    });
  } else if (f.unresolved) {
    selected.reset();
  } else if (cfgs.size() == 1) {
    selected = 0;
  }
  if (selected) cfgs[*selected].c.active = true;
}

/// cadmpeg `assign_configuration_bodies`.
void assign_configuration_bodies(
    std::vector<Config>& cfgs,
    const std::vector<std::pair<size_t, std::vector<std::string>>>& configuration_bodies,
    const ConfigurationScan& cs, const ActiveFacts& f) {
  std::map<uint32_t, std::vector<std::string>> partitions;
  for (const auto& [index, bodies] : configuration_bodies) {
    if (index > 0xffffffffu) continue;
    auto& merged = partitions[static_cast<uint32_t>(index)];
    for (const auto& b : bodies)
      if (std::find(merged.begin(), merged.end(), b) == merged.end()) merged.push_back(b);
  }
  bool manifest = cs.has_manifest_rows();
  if (manifest) {
    for (auto& cfg : cfgs) {
      std::optional<uint32_t> index;
      if (cfg.c.name) {
        auto [kind, value] = cs.index_for(*cfg.c.name);
        if (kind == ManifestIndex::kResolved && value <= 0xffffffffu &&
            partitions.count(static_cast<uint32_t>(value)))
          index = static_cast<uint32_t>(value);
      }
      cfg.source_index = index;
      cfg.c.bodies.clear();
      cfg.has_bodies = false;
    }
  }
  std::map<uint32_t, size_t> counts;
  for (const auto& cfg : cfgs)
    if (cfg.source_index) ++counts[*cfg.source_index];
  for (auto& cfg : cfgs) {
    if (!cfg.source_index || counts[*cfg.source_index] != 1) continue;
    auto it = partitions.find(*cfg.source_index);
    if (it == partitions.end()) continue;
    cfg.c.bodies = std::move(it->second);
    cfg.has_bodies = true;
    partitions.erase(it);
  }
  if (!manifest) {
    if (auto bound = bind_active_partition(cfgs, f)) {
      auto it = partitions.find(bound->first);
      if (it != partitions.end()) {
        cfgs[bound->second].c.bodies = std::move(it->second);
        cfgs[bound->second].has_bodies = true;
        partitions.erase(it);
      }
    }
  }
  for (auto& [index, bodies] : partitions) {
    std::optional<std::string> name;
    if (!counts.count(index)) name = cs.manifest_name(index);
    size_t ordinal = 0;
    if (!cfgs.empty()) {
      for (const auto& cfg : cfgs) ordinal = std::max(ordinal, cfg.c.ordinal);
      ++ordinal;
    }
    Config cfg;
    cfg.c.id = "sldprt:model:configuration#partition:" + std::to_string(index);
    cfg.c.ordinal = ordinal;
    cfg.c.name = name ? *name : "Config-" + std::to_string(index);
    cfg.c.bodies = std::move(bodies);
    cfg.source_index = index;
    cfg.has_bodies = true;
    cfgs.push_back(std::move(cfg));
  }
}

template <typename T>
void append(std::vector<T>& into, std::vector<T>& from) {
  into.insert(into.end(), std::make_move_iterator(from.begin()),
              std::make_move_iterator(from.end()));
}

}  // namespace

BrepResult decode_brep(const Container& container, const std::set<std::string>& placed_virtual,
                       Model& model) {
  (void)placed_virtual;  // virtual component documents are not decoded
  BrepResult result;
  ConfigurationScan cs(container);
  Streams st(container);

  std::vector<Config> cfgs;
  for (const auto& n : cs.natives()) {
    Config cfg;
    cfg.c.id = n.id;
    cfg.c.ordinal = n.ordinal;
    cfg.c.name = n.name;
    cfg.source_index = n.source_index;
    cfgs.push_back(std::move(cfg));
  }

  auto active_site = select_active_site(container, st, cs);
  ActiveFacts facts;
  if (cs.has_envelope()) facts.name = cs.active_name();
  if (active_site)
    facts.index = configuration_index(container.sections[active_site->section].source_stream);

  auto finish_metadata = [&] {
    // Metadata path: no `sldprt_active_partition_unresolved` attribute.
    bind_active_partition(cfgs, facts);
    mark_active(cfgs, facts);
    for (auto& cfg : cfgs) model.configurations.push_back(std::move(cfg.c));
    return result;
  };

  // Sites keyed by section site key (BTreeMap order).
  std::vector<StreamRef> streams = active_body_streams(container, st);
  std::map<std::string, std::vector<size_t>> sites;
  for (size_t i = 0; i < streams.size(); ++i)
    sites[container.sections[streams[i].section].site_key()].push_back(i);
  if (sites.empty()) return finish_metadata();

  struct Decoded {
    std::string key;
    size_t first;
    SiteBrep brep;
    bool partition = false, deltas = false;
  };
  std::vector<Decoded> decoded;
  for (const auto& [key, indices] : sites) {
    std::vector<const PsStream*> ps;
    Decoded d{key, indices[0], {}, false, false};
    for (size_t i : indices) {
      const PsStream& s = st.stream(streams[i]);
      ps.push_back(&s);
      d.partition |= icontains(s.description, "partition");
      d.deltas |= icontains(s.description, "deltas");
    }
    if (auto b = decode_site(ps)) d.brep = std::move(*b);
    decoded.push_back(std::move(d));
  }

  std::optional<size_t> resolved;
  if (active_site) {
    std::string key = container.sections[active_site->section].site_key();
    for (size_t i = 0; i < decoded.size(); ++i)
      if (decoded[i].key == key) resolved = i;
  }
  size_t selected = resolved.value_or(0);
  auto empty_model = [](const Decoded& d) {
    return d.brep.entity_records == 0 && d.partition && d.deltas;
  };
  bool selected_empty = empty_model(decoded[selected]);
  bool any_geometry = std::any_of(decoded.begin(), decoded.end(),
                                  [](const Decoded& d) { return d.brep.has_geometry(); });
  bool takes_local_bodies = selected_empty && (resolved || !any_geometry);
  if (resolved) {
    if (!selected_empty && !decoded[selected].brep.has_geometry()) return finish_metadata();
  } else {
    bool any_empty = std::any_of(decoded.begin(), decoded.end(), empty_model);
    if (!any_geometry && !any_empty) return finish_metadata();
  }

  Decoded main = std::move(decoded[selected]);
  decoded.erase(decoded.begin() + static_cast<std::ptrdiff_t>(selected));
  if (!resolved) qualify_site(main.brep, main.key);
  const std::string& main_stream = container.sections[streams[main.first].section].source_stream;
  if (takes_local_bodies) {
    auto configuration = configuration_index(main_stream);
    if (!configuration) configuration = cs.active_index();
    if (configuration) {
      for (size_t si = 0; si < container.sections.size(); ++si) {
        const Section& section = container.sections[si];
        if (!is_local_bodies_section(name_of(section))) continue;
        if (configuration_index(name_of(section)) != configuration) continue;
        for (const PsStream& s : st.of(si)) {
          if (is_body_stream(s)) continue;
          auto local = decode_site({&s});
          if (!local) continue;  // unreadable imported body stream (cadmpeg loss note)
          qualify_site(*local, section.site_key() + "+" + std::to_string(s.offset));
          merge_site(main.brep, std::move(*local));
        }
      }
    }
  }
  std::vector<std::pair<size_t, std::vector<std::string>>> configuration_bodies;
  if (auto index = configuration_index(main_stream))
    configuration_bodies.emplace_back(*index, body_ids(main.brep));
  for (auto& alt : decoded) {
    qualify_site(alt.brep, alt.key);
    const std::string& stream = container.sections[streams[alt.first].section].source_stream;
    if (auto index = configuration_index(stream))
      configuration_bodies.emplace_back(*index, body_ids(alt.brep));
    merge_site(main.brep, std::move(alt.brep));
  }

  SiteBrep& b = main.brep;
  append(model.bodies, b.bodies);
  append(model.regions, b.regions);
  append(model.shells, b.shells);
  append(model.faces, b.faces);
  append(model.loops, b.loops);
  append(model.coedges, b.coedges);
  append(model.edges, b.edges);
  append(model.vertices, b.vertices);
  append(model.points, b.points);
  append(model.surfaces, b.surfaces);
  append(model.curves, b.curves);
  result.decoded = true;
  result.face_keys = std::move(b.face_keys);
  result.face_identities = std::move(b.face_identities);

  if (!active_site) facts.unresolved = true;
  assign_configuration_bodies(cfgs, configuration_bodies, cs, facts);
  mark_active(cfgs, facts);
  for (auto& cfg : cfgs) model.configurations.push_back(std::move(cfg.c));
  result.face_colors = std::move(b.face_colors);
  NativePayloads payloads{std::move(b.face_payloads), std::move(b.body_payloads)};
  result.eligible_features = resolve_visual_states(container, payloads, model, result.visual_plan,
                                                   result.visual_unresolved);
  return result;
}

}  // namespace swglb
