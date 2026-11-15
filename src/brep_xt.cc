// SPDX-License-Identifier: Apache-2.0
#include "brep_xt.h"

#include <variant>

namespace swglb::xt {
namespace {

std::optional<int64_t> as_int(const openparasolid::ScalarValue& v) {
  if (const auto* i = std::get_if<int64_t>(&v)) return *i;
  if (const auto* u = std::get_if<uint64_t>(&v)) return static_cast<int64_t>(*u);
  if (const auto* b = std::get_if<bool>(&v)) return *b ? 1 : 0;
  return std::nullopt;
}

std::optional<double> as_real(const openparasolid::ScalarValue& v) {
  if (const auto* d = std::get_if<double>(&v)) return *d;
  return std::nullopt;
}

}  // namespace

namespace {
std::optional<openparasolid::SchemaParseReport> parse_report(ByteSpan payload) {
  openparasolid::SchemaLimits limits;
  limits.max_input_bytes = 1024ull * 1024 * 1024;
  limits.max_nodes = 50'000'000;
  limits.max_total_fields = 1'000'000'000;
  limits.max_values = 2'000'000'000;
  auto parsed =
      openparasolid::SchemaParser::Parse(std::span<const uint8_t>(payload.data(), payload.size()),
                                         nullptr, limits);
  if (!parsed.ok()) return std::nullopt;
  return std::move(parsed).value();
}

void index_names(const openparasolid::SchemaParseReport& report,
                 std::vector<std::unordered_map<std::string, uint64_t>>& names) {
  names.resize(report.schemas.size());
  for (size_t i = 0; i < report.schemas.size(); ++i) {
    const auto& fields = report.schemas[i].definition.fields;
    for (size_t f = 0; f < fields.size(); ++f) names[i].emplace(fields[f].name, f);
  }
}
}  // namespace

std::unique_ptr<Stream> Stream::parse(ByteSpan payload) {
  auto report = parse_report(payload);
  if (!report) return nullptr;
  auto s = std::unique_ptr<Stream>(new Stream());
  auto part = std::make_unique<Part>();
  part->report = std::move(*report);
  part->bytes = payload;
  index_names(part->report, part->names);
  s->parts_.push_back(std::move(part));
  const auto& r = s->parts_[0]->report;
  if (r.complete) {
    openparasolid::ModelOptions options;
    options.max_nodes = 50'000'000;
    options.max_fields = 1'000'000'000;
    options.max_references = 1'000'000'000;
    options.max_values = 2'000'000'000;
    options.max_diagnostics = 16;
    auto built = openparasolid::ModelBuilder::Build(r, options);
    if (built.ok()) s->model_ = std::make_unique<openparasolid::Model>(std::move(built).value());
  }
  s->rebuild();
  return s;
}

bool Stream::apply_deltas(ByteSpan payload) {
  auto report = parse_report(payload);
  if (!report || report->delta_nodes.empty()) return false;
  auto part = std::make_unique<Part>();
  part->report = std::move(*report);
  part->deltas = true;
  part->bytes = payload;
  index_names(part->report, part->names);
  parts_.push_back(std::move(part));
  rebuild();
  return true;
}

ByteSpan Stream::part_bytes(size_t part) const { return parts_[part]->bytes; }
bool Stream::part_is_deltas(size_t part) const { return parts_[part]->deltas; }

void Stream::rebuild() {
  // Deltas journals do not replace partition nodes: cadmpeg admits only the
  // delta records that agree with the partition (`consistent_deltas_nodes`),
  // which is not reproduced. Their attribute records still take part in the
  // per-stream face-colour versions (see brep_site.cc).
  by_index_.clear();
  merged_.clear();
  for (const auto& n : part_nodes(0)) {
    by_index_.insert_or_assign(n.index, &n);
  }
  for (const auto& n : part_nodes(0))
    if (by_index_[n.index] == &n) merged_.push_back(&n);
}

const std::vector<openparasolid::DecodedNode>& Stream::part_nodes(size_t part) const {
  const auto& r = parts_[part]->report;
  return parts_[part]->deltas ? r.delta_nodes : r.nodes;
}

const Stream::Part* Stream::part_of(const openparasolid::DecodedNode& n) const {
  for (const auto& p : parts_) {
    const auto& v = p->deltas ? p->report.delta_nodes : p->report.nodes;
    if (!v.empty() && &n >= v.data() && &n < v.data() + v.size()) return p.get();
  }
  return nullptr;
}

const openparasolid::DecodedNode* Stream::node(uint32_t index) const {
  if (index == 0) return nullptr;
  auto it = by_index_.find(index);
  return it == by_index_.end() ? nullptr : it->second;
}

const openparasolid::DecodedField* Stream::field(const openparasolid::DecodedNode& n,
                                                 std::string_view name) const {
  const Part* part = part_of(n);
  if (!part || n.schema_index >= part->names.size()) return nullptr;
  const auto& names = part->names[n.schema_index];
  auto it = names.find(std::string(name));
  if (it == names.end()) return nullptr;
  uint64_t def = it->second;
  if (def < n.fields.size() && n.fields[def].definition_index == def) return &n.fields[def];
  for (const auto& f : n.fields)
    if (f.definition_index == def) return &f;
  return nullptr;
}

int64_t Stream::integer(const openparasolid::DecodedNode& n, std::string_view name) const {
  const auto* f = field(n, name);
  if (!f || f->values.empty()) return 0;
  return as_int(f->values[0]).value_or(0);
}

std::vector<int64_t> Stream::integers(const openparasolid::DecodedNode& n,
                                      std::string_view name) const {
  std::vector<int64_t> out;
  const auto* f = field(n, name);
  if (!f) return out;
  out.reserve(f->values.size());
  for (const auto& v : f->values) out.push_back(as_int(v).value_or(0));
  return out;
}

std::vector<double> Stream::reals(const openparasolid::DecodedNode& n,
                                  std::string_view name) const {
  std::vector<double> out;
  const auto* f = field(n, name);
  if (!f) return out;
  out.reserve(f->values.size());
  for (const auto& v : f->values)
    if (auto d = as_real(v)) out.push_back(*d);
  return out;
}

std::optional<double> Stream::real(const openparasolid::DecodedNode& n,
                                   std::string_view name) const {
  const auto* f = field(n, name);
  if (!f || f->values.empty()) return std::nullopt;
  return as_real(f->values[0]);
}

std::vector<double> Stream::values_after_sense(const openparasolid::DecodedNode& n) const {
  std::vector<double> out;
  const Part* part = part_of(n);
  if (!part || n.schema_index >= part->names.size()) return out;
  const auto& names = part->names[n.schema_index];
  auto it = names.find("sense");
  if (it == names.end()) return out;
  for (const auto& f : n.fields) {
    if (f.definition_index <= it->second) continue;
    for (const auto& v : f.values) {
      if (auto d = as_real(v)) out.push_back(*d);
      else return out;  // a non-real field ends the analytic run
    }
  }
  return out;
}

std::string Stream::chars(const openparasolid::DecodedNode& n, std::string_view name) const {
  std::string out;
  const auto* f = field(n, name);
  if (!f) return out;
  for (const auto& v : f->values) {
    auto c = as_int(v);
    if (!c) break;
    out.push_back(static_cast<char>(*c));
  }
  return out;
}

const openparasolid::GeometryObject* Stream::geometry(uint32_t index) const {
  if (!model_) return nullptr;
  const auto* n = model_->FindNode(openparasolid::NodeId{index});
  if (!n || !n->geometry || *n->geometry >= model_->geometry().size()) return nullptr;
  return &model_->geometry()[*n->geometry];
}

}  // namespace swglb::xt
