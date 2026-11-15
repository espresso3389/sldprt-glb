// SPDX-License-Identifier: Apache-2.0
// Assembly stage tests, ported from cadmpeg-codec-sldprt
// `assembly/display/tests.rs` (with the placement and shared-mesh
// extensions) and `assembly/tests.rs`.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>

#include "assembly_internal.h"
#include "errors.h"
#include "stages.h"

using namespace swglb;
using namespace swglb::assembly;

namespace {

int failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                       \
    }                                                                   \
  } while (0)

bool throws(const std::function<void()>& f) {
  try {
    f();
  } catch (const DecodeError&) {
    return true;
  }
  return false;
}

constexpr const char* kXml = R"(<swSolidWorks><swHeader>
      <swFile id="1" swDocType="ASSEMBLY" swPath="Top.SLDASM"/>
      <swFile id="2" swDocType="ASSEMBLY" swPath="Sub.SLDASM"/>
      <swFile id="3" swDocType="PART" swPath="Part.SLDPRT"/>
      </swHeader><swModelList>
      <swModel id="10" swName="Top" swFileRef="1" swConfigurationName="Default">
        <swReference id="11" swName="Sub" swReferenceNumber="1" swModelRef="20" swSuppressed="NO" swHidden="NO" swTransform="0 1 0 0 -1 0 0 0 0 0 1 0 0.01 0 0 1"/>
      </swModel>
      <swModel id="20" swName="Top" swFileRef="2" swConfigurationName="Default">
        <swReference id="21" swName="Part" swReferenceNumber="1" swModelRef="30" swSuppressed="NO" swHidden="YES" swTransform="1 0 0 0 0 1 0 0 0 0 1 0 0.003 0 0 1"/>
      </swModel>
      <swModel id="30" swName="Part" swFileRef="3" swConfigurationName="Default"/>
      </swModelList><swConfigurationList>
        <swConfiguration id="100" swName="Default" swMostRecentConfiguration="YES" swModelRef="10"/>
      </swConfigurationList></swSolidWorks>)";

std::vector<PlanOccurrence> occurrences() { return project(kXml, {}).occurrences; }

/// display.rs tests `repeated`: a second placement of the part, visible,
/// with an identity transform.
std::vector<PlanOccurrence> repeated() {
  auto rows = occurrences();
  PlanOccurrence copy = rows[2];
  copy.id = "sldprt:assembly:occurrence#copy";
  copy.name = "Part-2";
  copy.visible = true;
  copy.transform = Transform::identity();
  rows.push_back(copy);
  return rows;
}

/// A synthetic DisplayLists section: component headers, each optionally
/// followed by one descriptor table (placeholder bytes; the table ranges
/// are carried by the tessellations as the display-list stage records them).
struct Display {
  Bytes payload;
  std::vector<ByteRange> tables;

  Display() { append_text("uoTempAssemblySHDData_c"); }
  void append_text(std::string_view s) { payload.insert(payload.end(), s.begin(), s.end()); }
  void header(std::string_view path) {
    payload.push_back(0x36);
    payload.push_back(0x80);  // u16 class reference tag 0x8036
    payload.insert(payload.end(), {0xff, 0xfe, 0xff, static_cast<uint8_t>(path.size())});
    for (char c : path) {
      payload.push_back(static_cast<uint8_t>(c));
      payload.push_back(0);
    }
    payload.push_back(0);
  }
  /// display.rs tests `display`: header and one table.
  void display(std::string_view path) {
    header(path);
    size_t start = payload.size();
    for (int i = 0; i < 24; ++i) payload.push_back(0x11);
    tables.push_back({start, payload.size()});
  }
  /// display.rs tests `reference`: header only.
  void reference(std::string_view path) { header(path); }
};

constexpr size_t kOrdinal = 0x41;

Container container_of(const Display& d) {
  Container c;
  Section s;
  s.name = "Contents/DisplayLists";
  s.source_stream = "Contents/DisplayLists";
  s.ordinal = kOrdinal;
  s.payload = d.payload;
  c.sections.push_back(std::move(s));
  return c;
}

/// One tessellation per table: positions (0,0,0), (1,0,0), (0,1,0) mm,
/// normals (1,0,0).
Model model_of(const Display& d) {
  Model m;
  for (size_t i = 0; i < d.tables.size(); ++i) {
    Tessellation t;
    t.id = "sldprt:displaylist:record#" + std::to_string(kOrdinal) + ":" + std::to_string(i);
    t.strips = {{{{0, 0, 0}, {1, 0, 0}}, {{1, 0, 0}, {1, 0, 0}}, {{0, 1, 0}, {1, 0, 0}}}};
    t.section_index = 0;
    t.table_index = i;
    t.table = d.tables[i];
    m.tessellations.push_back(std::move(t));
  }
  return m;
}

bool near(double a, double b, double tol) { return std::fabs(a - b) < tol; }

void projection_ids() {
  auto rows = occurrences();
  CHECK(rows.size() == 3);
  CHECK(rows[0].id == "sldprt:assembly:occurrence#root");
  CHECK(rows[1].id == "sldprt:assembly:occurrence#11");
  CHECK(rows[2].id == "sldprt:assembly:occurrence#11-21");
  CHECK(rows[0].name == std::optional<std::string>("Top"));
  CHECK(rows[2].name == std::optional<std::string>("Part-1"));
  CHECK(rows[2].visible == std::optional<bool>(false));
  CHECK(rows[2].prototype.kind == Prototype::Kind::External);
  CHECK(rows[2].prototype.document == "Part.SLDPRT");
  CHECK(rows[2].prototype.object == std::optional<std::string>("Default"));
  CHECK(rows[1].prototype.kind == Prototype::Kind::Local);
  CHECK(rows[1].prototype.definition == "sldprt:assembly:model#20");
}

/// assembly/tests.rs: the sub-assembly placed twice expands under each.
void sub_assembly_placed_twice() {
  const char* rotated = "0 1 0 0 -1 0 0 0 0 0 1 0 0 0.02 0 1";
  const char* shifted = "1 0 0 0 0 1 0 0 0 0 1 0 0.003 0 0 1";
  const char* shifted10 = "1 0 0 0 0 1 0 0 0 0 1 0 0.01 0 0 1";
  const char* identity = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1";
  std::string xml = std::string(R"(<?xml version="1.0" encoding="UTF-8"?>
<swSolidWorks xmlns="http://www.solidworks.com/sw2003/schema" swVersion="17000">
 <swHeader>
  <swFile id="3" swDocType="ASSEMBLY" swPath="C:\work\Top.SLDASM"/>
  <swFile id="6" swDocType="ASSEMBLY" swPath="C:\work\Sub.SLDASM"/>
  <swFile id="9" swDocType="PART" swPath="C:\work\Pin.SLDPRT"/>
  <swFile id="12" swDocType="PART" swPath="Box^Top.sldprt"/>
 </swHeader>
 <swModelList>
  <swModel id="8" swName="Pin" swConfigurationName="Long" swFileRef="9"/>
  <swModel id="13" swName="Box" swConfigurationName="Default" swFileRef="12"/>
  <swModel id="5" swName="Top" swConfigurationName="Default" swFileRef="6">
   <swReference id="14" swName="Pin" swReferenceNumber="1" swModelRef="8" swSuppressed="NO" swHidden="NO" swIsVirtualComponent="NO" swTransform=")") + shifted + R"("/>
  </swModel>
  <swModel id="2" swName="Top" swConfigurationName="Default" swFileRef="3">
   <swReference id="4" swName="Sub" swReferenceNumber="1" swModelRef="5" swSuppressed="NO" swHidden="NO" swIsVirtualComponent="NO" swTransform=")" + shifted10 + R"("/>
   <swReference id="7" swName="Sub" swReferenceNumber="2" swModelRef="5" swSuppressed="NO" swHidden="NO" swIsVirtualComponent="NO" swTransform=")" + rotated + R"("/>
   <swReference id="10" swName="Pin" swReferenceNumber="2" swModelRef="8" swSuppressed="YES" swHidden="NO" swIsVirtualComponent="NO" swTransform=")" + identity + R"("/>
   <swReference id="11" swName="Box" swReferenceNumber="1" swModelRef="13" swSuppressed="NO" swHidden="YES" swIsVirtualComponent="YES" swTransform=")" + identity + R"("/>
  </swModel>
  <swModel id="15" swName="Top" swConfigurationName="Default-_flexible1" swFileRef="3">
   <swReference id="16" swName="Pin" swReferenceNumber="3" swModelRef="8" swSuppressed="NO" swHidden="NO" swIsVirtualComponent="NO" swTransform=")" + identity + R"("/>
  </swModel>
 </swModelList>
 <swConfigurationList>
  <swConfiguration id="1" swName="Default" swMostRecentConfiguration="YES" swModelRef="2"/>
  <swConfiguration id="17" swName="Default-_flexible1" swMostRecentConfiguration="NO" swModelRef="15"/>
 </swConfigurationList>
</swSolidWorks>)";
  // Without the virtual section, the virtual part is unresolved.
  auto p = project(xml, {});
  std::vector<std::string> ids;
  for (const auto& o : p.occurrences) ids.push_back(o.id);
  std::vector<std::string> expected = {
      "sldprt:assembly:occurrence#root", "sldprt:assembly:occurrence#4",
      "sldprt:assembly:occurrence#4-14", "sldprt:assembly:occurrence#7",
      "sldprt:assembly:occurrence#7-14", "sldprt:assembly:occurrence#11"};
  CHECK(ids == expected);
  CHECK(p.occurrences[5].prototype.kind == Prototype::Kind::Unresolved);
  CHECK(p.virtual_sites.empty());
  // With the `VirtualComp/Box-sldprt` section, it is a local definition.
  auto v = project(xml, {{"box-sldprt", std::optional<std::string>("block@7")}});
  CHECK(v.occurrences[5].prototype.kind == Prototype::Kind::Local);
  CHECK(v.virtual_sites == std::vector<std::string>{"block@7"});
  // Placement: transposed rotation, millimetres.
  const auto& t = p.occurrences[3].transform;  // #7, ROTATED
  CHECK(t.rows[0][0] == 0 && t.rows[0][1] == -1 && t.rows[1][0] == 1 && t.rows[1][3] == 20);
  // Shared paths of the two Pin instances.
  CHECK(world(p.occurrences[2], p.occurrences).path == "Sub-1@Top/Pin-1@Sub");
  CHECK(world(p.occurrences[4], p.occurrences).path == "Sub-2@Top/Pin-1@Sub");
  // World placement of #7-14: rotate (3,0,0) -> (0,3,0), then +(0,20,0).
  auto w = world(p.occurrences[4], p.occurrences);
  auto q = w.transform.apply_point({0, 0, 0});
  CHECK(q && near(q->x, 0, 1e-12) && near(q->y, 23, 1e-12));
}

void placements_refused() {
  CHECK(placement("1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"));
  CHECK(!placement("1 0 0 0.5 0 1 0 0 0 0 1 0 0 0 0 1"));  // projective column
  CHECK(!placement("1 0 0 0 0 1 0 0 0 0 1 0 0 0 0"));      // 15 values
  CHECK(!placement("-1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"));   // mirrored
  CHECK(!placement("2 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"));    // scaled
  CHECK(!placement("1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 inf"));
}

void structure_lost() {
  CHECK(project("<swSolidWorks><unclosed></swSolidWorks>", {}).occurrences.empty());
  CHECK(project("<Other/>", {}).occurrences.empty());
  // Two active configurations.
  CHECK(project(R"(<swSolidWorks><swModelList><swModel id="1"/></swModelList>
    <swConfiguration swMostRecentConfiguration="YES" swModelRef="1"/>
    <swConfiguration swMostRecentConfiguration="YES" swModelRef="1"/></swSolidWorks>)",
                {})
            .occurrences.empty());
  // A reference cycle is omitted; the root remains.
  auto cyc = project(R"(<swSolidWorks><swModelList>
    <swModel id="1"><swReference id="2" swModelRef="1" swName="A" swTransform="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/></swModel>
    </swModelList><swConfiguration swMostRecentConfiguration="YES" swModelRef="1"/></swSolidWorks>)",
                     {});
  CHECK(cyc.occurrences.size() == 1);
  // Entities in attribute values.
  auto ent = project(R"(<swSolidWorks><swHeader><swFile id="1" swDocType="PART" swPath="a&amp;b&#x41;.SLDPRT"/>
    <swFile id="0" swDocType="ASSEMBLY" swPath="T.SLDASM"/></swHeader><swModelList>
    <swModel id="9" swFileRef="0"><swReference id="2" swModelRef="3" swName="P" swReferenceNumber="1" swTransform="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/></swModel>
    <swModel id="3" swFileRef="1" swConfigurationName=" "/>
    </swModelList><swConfiguration swMostRecentConfiguration="YES" swModelRef="9"/></swSolidWorks>)",
                     {});
  CHECK(ent.occurrences.size() == 2);
  CHECK(ent.occurrences[1].prototype.document == "a&bA.SLDPRT");
  CHECK(!ent.occurrences[1].prototype.object);  // blank configuration
}

/// display/tests.rs `nested_component_mesh_uses_world_placement_and_visibility`.
void nested_world_placement_and_visibility() {
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  Container c = container_of(d);
  Model m = model_of(d);
  auto plan = read_display(occurrences(), c, m);
  CHECK(plan.table_owner.size() == 1);
  apply_display(plan, m);
  CHECK(m.tessellations.size() == 1);
  const auto& v = m.tessellations[0].strips[0];
  CHECK(near(v[0].normal.x, 0, 1e-15) && near(v[0].normal.y, 1, 1e-15));
  CHECK(near(v[0].position.x, 10, 1e-10) && near(v[0].position.y, 3, 1e-10));
  CHECK(near(v[1].position.x, 10, 1e-6) && near(v[1].position.y, 4, 1e-6));
  const auto& so = m.tessellations[0].source_object;
  CHECK(so && so->visible == std::optional<bool>(false));
  CHECK(so && so->object_id == "Sub-1@Top/Part-1@Sub" && so->name == so->object_id);
}

/// display/tests.rs `unknown_component_is_rejected_instead_of_attached_to_previous_part`.
void unknown_component_rejected() {
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  d.display("Unknown-1@Top");
  Container c = container_of(d);
  Model m = model_of(d);
  CHECK(throws([&] { read_display(occurrences(), c, m); }));
}

/// display/tests.rs `cyclic_occurrence_parent_is_rejected`.
void cyclic_parent_rejected() {
  auto rows = occurrences();
  rows[0].parent = rows[0].id;
  CHECK(throws([&] { world(rows[0], rows); }));
  auto absent = occurrences();
  absent[1].parent = "sldprt:assembly:occurrence#missing";
  CHECK(throws([&] { world(absent[2], absent); }));
}

/// display/tests.rs `trailing_shared_instance_keeps_color_and_uses_its_own_transform`.
void trailing_shared_instance() {
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  d.reference("Sub-1@Top/Part-2@Sub");
  Container c = container_of(d);
  Model m = model_of(d);
  AppearanceBinding b;
  b.id = "sldprt:appearance:binding#color";
  b.target_kind = "tessellation";
  b.target_id = m.tessellations[0].id;
  b.appearance = "sldprt:appearance:color#red";
  m.appearance_bindings.push_back(b);
  auto plan = read_display(repeated(), c, m);
  apply_display(plan, m);
  CHECK(m.tessellations.size() == 2);
  CHECK(m.appearance_bindings.size() == 2);
  CHECK(m.appearance_bindings[0].appearance == m.appearance_bindings[1].appearance);
  CHECK(m.appearance_bindings[1].target_id == m.tessellations[1].id);
  CHECK(m.appearance_bindings[1].id == "sldprt:appearance:binding#color:instance:0");
  CHECK(m.tessellations[1].id == "sldprt:displaylist:record#65:0:instance:0");
  CHECK(m.tessellations[0].id != m.tessellations[1].id);
  const auto& p = m.tessellations[1].strips[0][0].position;
  CHECK(near(p.x, 10, 1e-10) && std::fabs(p.y) < 1e-10);
  // The copy starts from the component-local normal (1,0,0) -> (0,1,0).
  CHECK(near(m.tessellations[1].strips[0][0].normal.y, 1, 1e-15));
  CHECK(m.tessellations[1].source_object->visible == std::optional<bool>(true));
  CHECK(m.tessellations[0].source_object->visible == std::optional<bool>(false));
  CHECK(m.tessellations[1].source_object->object_id == "Sub-1@Top/Part-2@Sub");
}

/// display/tests.rs `unresolved_prototype_cannot_supply_shared_geometry`.
void unresolved_prototype() {
  auto rows = repeated();
  rows[3].prototype = Prototype{};
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  d.reference("Sub-1@Top/Part-2@Sub");
  Container c = container_of(d);
  Model m = model_of(d);
  CHECK(throws([&] { read_display(rows, c, m); }));
}

/// display/tests.rs `occurrence_without_saved_display_is_not_invented`.
void occurrence_without_saved_display() {
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  Container c = container_of(d);
  Model m = model_of(d);
  auto plan = read_display(repeated(), c, m);
  for (const auto& i : plan.instances) CHECK(i.empty());
}

/// display/tests.rs `different_configuration_cannot_supply_shared_geometry`.
void configuration_mismatch() {
  auto rows = repeated();
  CHECK(rows[3].prototype.kind == Prototype::Kind::External);
  rows[3].prototype.object = "Other configuration";
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  d.reference("Sub-1@Top/Part-2@Sub");
  Container c = container_of(d);
  Model m = model_of(d);
  CHECK(throws([&] { read_display(rows, c, m); }));
}

/// display/tests.rs `ambiguous_saved_mesh_owners_are_rejected`.
void ambiguous_owners() {
  auto rows = repeated();
  PlanOccurrence third = rows[3];
  third.id = "sldprt:assembly:occurrence#third";
  third.name = "Part-3";
  rows.push_back(third);
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  d.display("Sub-1@Top/Part-2@Sub");
  d.reference("Sub-1@Top/Part-3@Sub");
  Container c = container_of(d);
  Model m = model_of(d);
  CHECK(throws([&] { read_display(rows, c, m); }));
}

/// Two occurrences with one instance path are ambiguous.
void ambiguous_paths() {
  auto rows = repeated();
  rows[3].name = "Part-1";
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  Container c = container_of(d);
  Model m = model_of(d);
  CHECK(throws([&] { read_display(rows, c, m); }));
}

/// The stage entry points: read the tree from a container, apply, and
/// finalize (ids sorted; occurrences with effective visibility).
void stage_entry_points() {
  Display d;
  d.display("Sub-1@Top/Part-1@Sub");
  d.reference("Sub-1@Top/Part-2@Sub");
  Container c = container_of(d);
  std::string xml = kXml;
  Section tree;
  tree.name = "swXmlContents/COMPINSTANCETREE";
  tree.payload = {0xff, 0xfe};
  for (char ch : xml) {
    tree.payload.push_back(static_cast<uint8_t>(ch));
    tree.payload.push_back(0);
  }
  c.sections.push_back(tree);
  AssemblyPlanPtr plan = read_assembly(c);
  CHECK(plan != nullptr);
  CHECK(plan->projection.occurrences.size() == 3);
  Model m = model_of(d);
  apply_assembly(*plan, c, m);
  CHECK(m.occurrences.size() == 3);
  CHECK(m.occurrences[0].id == "sldprt:assembly:occurrence#11");
  CHECK(m.occurrences[1].id == "sldprt:assembly:occurrence#11-21");
  CHECK(m.occurrences[1].visible == false);
  CHECK(m.occurrences[2].id == "sldprt:assembly:occurrence#root");
  CHECK(m.tessellations.size() == 1);  // part-2 is not an occurrence here
  // A part has no plan.
  Container part;
  CHECK(read_assembly(part) == nullptr);
}

}  // namespace

int main() {
  projection_ids();
  sub_assembly_placed_twice();
  placements_refused();
  structure_lost();
  nested_world_placement_and_visibility();
  unknown_component_rejected();
  cyclic_parent_rejected();
  trailing_shared_instance();
  unresolved_prototype();
  occurrence_without_saved_display();
  configuration_mismatch();
  ambiguous_owners();
  ambiguous_paths();
  stage_entry_points();
  if (failures) {
    std::fprintf(stderr, "%d failure(s)\n", failures);
    return 1;
  }
  std::puts("assembly_test OK");
  return 0;
}
