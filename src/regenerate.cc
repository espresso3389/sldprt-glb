// SPDX-License-Identifier: Apache-2.0
// Stale display-mesh regeneration.
//
// The reference pipeline exports the B-rep to STEP, tessellates it with an
// OCCT STEP-to-glTF converter and merges it with
// `replace_surface(saved_glb, brep_glb, boundary)`. This engine builds the
// OCCT shape straight from the decoded B-rep (the stale check only admits
// single-body parts whose surfaces are all planes/cylinders and whose edges
// are lines/circles), tessellates it with the same parameters, writes it
// with the same RWGltf_CafWriter settings and merges it with a port of
// replace_surface.
#include "regenerate.h"

#include <cmath>
#include <cstring>
#include <optional>
#include <string>
#include <unordered_map>

#include "errors.h"
#include "glb_internal.h"
#include "json_value.h"

#ifdef SWGLB_HAVE_OCCT
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <Geom_Circle.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <Message_ProgressRange.hxx>
#include <Precision.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <RWGltf_CafWriter.hxx>
#include <RWGltf_WriterTrsfFormat.hxx>
#include <RWMesh_CoordinateSystem.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <Standard_Failure.hxx>
#include <TColStd_IndexedDataMapOfStringString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <unistd.h>
#endif

namespace swglb {
using glb_internal::GlbParts;
using glb_internal::glb_parts;
namespace {

uint32_t le32(const Bytes& b, size_t at) {
  return static_cast<uint32_t>(b[at]) | static_cast<uint32_t>(b[at + 1]) << 8 |
         static_cast<uint32_t>(b[at + 2]) << 16 | static_cast<uint32_t>(b[at + 3]) << 24;
}

}  // namespace

/// the reference writer glb_parts.
GlbParts glb_internal::glb_parts(const Bytes& blob) {
  if (blob.size() < 28 || std::memcmp(blob.data(), "glTF", 4) != 0 || le32(blob, 8) != blob.size())
    throw DecodeError("Invalid GLB");
  size_t size = le32(blob, 12);
  size_t end = 20 + size;
  if (std::memcmp(blob.data() + 16, "JSON", 4) != 0 || end + 8 > blob.size())
    throw DecodeError("Invalid GLB JSON");
  GlbParts parts;
  parts.doc = JValue::parse(
      std::string_view(reinterpret_cast<const char*>(blob.data()) + 20, size));
  size_t binary_size = le32(blob, end);
  if (std::memcmp(blob.data() + end + 4, "BIN\0", 4) != 0 || end + 8 + binary_size != blob.size())
    throw DecodeError("Invalid GLB buffer");
  parts.binary.assign(blob.begin() + static_cast<std::ptrdiff_t>(end + 8), blob.end());
  return parts;
}

namespace {

const JValue* get(const JValue& object, std::string_view key) {
  return object.is_object() ? object.find(key) : nullptr;
}

size_t list_size(const JValue& object, std::string_view key) {
  const JValue* v = get(object, key);
  return v && v->is_array() ? v->arr().size() : 0;
}

/// the reference writer replace_surface.
Bytes replace_surface(const Bytes& saved_glb, const Bytes& brep_glb, const Bounds& expected,
                      const std::string& extras_key) {
  GlbParts saved = glb_parts(saved_glb);
  GlbParts source = glb_parts(brep_glb);
  JValue& doc = saved.doc;
  Bytes& binary = saved.binary;
  const JValue& src = source.doc;
  if (list_size(src, "buffers") != 1 || list_size(src, "nodes") != 1 || list_size(src, "meshes") != 1)
    throw DecodeError("Unsupported regenerated geometry");
  const JValue& node = src.find("nodes")->arr()[0];
  const JValue* mesh_ref = get(node, "mesh");
  bool placed = false;
  for (const char* k : {"matrix", "translation", "rotation", "scale"}) placed |= get(node, k) != nullptr;
  if (!mesh_ref || !mesh_ref->is_int() || mesh_ref->as_int() != 0 || placed)
    throw DecodeError("Unsupported regenerated placement");
  JValue primitives = src.find("meshes")->arr()[0].find("primitives")->clone();
  if (!primitives.is_array() || primitives.arr().empty()) throw DecodeError("Missing regenerated triangles");
  for (const auto& p : primitives.arr()) {
    const JValue* mode = get(p, "mode");
    if ((mode && !(mode->is_int() && mode->as_int() == 4)) || !get(*get(p, "attributes"), "POSITION"))
      throw DecodeError("Missing regenerated triangles");
  }
  const auto& source_accessors = src.find("accessors")->arr();
  Bounds actual;
  bool first = true;
  for (const auto& p : primitives.arr()) {
    const JValue& a = source_accessors.at(static_cast<size_t>(p.find("attributes")->find("POSITION")->as_int()));
    for (int i = 0; i < 3; ++i) {
      double lo = a.find("min")->arr().at(i).as_double();
      double hi = a.find("max")->arr().at(i).as_double();
      if (first || lo < actual.first[i]) actual.first[i] = lo;
      if (first || hi > actual.second[i]) actual.second[i] = hi;
    }
    first = false;
  }
  if (bounds_differ(actual, expected))
    throw DecodeError("Regenerated geometry does not match CAD boundaries");

  binary.resize(binary.size() + (4 - binary.size() % 4) % 4, 0);
  size_t offset = binary.size();
  binary.insert(binary.end(), source.binary.begin(), source.binary.end());
  JValue& views = doc["bufferViews"];
  JValue& accessors = doc["accessors"];
  int64_t view_offset = static_cast<int64_t>(views.size());
  int64_t accessor_offset = static_cast<int64_t>(accessors.size());
  JValue& materials_out = doc.setdefault("materials", JValue::array());
  int64_t material_offset = static_cast<int64_t>(materials_out.size());
  for (const auto& v : src.find("bufferViews")->arr()) {
    JValue view = v.clone();
    const JValue* buffer = get(view, "buffer");
    if (!buffer || !buffer->is_int() || buffer->as_int() != 0)
      throw DecodeError("Unsupported regenerated buffer");
    const JValue* byte_offset = get(view, "byteOffset");
    view["byteOffset"] = (byte_offset ? byte_offset->as_int() : 0) + static_cast<int64_t>(offset);
    views.push_back(view);
  }
  for (const auto& a : source_accessors) {
    JValue accessor = a.clone();
    if (!get(accessor, "bufferView") || get(accessor, "sparse"))
      throw DecodeError("Unsupported regenerated accessor");
    accessor["bufferView"] = accessor["bufferView"].as_int() + view_offset;
    accessors.push_back(accessor);
  }
  JValue materials = get(src, "materials") ? src.find("materials")->clone() : JValue::array();
  bool wireframe = false;
  if (const JValue* extras = get(doc["nodes"].arr()[0], "extras"))
    if (const JValue* presentation = get(*extras, extras_key))
      if (const JValue* mode = get(*presentation, "mode"))
        wireframe = mode->dump() == "\"wireframe\"";
  if (wireframe) {
    for (auto& material : materials.arr()) {
      material.setdefault("pbrMetallicRoughness", JValue::object())
          .setdefault("baseColorFactor", JValue::Array{1, 1, 1, 1})
          .arr()[3] = 0;
      material["alphaMode"] = "BLEND";
    }
    bool missing = false;
    for (const auto& p : primitives.arr()) missing |= !p.contains("material");
    if (missing) {
      size_t fallback = materials.size();
      JValue pbr = JValue::object();
      pbr["baseColorFactor"] = JValue::Array{1, 1, 1, 0};
      JValue material = JValue::object();
      material["pbrMetallicRoughness"] = pbr;
      material["alphaMode"] = "BLEND";
      materials.push_back(material);
      for (auto& p : primitives.arr()) p.setdefault("material", fallback);
    }
  }
  for (auto& m : materials.arr()) materials_out.push_back(m);
  for (auto& p : primitives.arr()) {
    JValue attributes = JValue::object();
    for (const auto& [k, v] : p.find("attributes")->obj()) attributes[k] = v.as_int() + accessor_offset;
    p["attributes"] = attributes;
    if (p.contains("indices")) p["indices"] = p["indices"].as_int() + accessor_offset;
    if (p.contains("material")) p["material"] = p["material"].as_int() + material_offset;
  }
  doc["meshes"].arr()[0]["primitives"] = primitives;
  return glb_encode(std::move(doc), std::move(binary));
}

#ifdef SWGLB_HAVE_OCCT

gp_Pnt pnt(const Vec3& v) { return gp_Pnt(v.x, v.y, v.z); }
gp_Dir dir(const Vec3& v) { return gp_Dir(v.x, v.y, v.z); }

struct ColoredFace {
  TopoDS_Face face;
  std::optional<Color> color;
};

/// Rebuild the B-rep as OCCT faces. Faces are made independently from the
/// shared curve and vertex definitions, healed (pcurves, wire order and
/// orientation, cylinder seams) and sewn into a shell afterwards.
class ShapeBuilder {
 public:
  explicit ShapeBuilder(const Model& model) : model_(model) {
    for (const auto& p : model.points) points_.emplace(p.id, &p.position);
    for (const auto& v : model.vertices) vertex_points_.emplace(v.id, v.point);
    for (const auto& c : model.curves) curves_.emplace(c.id, &c.geometry);
    for (const auto& e : model.edges) edges_.emplace(e.id, &e);
    for (const auto& c : model.coedges) coedges_.emplace(c.id, &c);
    for (const auto& l : model.loops) loops_.emplace(l.id, &l);
    for (const auto& s : model.surfaces) surfaces_.emplace(s.id, &s.geometry);
  }

  std::vector<ColoredFace> faces() {
    std::optional<Color> body_color = model_.bodies.size() == 1 ? model_.bodies[0].color : std::nullopt;
    std::vector<ColoredFace> out;
    for (const auto& face : model_.faces) {
      out.push_back({make_face(face), face.color ? face.color : body_color});
    }
    return out;
  }

 private:
  template <class Map>
  static auto lookup(const Map& map, const std::string& key) {
    auto it = map.find(key);
    if (it == map.end()) throw DecodeError("Incomplete B-rep for regeneration");
    return it->second;
  }

  TopoDS_Vertex vertex(const std::string& id) {
    if (auto it = vertices_.find(id); it != vertices_.end()) return it->second;
    TopoDS_Vertex v;
    BRep_Builder().MakeVertex(v, pnt(*lookup(points_, lookup(vertex_points_, id))), kTolerance);
    vertices_.emplace(id, v);
    return v;
  }

  TopoDS_Edge edge(const std::string& id) {
    if (auto it = built_edges_.find(id); it != built_edges_.end()) return it->second;
    const Edge& e = *lookup(edges_, id);
    if (!e.curve) throw DecodeError("Edge without carrier curve");
    const CurveGeometry& g = *lookup(curves_, *e.curve);
    std::optional<TopoDS_Vertex> v1, v2;
    if (e.start) v1 = vertex(*e.start);
    if (e.end) v2 = vertex(*e.end);
    TopoDS_Edge result;
    if (g.kind == "line" && !e.param_range) {
      if (!v1 || !v2) throw DecodeError("Missing line endpoints");
      BRepBuilderAPI_MakeEdge make(*v1, *v2);
      if (!make.IsDone()) throw DecodeError("Could not build line edge");
      result = make.Edge();
    } else if (e.param_range && (g.kind == "line" || g.kind == "circle")) {
      double t0 = (*e.param_range)[0], t1 = (*e.param_range)[1];
      Handle(Geom_Curve) curve;
      if (g.kind == "line") {
        // CADIR line parameters scale with the (possibly unnormalised)
        // direction; Geom_Line is unit speed.
        double speed = std::sqrt(g.direction.x * g.direction.x + g.direction.y * g.direction.y +
                                 g.direction.z * g.direction.z);
        if (!(speed > 0)) throw DecodeError("Degenerate line");
        curve = new Geom_Line(pnt(g.origin), dir(g.direction));
        t0 *= speed;
        t1 *= speed;
      } else {
        curve = new Geom_Circle(gp_Ax2(pnt(g.center), dir(g.axis), dir(g.ref_direction)), g.radius);
      }
      result = make_curve_edge(curve, t0, t1, v1, v2);
    } else {
      throw DecodeError("Unsupported edge for regeneration");
    }
    built_edges_.emplace(id, result);
    return result;
  }

  static TopoDS_Edge make_curve_edge(const Handle(Geom_Curve)& curve, double t0, double t1,
                                     const std::optional<TopoDS_Vertex>& v1,
                                     const std::optional<TopoDS_Vertex>& v2) {
    if (t0 > t1) std::swap(t0, t1);
    if (v1 && v2) {
      for (int attempt = 0; attempt < 2; ++attempt) {
        BRepBuilderAPI_MakeEdge make(curve, attempt ? *v2 : *v1, attempt ? *v1 : *v2, t0, t1);
        if (make.IsDone()) return make.Edge();
      }
    }
    BRepBuilderAPI_MakeEdge make(curve, t0, t1);
    if (!make.IsDone()) throw DecodeError("Could not build curve edge");
    return make.Edge();
  }

  TopoDS_Face make_face(const Face& face) {
    if (!face.surface) throw DecodeError("Face without surface");
    const SurfaceGeometry& g = *lookup(surfaces_, *face.surface);
    Handle(Geom_Surface) surface;
    gp_Ax3 frame(pnt(g.origin), dir(g.normal), dir(g.u_axis));
    if (g.kind == "plane") surface = new Geom_Plane(frame);
    else if (g.kind == "cylinder") surface = new Geom_CylindricalSurface(frame, g.radius);
    else throw DecodeError("Unsupported surface for regeneration");
    BRep_Builder builder;
    TopoDS_Face result;
    builder.MakeFace(result, surface, kTolerance);
    for (const auto& loop_id : face.loops) {
      const Loop& loop = *lookup(loops_, loop_id);
      TopoDS_Wire wire;
      builder.MakeWire(wire);
      for (const auto& coedge_id : loop.coedges) {
        const Coedge& coedge = *lookup(coedges_, coedge_id);
        TopoDS_Edge e = edge(coedge.edge);
        builder.Add(wire, coedge.reversed ? TopoDS::Edge(e.Reversed()) : e);
      }
      builder.Add(result, wire);
    }
    ShapeFix_Face fix(result);
    fix.SetPrecision(kTolerance);
    fix.Perform();
    TopoDS_Face fixed = fix.Face();
    if (face.reversed) fixed.Reverse();
    return fixed;
  }

  static constexpr double kTolerance = 1e-4;  // mm
  const Model& model_;
  std::unordered_map<std::string, const Vec3*> points_;
  std::unordered_map<std::string, std::string> vertex_points_;
  std::unordered_map<std::string, const CurveGeometry*> curves_;
  std::unordered_map<std::string, const Edge*> edges_;
  std::unordered_map<std::string, const Coedge*> coedges_;
  std::unordered_map<std::string, const Loop*> loops_;
  std::unordered_map<std::string, const SurfaceGeometry*> surfaces_;
  std::unordered_map<std::string, TopoDS_Vertex> vertices_;
  std::unordered_map<std::string, TopoDS_Edge> built_edges_;
};

/// The reference tessellation and RWGltf_CafWriter output for the shape.
Bytes tessellate_glb(const Model& model) {
  std::vector<ColoredFace> faces = ShapeBuilder(model).faces();
  BRepBuilderAPI_Sewing sewing(1e-3);
  for (const auto& f : faces) sewing.Add(f.face);
  sewing.Perform();
  TopoDS_Shape shape = sewing.SewedShape();
  if (shape.IsNull()) throw DecodeError("Could not sew regenerated faces");
  ShapeFix_Shape fix(shape);
  fix.Perform();
  shape = fix.Shape();

  Bnd_Box box;
  BRepBndLib::Add(shape, box, Standard_False);
  if (box.IsVoid()) throw DecodeError("Regenerated shape has empty bounds");
  double x0, y0, z0, x1, y1, z1;
  box.Get(x0, y0, z0, x1, y1, z1);
  double diagonal = std::hypot(std::hypot(x1 - x0, y1 - y0), z1 - z0);
  if (!std::isfinite(diagonal) || diagonal <= std::numeric_limits<double>::epsilon())
    throw DecodeError("Regenerated shape has invalid bounds");
  // Reference defaults: ratio 0.001, minimum 0.01 (model units), 0.5 rad.
  double linear = std::max(0.01, diagonal * 0.001);
  BRepMesh_IncrementalMesh mesher(shape, linear, Standard_False, 0.5, Standard_True);
  if (!mesher.IsDone()) throw DecodeError("OCCT could not tessellate the regenerated shape");

  Handle(TDocStd_Document) document;
  XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", document);
  XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
  Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
  Handle(XCAFDoc_ColorTool) colors = XCAFDoc_DocumentTool::ColorTool(document->Main());
  TDF_Label root = shapes->AddShape(shape, Standard_False);
  for (const auto& f : faces) {
    if (!f.color) continue;
    TopoDS_Shape sewn = sewing.IsModified(f.face) ? sewing.Modified(f.face) : f.face;
    TopoDS_Shape healed = fix.Context()->Apply(sewn);
    for (TopExp_Explorer it(healed, TopAbs_FACE); it.More(); it.Next()) {
      TDF_Label label = shapes->AddSubShape(root, it.Current());
      if (label.IsNull()) continue;
      Quantity_ColorRGBA rgba(Quantity_Color(f.color->r, f.color->g, f.color->b, Quantity_TOC_sRGB),
                              f.color->a);
      colors->SetColor(label, rgba, XCAFDoc_ColorSurf);
    }
  }
  TDF_LabelSequence roots;
  shapes->GetFreeShapes(roots);

  static std::atomic<unsigned> counter{0};
  auto path = std::filesystem::temp_directory_path() /
              ("sldprt-glb-" + std::to_string(::getpid()) + "-" + std::to_string(counter++) + ".glb");
  struct Cleanup {
    std::filesystem::path p;
    ~Cleanup() {
      std::error_code ec;
      std::filesystem::remove(p, ec);
    }
  } cleanup{path};
  RWGltf_CafWriter writer(path.string().c_str(), Standard_True);
  writer.SetTransformationFormat(RWGltf_WriterTrsfFormat_Compact);
  writer.SetNodeNameFormat(RWMesh_NameFormat_InstanceOrProduct);
  writer.SetMeshNameFormat(RWMesh_NameFormat_Product);
  writer.SetToEmbedTexturesInGlb(Standard_True);
  writer.SetMergeFaces(Standard_False);
  writer.ChangeCoordinateSystemConverter().SetInputLengthUnit(0.001);
  writer.ChangeCoordinateSystemConverter().SetInputCoordinateSystem(RWMesh_CoordinateSystem_Zup);
  TColStd_IndexedDataMapOfStringString info;
  info.Add("Generator", "sldprt-glb");
  if (!writer.Perform(document, roots, nullptr, info, Message_ProgressRange()))
    throw DecodeError("OCCT could not write the regenerated GLB");
  XCAFApp_Application::GetApplication()->Close(document);
  std::ifstream in(path, std::ios::binary);
  Bytes out((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (out.empty()) throw DecodeError("OCCT produced an empty regenerated GLB");
  return out;
}

#endif  // SWGLB_HAVE_OCCT

}  // namespace

Bytes regenerate_brep_glb(const Model& model) {
#ifdef SWGLB_HAVE_OCCT
  try {
    return tessellate_glb(model);
  } catch (const Standard_Failure& error) {
    throw DecodeError(std::string("OCCT error: ") + error.GetMessageString());
  }
#else
  (void)model;
  throw DecodeError("Built without OpenCASCADE; cannot regenerate stale display mesh");
#endif
}

Bytes regenerate_stale_surface(const Model& model, const Bytes& saved_glb, const Bounds& expected,
                               const std::string& extras_key) {
  return replace_surface(saved_glb, regenerate_brep_glb(model), expected, extras_key);
}

Bytes replace_surface_glb(const Bytes& saved_glb, const Bytes& brep_glb, const Bounds& expected,
                          const std::string& extras_key) {
  return replace_surface(saved_glb, brep_glb, expected, extras_key);
}

}  // namespace swglb
