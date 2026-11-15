// SPDX-License-Identifier: Apache-2.0
// Unit tests for the B-rep stage: the SP_CURVE cache fit (ported from
// cadmpeg-codec-sldprt brep/fit.rs tests), the configuration XML helpers and
// the empty-model path of decode_brep.
#undef NDEBUG  // the checks below are assert()s; keep them in Release builds
#include <cassert>
#include <cmath>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "brep_config.h"
#include "brep_fit.h"
#include "container.h"
#include "stages.h"

using namespace swglb;

namespace {

/// De Boor evaluation of a nonrational curve.
Vec3 evaluate(const NurbsCurveGeometry& c, double t) {
  size_t p = c.degree, n = c.poles.size();
  size_t span = p;
  while (span + 1 < n && t >= c.knots[span + 1]) ++span;
  std::vector<Vec3> d(c.poles.begin() + (span - p), c.poles.begin() + (span + 1));
  for (size_t r = 1; r <= p; ++r)
    for (size_t j = p; j >= r; --j) {
      size_t i = span - p + j;
      double den = c.knots[i + p - r + 1] - c.knots[i];
      double a = den != 0 ? (t - c.knots[i]) / den : 0.0;
      d[j] = Vec3{(1 - a) * d[j - 1].x + a * d[j].x, (1 - a) * d[j - 1].y + a * d[j].y,
                  (1 - a) * d[j - 1].z + a * d[j].z};
    }
  return d[p];
}

double distance(Vec3 a, Vec3 b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

size_t multiplicity(const std::vector<double>& knots, double at) {
  size_t m = 0;
  for (double k : knots) m += k == at;
  return m;
}

auto always = [](double, double) { return true; };

// fit.rs `a_smooth_curve_is_fitted_within_the_bound_in_its_own_parameter`.
void smooth_curve_fits_within_bound() {
  auto helix = [](double t) -> std::optional<Vec3> {
    return Vec3{10.0 * std::cos(t), 10.0 * std::sin(t), 2.0 * t};
  };
  auto fit = brep::fit_curve(helix, {0.0, 3.0}, {}, always);
  assert(fit);
  assert(fit->degree == 3);
  assert(fit->knots.size() == fit->poles.size() + 4);
  assert(fit->knots.front() == 0.0 && fit->knots.back() == 3.0);
  for (int i = 0; i <= 200; ++i) {
    double t = 3.0 * i / 200.0;
    assert(distance(evaluate(*fit, t), *helix(t)) <= brep::kFitToleranceMm);
  }
}

// fit.rs `a_corner_is_kept_at_a_break_and_bounded_off_one`.
void corner_is_kept_at_break() {
  auto corner = [](double t) -> std::optional<Vec3> {
    return t <= 0.5 ? Vec3{t, 0, 0} : Vec3{0.5, t - 0.5, 0};
  };
  auto fit = brep::fit_curve(corner, {0.0, 1.0}, {0.5}, always);
  assert(fit);
  assert(multiplicity(fit->knots, 0.5) == 3);
  assert(distance(evaluate(*fit, 0.5), Vec3{0.5, 0, 0}) <= 1e-12);
}

// fit.rs `a_smooth_break_is_a_double_knot`.
void smooth_break_is_double_knot() {
  auto line = [](double t) -> std::optional<Vec3> { return Vec3{t, 2 * t, 0}; };
  auto fit = brep::fit_curve(line, {0.0, 1.0}, {0.5}, always);
  assert(fit);
  assert(multiplicity(fit->knots, 0.5) == 2);
}

// fit.rs `an_unresolved_span_is_halved_whatever_its_samples_read`.
void unresolved_span_is_halved() {
  auto line = [](double t) -> std::optional<Vec3> { return Vec3{t, 0, 0}; };
  auto fit = brep::fit_curve(line, {0.0, 1.0}, {}, [](double low, double high) { return high - low <= 0.25; });
  assert(fit);
  assert(fit->poles.size() > 4);
  for (double k : fit->knots) assert(k == 0.0 || k == 0.25 || k == 0.5 || k == 0.75 || k == 1.0);
}

// fit.rs `an_unevaluable_point_refuses_the_fit`.
void unevaluable_point_refuses_fit() {
  auto holed = [](double t) -> std::optional<Vec3> {
    if (t > 0.4 && t < 0.6) return std::nullopt;
    return Vec3{t, 0, 0};
  };
  assert(!brep::fit_curve(holed, {0.0, 1.0}, {}, always));
  auto line = [](double t) -> std::optional<Vec3> { return Vec3{t, 0, 0}; };
  assert(!brep::fit_curve(line, {1.0, 1.0}, {}, always));
  assert(!brep::fit_curve(line, {1.0, 0.0}, {}, always));
}

ByteSpan span_of(const std::string& s) {
  return ByteSpan(reinterpret_cast<const uint8_t*>(s.data()), s.size());
}

void xml_helpers() {
  // UTF-8, with and without the leading 0x86 marker.
  assert(xml_text(span_of("<a/>")) == std::string("<a/>"));
  assert(xml_text(span_of("\x86<a/>")) == std::string("<a/>"));
  // UTF-16LE after a BOM.
  std::string utf16 = std::string("\xff\xfe", 2) + std::string("<\0b\0/\0>\0", 8);
  assert(xml_text(span_of(utf16)) == std::string("<b/>"));
  // Invalid UTF-8 without a BOM.
  assert(!xml_text(span_of("\xc3\x28")));

  auto root = parse_xml("<sw:root x=\"1\"><child/><child y='2'></child></sw:root>");
  assert(root);
  assert(root->name == "root");
  assert(root->children.size() == 2);
  assert(root->children[0].name == "child");
  assert(!parse_xml("<root><child></root>"));
  assert(!parse_xml("not xml"));

  assert(configuration_index("Contents/Config-12-Partition") == size_t{12});
  assert(configuration_index("contents/CONFIG-3") == size_t{3});
  assert(!configuration_index("Contents/Config-Partition"));
  assert(!configuration_index("Contents/Partition"));
}

void empty_container_decodes_no_model() {
  Container container;
  Model model;
  BrepResult result = decode_brep(container, {}, model);
  assert(!result.decoded);
  assert(model.faces.empty() && model.bodies.empty());
  assert(result.face_keys.empty() && result.eligible_features.empty());
}

}  // namespace

int main() {
  smooth_curve_fits_within_bound();
  corner_is_kept_at_break();
  smooth_break_is_double_knot();
  unresolved_span_is_halved();
  unevaluable_point_refuses_fit();
  xml_helpers();
  empty_container_decodes_no_model();
  std::cout << "brep_test: ok\n";
  return 0;
}
