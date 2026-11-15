// SPDX-License-Identifier: Apache-2.0
// Port of cadmpeg-codec-sldprt `brep/fit.rs` (cadmpeg 5af95421, Apache-2.0).
#include "brep_fit.h"

#include <algorithm>
#include <cmath>

namespace swglb::brep {
namespace {

/// Share of kFitToleranceMm a span's sampled departure may reach.
constexpr double kSampleMargin = 0.25;
/// Span fractions where the departure is sampled.
constexpr double kSamples[7] = {0.5, 0.25, 0.75, 0.118, 0.382, 0.618, 0.882};
/// The step of the one-sided derivative stencil, as a share of its span.
constexpr double kStencilStep = 1.0e-3;
/// A break is a corner when sharing one derivative across it moves a pole
/// further than this, millimetres.
constexpr double kSmoothMm = 1.0e-7;
/// Spans a fitted curve may hold.
constexpr size_t kMaxSpans = 4096;
/// The narrowest span, and the closest breaks kept, as a share of the range.
constexpr double kLeastSpanShare = 1.0e-9;
/// Fourth-order one-sided derivative stencil: (steps into the span, weight).
constexpr double kStencil[5][2] = {{0, -25}, {1, 48}, {2, -36}, {3, 16}, {4, -3}};

using V3 = std::array<double, 3>;
using PointFn = std::function<std::optional<V3>(double)>;

V3 sub(const V3& a, const V3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 axpy(const V3& a, double s, const V3& b) { return {a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]}; }
double norm(const V3& a) { return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }

struct Span {
  double low = 0, high = 0;
  V3 ends[2]{};
  V3 derivatives[2]{};

  static std::optional<Span> make(const PointFn& point, double low, double high) {
    double width = high - low;
    if (!(width > 0.0 && std::isfinite(width))) return std::nullopt;
    auto derivative = [&](double at, double step) -> std::optional<V3> {
      V3 sum{0, 0, 0};
      for (const auto& w : kStencil) {
        auto sample = point(at + step * w[0]);
        if (!sample) return std::nullopt;
        sum = axpy(sum, w[1], *sample);
      }
      for (double& v : sum) v /= 12.0 * step;
      return sum;
    };
    double step = width * kStencilStep;
    auto start = point(low);
    auto end = point(high);
    if (!start || !end) return std::nullopt;
    auto d0 = derivative(low, step);
    auto d1 = derivative(high, -step);
    if (!d0 || !d1) return std::nullopt;
    Span s;
    s.low = low;
    s.high = high;
    s.ends[0] = *start;
    s.ends[1] = *end;
    s.derivatives[0] = *d0;
    s.derivatives[1] = *d1;
    return s;
  }

  std::array<V3, 2> inner_poles() const {
    double third = (high - low) / 3.0;
    return {axpy(ends[0], third, derivatives[0]), axpy(ends[1], -third, derivatives[1])};
  }

  V3 at(double s) const {
    auto [p1, p2] = inner_poles();
    double r = 1.0 - s;
    double w[4] = {r * r * r, 3.0 * r * r * s, 3.0 * r * s * s, s * s * s};
    const V3* poles[4] = {&ends[0], &p1, &p2, &ends[1]};
    V3 out{0, 0, 0};
    for (int i = 0; i < 4; ++i) out = axpy(out, w[i], *poles[i]);
    return out;
  }

  std::optional<double> departure(const PointFn& point) const {
    double worst = 0.0;
    for (double f : kSamples) {
      auto exact = point(low + (high - low) * f);
      if (!exact) return std::nullopt;
      double gap = norm(sub(at(f), *exact));
      if (!std::isfinite(gap)) return std::nullopt;
      worst = std::max(worst, gap);
    }
    return worst;
  }
};

}  // namespace

std::optional<NurbsCurveGeometry> fit_curve(const std::function<std::optional<Vec3>(double)>& raw,
                                            std::array<double, 2> range,
                                            const std::vector<double>& breaks,
                                            const std::function<bool(double, double)>& resolved) {
  auto [low, high] = range;
  if (!(std::isfinite(low) && std::isfinite(high) && low < high)) return std::nullopt;
  PointFn point = [&](double t) -> std::optional<V3> {
    auto p = raw(std::clamp(t, low, high));
    if (!p || !std::isfinite(p->x) || !std::isfinite(p->y) || !std::isfinite(p->z))
      return std::nullopt;
    return V3{p->x, p->y, p->z};
  };
  // Breaks within a billionth of the range of a kept break or an end are one.
  double least = (high - low) * kLeastSpanShare;
  std::vector<double> inner;
  for (double t : breaks)
    if (std::isfinite(t) && t > low + least && t < high - least) inner.push_back(t);
  std::sort(inner.begin(), inner.end());
  std::vector<double> parameters{low};
  double last = low;
  for (double t : inner)
    if (t - last > least) {
      parameters.push_back(t);
      last = t;
    }
  parameters.push_back(high);

  std::vector<Span> pending;
  for (size_t i = parameters.size() - 1; i > 0; --i) {
    auto span = Span::make(point, parameters[i - 1], parameters[i]);
    if (!span) return std::nullopt;
    pending.push_back(*span);
  }
  std::vector<Span> spans;
  while (!pending.empty()) {
    Span span = pending.back();
    pending.pop_back();
    auto departure = span.departure(point);
    if (!departure) return std::nullopt;
    if (*departure <= kFitToleranceMm * kSampleMargin && resolved(span.low, span.high)) {
      spans.push_back(span);
      continue;
    }
    double middle = 0.5 * (span.low + span.high);
    if (spans.size() + pending.size() + 2 > kMaxSpans || middle - span.low <= least)
      return std::nullopt;
    auto upper = Span::make(point, middle, span.high);
    auto lower = Span::make(point, span.low, middle);
    if (!upper || !lower) return std::nullopt;
    pending.push_back(*upper);
    pending.push_back(*lower);
  }
  if (spans.empty()) return std::nullopt;

  auto corner = [](const Span& a, const Span& b) {
    double widest = std::max(a.high - a.low, b.high - b.low);
    return norm(sub(a.derivatives[1], b.derivatives[0])) * widest / 6.0 > kSmoothMm;
  };
  auto mean = [](const V3& a, const V3& b) {
    V3 m = axpy(a, 1.0, b);
    for (double& v : m) v /= 2.0;
    return m;
  };
  std::vector<double> knots(4, low);
  std::vector<V3> poles{spans.front().ends[0]};
  for (size_t i = 0; i < spans.size(); ++i) {
    const Span& span = spans[i];
    auto [first, second] = span.inner_poles();
    // A smooth break shares the mean derivative.
    if (i > 0 && !corner(spans[i - 1], span))
      first = axpy(span.ends[0], (span.high - span.low) / 3.0,
                   mean(spans[i - 1].derivatives[1], span.derivatives[0]));
    poles.push_back(first);
    if (i + 1 < spans.size()) {
      const Span& next = spans[i + 1];
      if (!corner(span, next)) {
        second = axpy(span.ends[1], -(span.high - span.low) / 3.0,
                      mean(span.derivatives[1], next.derivatives[0]));
        poles.push_back(second);
        knots.insert(knots.end(), 2, span.high);
      } else {
        poles.push_back(second);
        poles.push_back(span.ends[1]);
        knots.insert(knots.end(), 3, span.high);
      }
    } else {
      poles.push_back(second);
      poles.push_back(span.ends[1]);
      knots.insert(knots.end(), 4, span.high);
    }
  }
  NurbsCurveGeometry out;
  out.degree = 3;
  out.knots = std::move(knots);
  for (const auto& p : poles) out.poles.push_back(Vec3{p[0], p[1], p[2]});
  if (out.knots.size() != out.poles.size() + 4) return std::nullopt;
  return out;
}

}  // namespace swglb::brep
