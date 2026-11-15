// SPDX-License-Identifier: Apache-2.0
#include "json_value.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "errors.h"

namespace swglb {

JValue& JValue::operator[](std::string_view key) {
  auto& o = obj();
  for (auto& [k, v] : o)
    if (k == key) return v;
  o.emplace_back(std::string(key), JValue());
  return o.back().second;
}

const JValue* JValue::find(std::string_view key) const {
  if (!is_object()) return nullptr;
  for (const auto& [k, v] : obj())
    if (k == key) return &v;
  return nullptr;
}

JValue& JValue::setdefault(std::string_view key, JValue value) {
  auto& o = obj();
  for (auto& [k, v] : o)
    if (k == key) return v;
  o.emplace_back(std::string(key), std::move(value));
  return o.back().second;
}

JValue JValue::clone() const {
  if (is_array()) {
    Array a;
    a.reserve(arr().size());
    for (const auto& v : arr()) a.push_back(v.clone());
    return JValue(std::move(a));
  }
  if (is_object()) {
    Object o;
    o.reserve(obj().size());
    for (const auto& [k, v] : obj()) o.emplace_back(k, v.clone());
    return JValue(std::move(o));
  }
  return *this;
}

std::string python_float_repr(double v) {
  if (!std::isfinite(v)) throw DecodeError("Out of range float values are not JSON compliant");
  if (v == 0) return std::signbit(v) ? "-0.0" : "0.0";
  char buf[64];
  auto res = std::to_chars(buf, buf + sizeof buf, v, std::chars_format::scientific);
  std::string_view sci(buf, static_cast<size_t>(res.ptr - buf));
  bool negative = sci.front() == '-';
  if (negative) sci.remove_prefix(1);
  size_t e = sci.find('e');
  std::string digits;
  for (char c : sci.substr(0, e))
    if (c != '.') digits.push_back(c);
  int exponent = std::atoi(std::string(sci.substr(e + 1)).c_str());
  std::string out = negative ? "-" : "";
  int n = static_cast<int>(digits.size());
  if (exponent >= -4 && exponent < 16) {
    if (exponent >= 0) {
      if (n <= exponent + 1) {
        out += digits;
        out.append(static_cast<size_t>(exponent + 1 - n), '0');
        out += ".0";
      } else {
        out += digits.substr(0, static_cast<size_t>(exponent + 1));
        out += '.';
        out += digits.substr(static_cast<size_t>(exponent + 1));
      }
    } else {
      out += "0.";
      out.append(static_cast<size_t>(-exponent - 1), '0');
      out += digits;
    }
  } else {
    out += digits[0];
    if (n > 1) {
      out += '.';
      out += digits.substr(1);
    }
    char ebuf[16];
    std::snprintf(ebuf, sizeof ebuf, "e%c%02d", exponent < 0 ? '-' : '+', std::abs(exponent));
    out += ebuf;
  }
  return out;
}

double f32_as_json_double(float f) {
  char buf[64];
  auto res = std::to_chars(buf, buf + sizeof buf, f);
  double d = 0;
  std::from_chars(buf, res.ptr, d);
  return d;
}

namespace {

void dump_string(std::string& out, std::string_view s) {
  static const char* hex = "0123456789abcdef";
  auto u16 = [&](uint32_t cu) {
    out += "\\u";
    out += hex[(cu >> 12) & 15];
    out += hex[(cu >> 8) & 15];
    out += hex[(cu >> 4) & 15];
    out += hex[cu & 15];
  };
  out += '"';
  for (size_t i = 0; i < s.size();) {
    unsigned char c = static_cast<unsigned char>(s[i]);
    if (c < 0x80) {
      switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        default:
          if (c < 0x20) u16(c);
          else out += static_cast<char>(c);
      }
      ++i;
      continue;
    }
    // Decode UTF-8 (invalid bytes become U+FFFD).
    uint32_t cp = 0xfffd;
    size_t len = 1;
    if ((c & 0xe0) == 0xc0 && i + 1 < s.size()) {
      cp = ((c & 0x1fu) << 6) | (s[i + 1] & 0x3fu);
      len = 2;
    } else if ((c & 0xf0) == 0xe0 && i + 2 < s.size()) {
      cp = ((c & 0x0fu) << 12) | ((s[i + 1] & 0x3fu) << 6) | (s[i + 2] & 0x3fu);
      len = 3;
    } else if ((c & 0xf8) == 0xf0 && i + 3 < s.size()) {
      cp = ((c & 0x07u) << 18) | ((s[i + 1] & 0x3fu) << 12) | ((s[i + 2] & 0x3fu) << 6) |
           (s[i + 3] & 0x3fu);
      len = 4;
    }
    if (cp >= 0x10000) {
      cp -= 0x10000;
      u16(0xd800 + (cp >> 10));
      u16(0xdc00 + (cp & 0x3ff));
    } else {
      u16(cp);
    }
    i += len;
  }
  out += '"';
}

}  // namespace

void JValue::dump(std::string& out) const {
  std::visit(
      [&](const auto& v) {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, std::nullptr_t>) {
          out += "null";
        } else if constexpr (std::is_same_v<T, bool>) {
          out += v ? "true" : "false";
        } else if constexpr (std::is_same_v<T, int64_t>) {
          out += std::to_string(v);
        } else if constexpr (std::is_same_v<T, double>) {
          out += python_float_repr(v);
        } else if constexpr (std::is_same_v<T, std::string>) {
          dump_string(out, v);
        } else if constexpr (std::is_same_v<T, std::shared_ptr<Array>>) {
          out += '[';
          bool first = true;
          for (const auto& item : *v) {
            if (!first) out += ',';
            first = false;
            item.dump(out);
          }
          out += ']';
        } else {
          out += '{';
          bool first = true;
          for (const auto& [k, item] : *v) {
            if (!first) out += ',';
            first = false;
            dump_string(out, k);
            out += ':';
            item.dump(out);
          }
          out += '}';
        }
      },
      v_);
}

}  // namespace swglb
