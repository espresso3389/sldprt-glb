// SPDX-License-Identifier: Apache-2.0
// Minimal ordered JSON value whose serialisation matches Python's
// `json.dumps(doc, separators=(',', ':'), allow_nan=False)` byte for byte
// (insertion-ordered objects, float repr, ensure_ascii escaping).
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace swglb {

class JValue {
 public:
  using Object = std::vector<std::pair<std::string, JValue>>;
  using Array = std::vector<JValue>;

  JValue() : v_(nullptr) {}
  JValue(std::nullptr_t) : v_(nullptr) {}
  JValue(bool b) : v_(b) {}
  JValue(int i) : v_(static_cast<int64_t>(i)) {}
  JValue(int64_t i) : v_(i) {}
  JValue(size_t i) : v_(static_cast<int64_t>(i)) {}
  JValue(double d) : v_(d) {}
  JValue(const char* s) : v_(std::string(s)) {}
  JValue(std::string s) : v_(std::move(s)) {}
  JValue(Array a) : v_(std::make_shared<Array>(std::move(a))) {}
  JValue(Object o) : v_(std::make_shared<Object>(std::move(o))) {}

  static JValue object() { return JValue(Object{}); }
  static JValue array() { return JValue(Array{}); }

  bool is_object() const { return std::holds_alternative<std::shared_ptr<Object>>(v_); }
  bool is_array() const { return std::holds_alternative<std::shared_ptr<Array>>(v_); }
  bool is_float() const { return std::holds_alternative<double>(v_); }
  bool is_int() const { return std::holds_alternative<int64_t>(v_); }
  bool is_string() const { return std::holds_alternative<std::string>(v_); }
  bool is_bool() const { return std::holds_alternative<bool>(v_); }
  const std::string& str() const { return std::get<std::string>(v_); }
  bool as_bool() const { return std::get<bool>(v_); }

  Object& obj() { return *std::get<std::shared_ptr<Object>>(v_); }
  const Object& obj() const { return *std::get<std::shared_ptr<Object>>(v_); }
  Array& arr() { return *std::get<std::shared_ptr<Array>>(v_); }
  const Array& arr() const { return *std::get<std::shared_ptr<Array>>(v_); }
  double as_double() const {
    return is_float() ? std::get<double>(v_) : static_cast<double>(std::get<int64_t>(v_));
  }
  int64_t as_int() const { return std::get<int64_t>(v_); }

  /// Object member access; inserts at the end when absent (Python dict
  /// assignment order).
  JValue& operator[](std::string_view key);
  const JValue* find(std::string_view key) const;
  bool contains(std::string_view key) const { return find(key) != nullptr; }
  /// Python `dict.setdefault`.
  JValue& setdefault(std::string_view key, JValue value);
  void push_back(JValue value) { arr().push_back(std::move(value)); }
  size_t size() const { return is_array() ? arr().size() : obj().size(); }

  /// Deep copy (values share structure by default).
  JValue clone() const;

  /// Python `json.loads`: numbers with '.', 'e' or 'E' become floats, others
  /// ints; object key order is kept (later duplicates replace earlier values
  /// in place, as a dict does). Throws DecodeError on malformed input.
  static JValue parse(std::string_view text);

  void dump(std::string& out) const;
  std::string dump() const {
    std::string s;
    dump(s);
    return s;
  }

 private:
  std::variant<std::nullptr_t, bool, int64_t, double, std::string, std::shared_ptr<Array>,
               std::shared_ptr<Object>>
      v_;
};

/// Python `repr(float)`. Throws DecodeError for NaN/Infinity (allow_nan=False).
std::string python_float_repr(double v);

/// The double Python obtains when parsing serde's shortest f32 text for `f`
/// (CADIR colours are f32 serialised by serde_json).
double f32_as_json_double(float f);

}  // namespace swglb
