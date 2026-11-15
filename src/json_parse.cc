// SPDX-License-Identifier: Apache-2.0
// JSON reader matching Python's json.loads for the GLB documents this engine
// merges (int/float distinction, insertion-ordered objects).
#include <charconv>

#include "errors.h"
#include "json_value.h"

namespace swglb {
namespace {

class Parser {
 public:
  explicit Parser(std::string_view s) : s_(s) {}

  JValue document() {
    JValue v = value(0);
    ws();
    if (i_ != s_.size()) fail();
    return v;
  }

 private:
  [[noreturn]] void fail() { throw DecodeError("Invalid JSON"); }
  void ws() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r'))
      ++i_;
  }
  bool eat(char c) {
    ws();
    if (i_ < s_.size() && s_[i_] == c) {
      ++i_;
      return true;
    }
    return false;
  }
  void expect(char c) {
    if (!eat(c)) fail();
  }
  JValue value(int depth) {
    if (depth > 512) fail();
    ws();
    if (i_ >= s_.size()) fail();
    char c = s_[i_];
    if (c == '{') {
      ++i_;
      JValue o = JValue::object();
      if (eat('}')) return o;
      do {
        ws();
        std::string k = string();
        expect(':');
        o[k] = value(depth + 1);
      } while (eat(','));
      expect('}');
      return o;
    }
    if (c == '[') {
      ++i_;
      JValue a = JValue::array();
      if (eat(']')) return a;
      do a.push_back(value(depth + 1));
      while (eat(','));
      expect(']');
      return a;
    }
    if (c == '"') return JValue(string());
    if (s_.substr(i_, 4) == "true") {
      i_ += 4;
      return JValue(true);
    }
    if (s_.substr(i_, 5) == "false") {
      i_ += 5;
      return JValue(false);
    }
    if (s_.substr(i_, 4) == "null") {
      i_ += 4;
      return JValue(nullptr);
    }
    return number();
  }
  JValue number() {
    size_t start = i_;
    bool is_float = false;
    if (i_ < s_.size() && s_[i_] == '-') ++i_;
    while (i_ < s_.size()) {
      char c = s_[i_];
      if (c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') is_float = true;
      else if (c < '0' || c > '9') break;
      ++i_;
    }
    const char* b = s_.data() + start;
    const char* e = s_.data() + i_;
    if (b == e) fail();
    if (is_float) {
      double d = 0;
      auto r = std::from_chars(b, e, d);
      if (r.ec != std::errc() || r.ptr != e) fail();
      return JValue(d);
    }
    int64_t v = 0;
    auto r = std::from_chars(b, e, v);
    if (r.ec != std::errc() || r.ptr != e) fail();
    return JValue(v);
  }
  uint32_t hex4() {
    if (i_ + 4 > s_.size()) fail();
    uint32_t v = 0;
    auto r = std::from_chars(s_.data() + i_, s_.data() + i_ + 4, v, 16);
    if (r.ptr != s_.data() + i_ + 4) fail();
    i_ += 4;
    return v;
  }
  static void utf8(std::string& out, uint32_t cp) {
    if (cp < 0x80) {
      out += static_cast<char>(cp);
    } else if (cp < 0x800) {
      out += static_cast<char>(0xc0 | (cp >> 6));
      out += static_cast<char>(0x80 | (cp & 0x3f));
    } else if (cp < 0x10000) {
      out += static_cast<char>(0xe0 | (cp >> 12));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3f));
      out += static_cast<char>(0x80 | (cp & 0x3f));
    } else {
      out += static_cast<char>(0xf0 | (cp >> 18));
      out += static_cast<char>(0x80 | ((cp >> 12) & 0x3f));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3f));
      out += static_cast<char>(0x80 | (cp & 0x3f));
    }
  }
  std::string string() {
    if (i_ >= s_.size() || s_[i_] != '"') fail();
    ++i_;
    std::string out;
    while (true) {
      if (i_ >= s_.size()) fail();
      char c = s_[i_++];
      if (c == '"') return out;
      if (c != '\\') {
        out += c;
        continue;
      }
      if (i_ >= s_.size()) fail();
      char esc = s_[i_++];
      switch (esc) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case '/': out += '/'; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        case 'u': {
          uint32_t cp = hex4();
          if (cp >= 0xd800 && cp < 0xdc00 && s_.substr(i_, 2) == "\\u") {
            size_t save = i_;
            i_ += 2;
            uint32_t lo = hex4();
            if (lo >= 0xdc00 && lo < 0xe000) cp = 0x10000 + ((cp - 0xd800) << 10) + (lo - 0xdc00);
            else i_ = save;
          }
          utf8(out, cp);
          break;
        }
        default: fail();
      }
    }
  }
  std::string_view s_;
  size_t i_ = 0;
};

}  // namespace

JValue JValue::parse(std::string_view text) { return Parser(text).document(); }

}  // namespace swglb
