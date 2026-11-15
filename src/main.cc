// SPDX-License-Identifier: Apache-2.0
// sldprt-glb: SOLIDWORKS part/assembly saved display data to GLB.
//
//   sldprt-glb [options] <input.SLDPRT|.SLDASM> <output.glb>
//       Exit 0 on success; when the document has no usable saved display
//       data, print "solidworks_saved_image" and exit 3.
//       --generator <text>   asset.generator (default "sldprt-glb")
//       --extras-key <key>   node extras key of the presentation data
//                            (default "solidworksPresentation")
//   sldprt-glb dump <input> [--no-mesh] [--no-brep]
//       CADIR-shaped JSON of the decoded model on stdout.
//   sldprt-glb sections <input>
//       Container sections (ordinal, size, name).
//   sldprt-glb brep-glb <input> <output.glb>
//       The tessellated B-rep used for stale-mesh regeneration.
//   sldprt-glb from-cadir [options] <cadir.json> <output.glb> [--part]
//       GLB from a cadmpeg CADIR document (writer verification).
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "cadir_load.h"
#include "container.h"
#include "engine.h"
#include "errors.h"
#include "glb.h"
#include "model_json.h"
#include "regenerate.h"

using namespace swglb;

namespace {

bool read_file(const std::filesystem::path& path, Bytes* out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  out->assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  return true;
}

std::optional<Container> open(const char* path) {
  Bytes image;
  if (!read_file(path, &image)) {
    std::cerr << "cannot read " << path << "\n";
    return std::nullopt;
  }
  ContainerError error;
  auto c = read_container(image, &error);
  if (!c) std::cerr << error.message << "\n";
  return c;
}

int usage() {
  std::cerr << "usage: sldprt-glb [--generator <text>] [--extras-key <key>] <input.SLDPRT|.SLDASM> <output.glb>\n"
               "       sldprt-glb dump <input> [--no-mesh] [--no-brep]\n"
               "       sldprt-glb sections <input>\n"
               "       sldprt-glb brep-glb <input> <output.glb>\n"
               "       sldprt-glb from-cadir [--generator <text>] [--extras-key <key>] <cadir.json> <output.glb> [--part]\n";
  return 2;
}

int convert(const std::filesystem::path& input, const std::filesystem::path& output, GlbOptions options) {
  std::string ext = input.extension().string();
  std::string out_ext = output.extension().string();
  if (!(iequals(ext, ".sldprt") || iequals(ext, ".sldasm")) || !iequals(out_ext, ".glb")) {
    std::cerr << "Expected .SLDPRT or .SLDASM input and .glb output\n";
    return 2;
  }
  if (std::filesystem::exists(output)) {
    std::cerr << "Output already exists\n";
    return 2;
  }
  try {
    Bytes image;
    if (!read_file(input, &image)) throw DecodeError("cannot read input");
    ContainerError error;
    auto container = read_container(image, &error);
    if (!container) throw DecodeError(error.message);
    Model model = decode_document(*container);
    options.part = iequals(ext, ".sldprt");
    Bytes glb = write_glb(model, options);
    std::ofstream out(output, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(glb.data()), static_cast<std::streamsize>(glb.size()));
    if (!out) throw DecodeError("cannot write output");
    return 0;
  } catch (const std::exception& e) {
    if (std::getenv("SWGLB_DEBUG")) std::cerr << "error: " << e.what() << "\n";
    std::error_code ignored;
    std::filesystem::remove(output, ignored);
    std::puts("solidworks_saved_image");
    return 3;
  }
}

/// Remove the GLB naming options from `args`; false on a malformed option.
bool take_glb_options(std::vector<std::string>& args, GlbOptions& options) {
  std::vector<std::string> rest;
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--generator" || args[i] == "--extras-key") {
      if (i + 1 >= args.size() || args[i + 1].empty()) return false;
      (args[i] == "--generator" ? options.generator : options.extras_key) = args[i + 1];
      ++i;
    } else {
      rest.push_back(args[i]);
    }
  }
  args = std::move(rest);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) return usage();
  std::string command = argv[1];
  if (command == "sections") {
    auto c = open(argv[2]);
    if (!c) return 1;
    for (const auto& s : c->sections)
      std::printf("%zu\t%zu\t%s\n", s.ordinal, s.payload.size(), s.source_stream.c_str());
    return 0;
  }
  if (command == "dump") {
    bool mesh = true;
    DecodeOptions options;
    for (int i = 3; i < argc; ++i) {
      if (!std::strcmp(argv[i], "--no-mesh")) mesh = false;
      else if (!std::strcmp(argv[i], "--no-brep")) options.brep = false;
      else return usage();
    }
    auto c = open(argv[2]);
    if (!c) return 1;
    try {
      Model model = decode_document(*c, options);
      write_model_json(std::cout, model, mesh);
    } catch (const std::exception& e) {
      std::cerr << "error: " << e.what() << "\n";
      return 1;
    }
    return 0;
  }
  if (command == "brep-glb" && argc == 4) {
    auto c = open(argv[2]);
    if (!c) return 1;
    try {
      Bytes glb = regenerate_brep_glb(decode_document(*c));
      std::ofstream out(argv[3], std::ios::binary | std::ios::trunc);
      out.write(reinterpret_cast<const char*>(glb.data()), static_cast<std::streamsize>(glb.size()));
      return out ? 0 : 1;
    } catch (const std::exception& e) {
      std::cerr << "error: " << e.what() << "\n";
      return 1;
    }
  }
  if (command == "from-cadir") {
    // GLB from a cadmpeg CADIR (writer verification; --part enables the
    // SLDPRT stale-mesh check).
    std::vector<std::string> args(argv + 2, argv + argc);
    GlbOptions options;
    if (!take_glb_options(args, options)) return usage();
    options.part = false;
    if (!args.empty() && args.back() == "--part") {
      options.part = true;
      args.pop_back();
    }
    if (args.size() != 2) return usage();
    Bytes text;
    if (!read_file(args[0], &text)) return 1;
    try {
      Model model = load_cadir(std::string_view(reinterpret_cast<const char*>(text.data()), text.size()));
      Bytes glb = write_glb(model, options);
      std::ofstream out(args[1], std::ios::binary | std::ios::trunc);
      out.write(reinterpret_cast<const char*>(glb.data()), static_cast<std::streamsize>(glb.size()));
      return out ? 0 : 1;
    } catch (const std::exception& e) {
      std::cerr << "error: " << e.what() << "\n";
      return 3;
    }
  }
  std::vector<std::string> args(argv + 1, argv + argc);
  GlbOptions options;
  if (!take_glb_options(args, options) || args.size() != 2) return usage();
  return convert(args[0], args[1], std::move(options));
}
