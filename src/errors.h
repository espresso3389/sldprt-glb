// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdexcept>
#include <string>

namespace swglb {

/// Structurally malformed input; mirrors cadmpeg `CodecError::Malformed`.
/// The conversion fails and the caller keeps the saved preview image.
struct DecodeError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

}  // namespace swglb
