// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// The level's collision for the board's collision code, in Jak II's units and axes.
#pragma once

#include "jak/host.hpp"

namespace rac1 {

class World : public jak::World {
  public:
    int triangles(const jak::Vec& lo, const jak::Vec& hi, jak::CollideTri* out, int max) override;
};

} // namespace rac1
