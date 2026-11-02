// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/context.hpp"

#include <string>

#include "jak/cpad.hpp"

namespace rac1 {

namespace {

// A button setting's value as the pad buttons held for it. R1 and R2 do the same, as Ratchet
// crouches on either.
PadButtons pad_buttons(const std::string& v) {
    using namespace jak::pad;
    const auto b = [](uint32_t any, uint32_t all) {
        return PadButtons{static_cast<uint16_t>(any), static_cast<uint16_t>(all)};
    };
    if (v == "r1r2x") return b(r1 | r2, x);
    if (v == "r1r2") return b(r1 | r2, 0);
    if (v == "r1x") return b(0, r1 | x);
    if (v == "r1") return b(0, r1);
    if (v == "l1") return b(0, l1);
    if (v == "r2") return b(0, r2);
    if (v == "l2") return b(0, l2);
    if (v == "r3") return b(0, r3);
    if (v == "circle") return b(0, circle);
    if (v == "square") return b(0, square);
    return {};
}

} // namespace

Context& ctx() {
    static Context c;
    return c;
}

void load_settings(openrac::Mod& m) {
    Settings& s = ctx().settings;
    const std::string button = m.setting<std::string>("board_button");
    s.toggle = button == "r2"     ? Toggle::r2
               : button == "l2r2" ? Toggle::l2r2
               : button == "down" ? Toggle::down
                                  : Toggle::up;
    s.keys.clear();
    const std::string keys = m.setting<std::string>("board_key");
    size_t start = 0;
    while (start <= keys.size()) {
        const size_t comma = keys.find(',', start);
        const int bind = openrac::bind_from_name(keys.substr(start, comma == std::string::npos ? comma : comma - start));
        if (bind != OPENRAC_BIND_NONE) s.keys.push_back(bind);
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    s.trick = pad_buttons(m.setting<std::string>("trick_button"));
    s.grind = pad_buttons(m.setting<std::string>("grind_button"));
    s.grab = pad_buttons(m.setting<std::string>("grab_button"));
    s.trick_display = m.setting<bool>("trick_display");
    s.board_camera = m.setting<bool>("board_camera");
    s.heat_haze = m.setting<bool>("heat_haze");
    s.trace = m.setting<bool>("trace");
}

bool playing() { return mod().play_status().phase == OPENRAC_PLAY_PLAYING; }

void trace(const std::string& line) {
    if (settings().trace) mod().info(line);
}

} // namespace rac1
