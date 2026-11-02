// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/context.hpp"

#include <string>

#include "jak/cpad.hpp"

namespace rac1 {

namespace {

// A button setting's value as the pad buttons held for it.
uint16_t pad_buttons(const std::string& v) {
    using namespace jak::pad;
    const uint32_t b = v == "r1x"      ? r1 | x
                       : v == "r1"     ? r1
                       : v == "l1"     ? l1
                       : v == "r2"     ? r2
                       : v == "l2"     ? l2
                       : v == "r3"     ? r3
                       : v == "circle" ? circle
                       : v == "square" ? square
                                       : 0u;
    return static_cast<uint16_t>(b);
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
