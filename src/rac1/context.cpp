// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
#include "rac1/context.hpp"

#include <string>

namespace rac1 {

Context& ctx() {
    static Context c;
    return c;
}

void load_settings(openrac::Mod& m) {
    Settings& s = ctx().settings;
    const std::string button = m.setting<std::string>("board_button");
    s.toggle = button == "r2" ? Toggle::r2 : (button == "l2r2" ? Toggle::l2r2 : Toggle::down);
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
    s.swap_l1_r1 = m.setting<bool>("swap_l1_r1");
    s.trick_display = m.setting<bool>("trick_display");
    s.trace = m.setting<bool>("trace");
}

bool playing() { return mod().play_status().phase == OPENRAC_PLAY_PLAYING; }

void trace(const std::string& line) {
    if (settings().trace) mod().info(line);
}

} // namespace rac1
