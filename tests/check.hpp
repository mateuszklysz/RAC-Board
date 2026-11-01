// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Mateusz Kłysz
// A tiny test harness: CHECK and CHECK_NEAR print the failing line and count it; main returns it.
#pragma once

#include <cmath>
#include <cstdio>

inline int g_failures = 0;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            std::printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);                   \
            g_failures++;                                                                          \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                                      \
    do {                                                                                           \
        const double check_a = (a), check_b = (b);                                                 \
        if (!(std::fabs(check_a - check_b) <= (eps))) {                                            \
            std::printf("%s:%d: %s = %.9g, expected %.9g\n", __FILE__, __LINE__, #a, check_a,      \
                        check_b);                                                                  \
            g_failures++;                                                                          \
        }                                                                                          \
    } while (0)

#define TEST_MAIN_END()                                                                            \
    do {                                                                                           \
        if (g_failures) {                                                                          \
            std::printf("%d failure(s)\n", g_failures);                                            \
        }                                                                                          \
        return g_failures ? 1 : 0;                                                                 \
    } while (0)
