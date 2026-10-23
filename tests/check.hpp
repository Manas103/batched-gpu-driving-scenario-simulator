#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>

// Minimal hand-rolled check macro, the pattern used by the other C++ repos
// in this portfolio's test suites: no framework dependency, one line per
// assertion, a running pass count, and a nonzero exit code on any failure.
static int g_checks_run = 0;
static int g_checks_failed = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        g_checks_run++;                                                          \
        if (!(cond)) {                                                           \
            g_checks_failed++;                                                   \
            fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
        }                                                                        \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                    \
    do {                                                                         \
        g_checks_run++;                                                          \
        double _a = (a), _b = (b), _t = (tol);                                   \
        if (!(fabs(_a - _b) <= _t)) {                                            \
            g_checks_failed++;                                                   \
            fprintf(stderr, "CHECK_NEAR FAILED: %s vs %s (%.9g vs %.9g, tol %.3g) (%s:%d)\n", \
                    #a, #b, _a, _b, _t, __FILE__, __LINE__);                      \
        }                                                                        \
    } while (0)

#define CHECK_SUITE_END()                                                        \
    do {                                                                         \
        printf("%d checks, %d failed\n", g_checks_run, g_checks_failed);         \
        return g_checks_failed == 0 ? 0 : 1;                                     \
    } while (0)
