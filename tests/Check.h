#pragma once

// Minimal self-registering test harness; keeps the project dependency-free.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace check {

struct TestCase {
    const char* name;
    void (*fn)();
};

std::vector<TestCase>& registry();
void fail(const char* file, int line, const std::string& message);

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

inline std::string format(const char* fmt, double a, double b, double c) {
    char buf[256];
    std::snprintf(buf, sizeof buf, fmt, a, b, c);
    return buf;
}

}  // namespace check

#define TEST(name)                                               \
    static void name();                                          \
    static const check::Registrar name##Registrar(#name, name);  \
    static void name()

#define CHECK(cond)                                         \
    do {                                                    \
        if (!(cond)) check::fail(__FILE__, __LINE__, #cond); \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                               \
    do {                                                                                      \
        const double a_ = (actual), e_ = (expected), t_ = (tolerance);                        \
        if (!(std::abs(a_ - e_) <= t_)) {                                                     \
            check::fail(__FILE__, __LINE__,                                                   \
                        #actual " = " + check::format("%.6g, expected %.6g +/- %.3g", a_, e_, t_)); \
        }                                                                                     \
    } while (0)

#define CHECK_RANGE(actual, lo, hi)                                                         \
    do {                                                                                    \
        const double a_ = (actual), l_ = (lo), h_ = (hi);                                   \
        if (!(a_ >= l_ && a_ <= h_)) {                                                      \
            check::fail(__FILE__, __LINE__,                                                 \
                        #actual " = " + check::format("%.6g, expected in [%.6g, %.6g]", a_, l_, h_)); \
        }                                                                                   \
    } while (0)
