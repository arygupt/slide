#include <chrono>
#include <cstring>

#include "Check.h"

namespace check {

namespace {
int currentFailures = 0;
}

std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

void fail(const char* file, int line, const std::string& message) {
    ++currentFailures;
    std::printf("    %s:%d: %s\n", file, line, message.c_str());
}

}  // namespace check

// Usage: slide-tests [substring]  -- runs tests whose name contains substring.
int main(int argc, char** argv) {
    using Clock = std::chrono::steady_clock;
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int run = 0;
    int failed = 0;
    for (const check::TestCase& test : check::registry()) {
        if (filter && !std::strstr(test.name, filter)) continue;
        check::currentFailures = 0;
        const auto start = Clock::now();
        test.fn();
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        ++run;
        const bool ok = check::currentFailures == 0;
        if (!ok) ++failed;
        std::printf("%s %-48s %8.1f ms\n", ok ? "[ pass ]" : "[ FAIL ]", test.name, ms);
    }
    std::printf("\n%d/%d tests passed\n", run - failed, run);
    return failed == 0 ? 0 : 1;
}
