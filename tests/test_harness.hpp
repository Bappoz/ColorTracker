// Harness mínimo de teste: registro estático + macros. ~60 linhas contra uma
// dependência de gtest que não agregaria nada num projeto sem mock nem fixture.
#pragma once

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace ecvtest {

struct TestCase {
    const char* name;
    void (*fn)();
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

inline int& failures() {
    static int f = 0;
    return f;
}

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

inline void fail(const char* file, int line, const std::string& msg) {
    std::printf("  FALHA %s:%d — %s\n", file, line, msg.c_str());
    ++failures();
}

inline int run_all() {
    int failed_tests = 0;
    for (const TestCase& t : registry()) {
        const int before = failures();
        std::printf("[ RUN ] %s\n", t.name);
        t.fn();
        if (failures() > before) {
            ++failed_tests;
            std::printf("[FALHA] %s\n", t.name);
        } else {
            std::printf("[  OK ] %s\n", t.name);
        }
    }
    std::printf("\n%zu testes, %d com falha\n", registry().size(), failed_tests);
    return failed_tests == 0 ? 0 : 1;
}

}  // namespace ecvtest

#define ECV_TEST(name)                                        \
    static void name();                                       \
    static ecvtest::Registrar registrar_##name(#name, &name); \
    static void name()

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) ecvtest::fail(__FILE__, __LINE__, "esperado: " #cond); \
    } while (0)

#define CHECK_EQ(a, b)                                                                    \
    do {                                                                                  \
        const auto va_ = (a);                                                             \
        const auto vb_ = (b);                                                             \
        if (!(va_ == vb_))                                                                \
            ecvtest::fail(__FILE__, __LINE__,                                             \
                          std::string(#a " == " #b " (") + std::to_string(va_) + " vs " + \
                              std::to_string(vb_) + ")");                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                            \
    do {                                                                                 \
        const double va_ = static_cast<double>(a);                                       \
        const double vb_ = static_cast<double>(b);                                       \
        if (std::fabs(va_ - vb_) > (tol))                                                \
            ecvtest::fail(__FILE__, __LINE__,                                            \
                          std::string(#a " ~ " #b " (") + std::to_string(va_) + " vs " + \
                              std::to_string(vb_) + ")");                                \
    } while (0)
