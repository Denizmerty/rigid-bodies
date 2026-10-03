#pragma once

#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace rigidbodies::testing
{

    class Failure : public std::exception
    {
    public:
        explicit Failure(std::string message) : message_(std::move(message))
        {
        }

        [[nodiscard]] const char* what() const noexcept override
        {
            return message_.c_str();
        }

    private:
        std::string message_;
    };

    using TestFunction = void (*)();

    struct TestCase
    {
        std::string_view name;
        TestFunction function;
    };

    inline std::vector<TestCase>& registry()
    {
        static std::vector<TestCase> tests;
        return tests;
    }

    class Registrar
    {
    public:
        Registrar(std::string_view name, TestFunction function)
        {
            registry().push_back({ name, function });
        }
    };

    // The assertion macros supply the source file and line explicitly.
    [[noreturn]] inline void fail_at(std::string_view message, const char* file, int line)
    {
        std::ostringstream output;
        output << file << ':' << line << ": " << message;
        throw Failure(output.str());
    }

    inline void expect_at(bool condition, std::string_view message, const char* file, int line)
    {
        if (!condition)
        {
            fail_at(message, file, line);
        }
    }

    inline void expect_near_at(double actual, double expected, double tolerance, std::string_view message, const char* file, int line)
    {
        if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance)
        {
            std::ostringstream output;
            output << message << " (expected " << expected << " within " << tolerance << ", measured " << actual << ')';
            fail_at(output.str(), file, line);
        }
    }

    inline int run_all()
    {
        std::size_t failures = 0;
        for (const auto& test : registry())
        {
            try
            {
                test.function();
                std::cout << "pass  " << test.name << '\n';
            }
            catch (const std::exception& error)
            {
                ++failures;
                std::cout << "FAIL  " << test.name << "\n      " << error.what() << '\n';
            }
        }

        std::cout << registry().size() - failures << '/' << registry().size() << " tests passed\n";
        return failures == 0 ? 0 : 1;
    }

} // namespace rigidbodies::testing

#define RIGIDBODIES_TEST_CONCAT_INNER(left, right) left##right
#define RIGIDBODIES_TEST_CONCAT(left, right) RIGIDBODIES_TEST_CONCAT_INNER(left, right)

#define RIGIDBODIES_TEST(name)                                                                        \
    static void RIGIDBODIES_TEST_CONCAT(test_body_, __LINE__)();                                      \
    static const rigidbodies::testing::Registrar RIGIDBODIES_TEST_CONCAT(test_registrar_, __LINE__) { \
        name, &RIGIDBODIES_TEST_CONCAT(test_body_, __LINE__)                                          \
    };                                                                                                \
    static void RIGIDBODIES_TEST_CONCAT(test_body_, __LINE__)()

// Variadic rather than taking named parameters, because a braced argument such as
// Vec2 { 0.0, 0.0 } carries a comma that the preprocessor would otherwise read as an argument
// separator. Forwarding the whole list to the function lets the compiler, which understands braces,
// do the splitting instead.
#define RIGIDBODIES_EXPECT(...) rigidbodies::testing::expect_at(__VA_ARGS__, __FILE__, __LINE__)

#define RIGIDBODIES_EXPECT_NEAR(...) rigidbodies::testing::expect_near_at(__VA_ARGS__, __FILE__, __LINE__)

#define RIGIDBODIES_FAIL(...) rigidbodies::testing::fail_at(__VA_ARGS__, __FILE__, __LINE__)
