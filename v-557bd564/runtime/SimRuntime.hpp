#pragma once
#include <iostream>
#include <string>

namespace runtime {

struct SimRuntime {
    void reportExecutionState(const std::string& state) noexcept
    {
        std::cout << "[Runtime] execution state: " << state << '\n';
    }

    void logInfo(const std::string& msg) noexcept
    {
        std::cout << msg << '\n';
    }

    void logError(const std::string& msg) noexcept
    {
        std::cerr << msg << '\n';
    }
};

}