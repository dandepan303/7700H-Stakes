#include <iostream>
#include <ostream>

#include "Robot/logger/stdout.hpp"

namespace Robot {
BufferedStdout::BufferedStdout()
    : Buffer([](const std::string& text) { std::cout << text << std::flush; }) {
    setRate(50);
}

BufferedStdout& bufferedStdout() {
    static BufferedStdout bufferedStdout;
    return bufferedStdout;
}
} // namespace Robot