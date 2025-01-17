#include "Robot/logger/message.hpp"

namespace Robot {
std::string format_as(Level level) {
    switch (level) {
        case Level::DEBUG: return "DEBUG";
        case Level::INFO: return "INFO";
        case Level::WARN: return "WARN";
        case Level::ERROR: return "ERROR";
        case Level::FATAL: return "FATAL";
    }
    __builtin_unreachable();
}
} // namespace Robot