#include "Robot/logger/logger.hpp"

namespace Robot {
std::shared_ptr<InfoSink> infoSink() {
    static std::shared_ptr<InfoSink> infoSink = std::make_shared<InfoSink>();
    return infoSink;
}

std::shared_ptr<TelemetrySink> telemetrySink() {
    static std::shared_ptr<TelemetrySink> telemetrySink = std::make_shared<TelemetrySink>();
    return telemetrySink;
}
} // namespace Robot