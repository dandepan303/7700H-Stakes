#pragma once

#include "Robot/logger/baseSink.hpp"

namespace Robot {
/**
 * @brief Sink for sending telemetry data.
 *
 * This is the primary way of interacting with the telemetry portion of Robot's logging implementation. This sink is
 * used for sending data that is not meant to be viewed by the user, but will still be used by something else, like a
 * data visualization tool. Messages sent through this sink will not be cleared from the terminal and not be visible to
 * the user.

 * <h3> Example Usage </h3>
 * @code
 * Robot::telemetrySink()->setLowestLevel(Robot::Level::INFO);
 * Robot::telemetrySink()->info("{},{}", motor1.get_temperature(), motor2.get_temperature());
 * @endcode
 */
class TelemetrySink : public BaseSink {
    public:
        /**
         * @brief Construct a new Telemetry Sink object
         */
        TelemetrySink();
    private:
        /**
         * @brief Log the given message
         *
         * @param message
         */
        void sendMessage(const Message& message) override;
};
} // namespace Robot
