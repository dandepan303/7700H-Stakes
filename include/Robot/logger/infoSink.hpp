#pragma once

#include "Robot/logger/message.hpp"
#include "Robot/logger/baseSink.hpp"

namespace Robot {
/**
 * @brief Sink for sending messages to the terminal.
 *
 * This is the primary way of interacting with Robot's logging implementation. This sink is used for printing useful
 * information to the user's terminal.
 * <h3> Example Usage </h3>
 * @code
 * Robot::infoSink()->setLowestLevel(Robot::Level::INFO);
 * Robot::infoSink()->info("info: {}!", "my cool info here");
 * // Specify the order or placeholders
 * Robot::infoSink()->debug("{1} {0}!","world", "hello");
 * // Specify the precision of floating point numbers
 * Robot::infoSink()->warn("Thingy exceeded value: {:.2f}!", 93.1234);
 * @endcode
 */
class InfoSink : public BaseSink {
    public:
        /**
         * @brief Construct a new Info Sink object
         */
        InfoSink();
    private:
        /**
         * @brief Log the given message
         *
         * @param message
         */
        void sendMessage(const Message& message) override;
};
} // namespace Robot
