#define FMT_HEADER_ONLY
#include "fmt/core.h"

#include "Robot/pose.hpp"

Robot::Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

Robot::Pose Robot::Pose::operator+(const Robot::Pose& other) const {
    return Robot::Pose(this->x + other.x, this->y + other.y, this->theta);
}

Robot::Pose Robot::Pose::operator-(const Robot::Pose& other) const {
    return Robot::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float Robot::Pose::operator*(const Robot::Pose& other) const { return this->x * other.x + this->y * other.y; }

Robot::Pose Robot::Pose::operator*(const float& other) const {
    return Robot::Pose(this->x * other, this->y * other, this->theta);
}

Robot::Pose Robot::Pose::operator/(const float& other) const {
    return Robot::Pose(this->x / other, this->y / other, this->theta);
}

Robot::Pose Robot::Pose::lerp(Robot::Pose other, float t) const {
    return Robot::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float Robot::Pose::distance(Robot::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }

float Robot::Pose::angle(Robot::Pose other) const { return std::atan2(other.y - this->y, other.x - this->x); }

Robot::Pose Robot::Pose::rotate(float angle) const {
    return Robot::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                        this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}

std::string Robot::format_as(const Robot::Pose& pose) {
    // the double brackets become single brackets
    return fmt::format("Robot::Pose {{ x: {}, y: {}, theta: {} }}", pose.x, pose.y, pose.theta);
}
