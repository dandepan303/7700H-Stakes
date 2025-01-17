#include "main.h"
#include "Robot/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "gif-pros/gifclass.hpp"

using namespace std;

// Controller
pros::Controller Controller(pros::E_CONTROLLER_MASTER);

// Motor groups
pros::MotorGroup LeftMotors({-1, -13, -14}, pros::MotorGearset::blue);
pros::MotorGroup RightMotors({15, 17, 18}, pros::MotorGearset::blue);

// General motors
pros::Motor Intake(20, pros::MotorGearset::blue);
pros::MotorGroup Wall({21, -2});

// Pneumatics
pros::ADIPneumatics Clamp('A', false);
pros::ADIPneumatics Doinker('C', false);
pros::ADIPneumatics IntakeLift('B', false);

// Inertial sensor on port 10
pros::Imu imu(3);

// Rotation sensor on port 15 (Lady Brown)
pros::Rotation rotationSensor(12);

// Optical sensor for color sorting
pros::Optical Optical(11);

/**
 * Construct the Gif class
 * @param fname  the gif filename on the SD card (prefixed with /usd/)
 * @param parent the LVGL parent object
 */
Gif gif("/usd/mygif.gif", lv_scr_act());

// Drivetrain settings
Robot::Drivetrain drivetrain(
    &LeftMotors, // left motor group
    &RightMotors, // right motor group
    14, // 10 inch track width
    Robot::Omniwheel::NEW_275, // using new 2.75" omnis
    450, // drivetrain rpm is 450
    4 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
Robot::ControllerSettings linearController(
    10, // proportional gain (kP)
    0, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular motion controller
Robot::ControllerSettings angularController(
    2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// sensors for odometry
Robot::OdomSensors sensors(
    nullptr, // vertical tracking wheel
    nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
    nullptr, // horizontal tracking wheel
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

// input curve for throttle input during driver control
Robot::ExpoDriveCurve throttleCurve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// input curve for steer input during driver control
Robot::ExpoDriveCurve steerCurve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// create the chassis
Robot::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Constants for Lady Brown PID
const int numStates = 3;
// Make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {0, -1400, -13000};
int currState = 0;
int target = 0;

// Start Lady Brown Code
void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}

void liftControl() {
    double kp = 0.03;
    double error = target - rotationSensor.get_position();
    double velocity = kp * error;
    Wall.move(velocity);
}

// Color sorting code
void colorSorting() {
    if (Optical.get_proximity() > 250) {
        if ((Optical.get_hue() > 190) && (Optical.get_hue() < 220)) {
            Controller.rumble(".");
            pros::delay(100);
            Intake.move_velocity(0);
            pros::delay(500);
            Intake.move_velocity(600);
        }
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	static Gif startup("/usd/startup.gif", lv_scr_act());
    chassis.calibrate(); // calibrate sensors
	rotationSensor.set_position(0);
    Optical.set_led_pwm(100);

	 // Start Lady Brown lift control task
    pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });

	startup.clean();
	static Gif standford("/usd/stanford.gif", lv_scr_act());

     /*
     pros::Task colorSortingTask([]{
        while (true) {
            colorSorting();
            pros::delay(10);
        }
    });
    */

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // Robot::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
    
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features Robot has to offer
 */
void autonomous() {
    /*
    chassis.setPose(-53, -35, 270);
    chassis.moveToPose(-22, -24, 240, 1500, { .forwards = false});
    chassis.waitUntilDone();
    pros::delay(200);
    Clamp.toggle();
    pros::delay(200);
    Intake.move_velocity(600);
    chassis.moveToPoint(-23.5, -47, 1000, { .forwards = true});
    chassis.waitUntilDone();
    pros::delay(1500);
    Intake.move_velocity(0);    
    //HERE
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPose(-45, -5, 0, 2500);
    chassis.waitUntilDone();
    IntakeLift.toggle();
    Clamp.toggle();
    chassis.moveToPoint(-45, 3, 1000);
    chassis.waitUntilDone();
    IntakeLift.toggle();
    pros::delay(200);
    chassis.moveToPoint(-45, -10, 1000, {.forwards = false});
    chassis.waitUntilDone();
    Intake.move_velocity(0);
    Doinker.toggle();
    chassis.turnToHeading(90, 500, {.direction = Robot::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPose(-58, -12, 90, 3000, {.forwards = false});
    chassis.waitUntilDone();
    Doinker.toggle();
    Intake.move_velocity(600);
    pros::delay(1000);
    //HERE
    chassis.moveToPose(-10, -62, 95, 2000);
    //skidibi legitness!! :3
    // super sugoi sugoi sugoi!!!!
   */
    
	/*
    //BLUE POSITIVE SIDE **WORKING**
    chassis.setPose(55, -60, 270);
    chassis.moveToPoint(25, -60, 1500);
    chassis.moveToPoint(13, -56, 1000);
    chassis.waitUntilDone();
    Doinker.toggle();
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(17, -59, 500, {.forwards = false});
    chassis.turnToHeading(345, 500);
    chassis.waitUntilDone();
    Doinker.toggle();
    pros::delay(100);
    chassis.moveToPoint(14, -46, 500);
    chassis.waitUntilDone();
    //chassis.turnToHeading(10, 500);
    chassis.moveToPoint(10, -57, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(205, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(11, -50, 500, {.forwards = false});
    chassis.moveToPoint(12, -45, 500, {.forwards = false, .maxSpeed = 40});
    chassis.waitUntilDone();
    Clamp.toggle();
    chassis.waitUntilDone();
    pros::delay(200);
    Intake.move_velocity(600);
    pros::delay(500);
    Intake.move_velocity(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(12, -40, 500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 500);
    chassis.waitUntilDone();
    pros::delay(200);
    Intake.move_velocity(600);
    chassis.moveToPoint(19, -49, 500, {.maxSpeed = 60});
    pros::delay(1100);
    Intake.move_velocity(0);
    chassis.turnToHeading(300, 500);
    chassis.moveToPoint(170, -50, 700, {.forwards = false});
    Clamp.toggle();
    chassis.waitUntilDone();
    chassis.moveToPoint(30, -40, 500);
    chassis.turnToHeading(193, 500);
    chassis.moveToPoint(30, -5, 700, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(200);
    Clamp.toggle();
    chassis.waitUntilDone();
    pros::delay(200);
    Intake.move_velocity(600);
    pros::delay(300);
    chassis.turnToHeading(270, 500);
    */

   
   chassis.setPose(-63, 0, 90);
   Intake.move_velocity(200);
   pros::delay(500);
   Intake.move_velocity(0);
   chassis.moveToPoint(-50, 0, 1000);
   chassis.moveToPose(-52, -20, 0, 2000, {.forwards = false});
   chassis.waitUntilDone();
   pros::delay(100);
   Clamp.toggle();
   pros::delay(300);
   chassis.moveToPoint(-24, -24, 1000);
   Intake.move_velocity(600);
   chassis.moveToPose(0, -62, 135, 2000, {.maxSpeed = 100});
   chassis.waitUntilDone();
   pros::delay(100);
   Intake.move_velocity(-600);
   pros::delay(100);
   Intake.move_velocity(600);
   chassis.moveToPoint(-10, -54, 1000);
   chassis.turnToPoint(24, -45, 500);
   chassis.moveToPoint(24, -45, 1000);
   chassis.waitUntilDone();
   nextState();
   pros::delay(100);
   Intake.move_velocity(-600);
   pros::delay(100);
   Intake.move_velocity(600);
   chassis.moveToPoint(-5, -47, 1000, {.forwards = false});

   // At Wall Stake
   chassis.moveToPose(-5, -65, 180, 2000);
   chassis.waitUntilDone();
   chassis.setPose(0, -55, 180);
   Intake.move_velocity(0);
   nextState();
   pros::delay(500);
   nextState();
   Intake.move_velocity(600);
   chassis.moveToPoint(0, -50, 1000, {.forwards = false});
   chassis.moveToPose(-60, -50, 260, 3000, {.maxSpeed = 100});

   /*
   chassis.turnToPoint(-47, -59, 500);
   chassis.moveToPose(-47, -65, 180, 1000);
   chassis.turnToHeading(45, 500);
   chassis.moveToPose(-57, -57, 45, 500, {.forwards = false});
   chassis.waitUntilDone();
   Intake.move_velocity(0);
   Clamp.toggle();
   pros::delay(200);
   chassis.moveToPoint(-47, -47, 1000);
   chassis.moveToPose(-52, 30, 180, 3000, {.forwards = false});
   chassis.waitUntilDone();
   pros::delay(300);
   Clamp.toggle();
   */
}
//erm whast in the ksibidi is going on!!

/**
 * Runs in driver control
 */
void opcontrol() {
    bool IntakeOn = false;
    bool Clamped = false;
    bool DoinkerDown = false;
    bool WrongRing;

    while (true) {
        pros::lcd::print(0, "X: %f", rotationSensor.get_position()); // x
        // get joystick positions
        int leftY = Controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

        if (Controller.get_digital_new_press(DIGITAL_R1)) {
            if (IntakeOn == false) {
                Intake.move_velocity(600);
                IntakeOn = true;
            }
            else if (IntakeOn == true) {
                Intake.move_velocity(0);
                IntakeOn = false;
            }
        }

        if (Controller.get_digital(DIGITAL_R2)) {
            Intake.move_velocity(-600);
        } else {
            if (IntakeOn == true) {
                Intake.move_velocity(600);
            }
            else if (IntakeOn == false) {
                Intake.move_velocity(0);
            }
        }

        if (Controller.get_digital_new_press(DIGITAL_L1)) {
			Clamp.toggle();
		}

         if (Controller.get_digital_new_press(DIGITAL_L2)) {
			Doinker.toggle();
		}

        if (Controller.get_digital_new_press(DIGITAL_RIGHT)) {
            IntakeLift.toggle();
        }

        if (Controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			nextState();
		}

        // delay to save resources
        pros::delay(10);
    }
}
