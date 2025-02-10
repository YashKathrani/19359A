#include "lemlib/api.hpp"  //test test
#include "lemlib/asset.hpp"
#include "pros/misc.h"
#include "main.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut clamp{'A'};
pros::adi::DigitalOut intakeLift{'D'};
pros::adi::DigitalOut doinkerArm{'B'};
pros::adi::DigitalOut doinkerClawDown{'E'};
pros::adi::DigitalOut doinkerClawOpen{'C'};
pros::Motor wallStakeMotor(6);
pros::Motor frontIntake(9);
pros::Motor conveyorLift(10);
pros::Imu inertialSensor(7);

// rotation sensors
pros::Rotation horizontal_encoder(-19);
pros::Rotation vertical_encoder(20);
pros::Rotation wallStakeRotation(8);

pros::Optical intakeOpticalSensor(11);
pros::Distance intakeDistanceSensor(12);

int currentStage = 0;  // Start at first stage
int wallStakePositions[] = {270, 222, 211, 115, 24, 170};
const int wallStakePosCount = sizeof(wallStakePositions) / sizeof(wallStakePositions[0]);
double wallStakePCoeff = 2;
double wallStakeDCoeff = 100;
int targetPosition;

bool clampStatus = false;
bool intakeLiftStatus = false;
bool doinkerStatus = false;

auto drive_gearset = pros::v5::MotorGears::ratio_6_to_1;

// motor groups
pros::MotorGroup left_motor_group({-5, 3, -4}, drive_gearset);
pros::MotorGroup right_motor_group({-21, 1, 2}, drive_gearset);

// tracking wheels
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0.875); //offset = distance from center plane in thier axis only CHECK
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.625); 

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              12.5,        // inch track width CHECK THIS 
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450,                        // drivetrain rpm is 450
                              2                           // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,   // vertical tracking wheel 1
                            nullptr,                    // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr,                    // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertialSensor             // inertial sensor
);

lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
                                              0.02, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.7, // proportional gain (kP)
                                              0.00127, // integral gain (kI)
                                              17.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

double err = 0;
double previousPosition;

void wallStakeLoop() {
    targetPosition = wallStakePositions[currentStage];
    // Get current position from the rotational sensor
    float angle_in_centidegrees = wallStakeRotation.get_angle();
    float angle_in_degrees = angle_in_centidegrees / 100.0;
    double current_position = angle_in_degrees;
    if (current_position > 340) current_position -= 360;
    double derivative;
    if (previousPosition != 0) {
        derivative = current_position - previousPosition;
    }
    // PID variables
    double error = current_position - targetPosition;
    double velocity = wallStakePCoeff * error + wallStakeDCoeff * derivative;
    // Apply motor power
    wallStakeMotor.move(velocity);

    err = error;
};

#pragma region quick_type

#define clamp_on clamp.set_value(true);
#define clamp_off clamp.set_value(false);

void runIntake(float direction) {
    frontIntake.move_velocity(200 * direction);
    conveyorLift.move_velocity(-200 * direction);
}

#define INTAKE_FRONT 1
#define INTAKE_CONVEYOR 2

void runIntakeSolo(int8_t direction, int8_t which) {
    if (which == INTAKE_FRONT) {
        frontIntake.move_velocity(600 * direction);
        conveyorLift.move_velocity(0);
    }
    else if (which == INTAKE_CONVEYOR) {
        frontIntake.move_velocity(0);
        conveyorLift.move_velocity(-600 * direction);
    }
}

#define intakeDown intakeLift.set_value(true);
#define intakeUp intakeLift.set_value(false);

auto dir_ccw = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
auto dir_cw = lemlib::AngularDirection::CW_CLOCKWISE;

#pragma endregion quick_type

bool opp_is_blue = true; //!<-- CHANGE ON UPLOAD
int oppHueRange[2];

const int8_t INTAKE_RUNNING=0, INTAKE_FLICK_WAIT_DISTANCE=1, INTAKE_FLICK_AFTER_DISTANCE=2, INTAKE_FLICK_BACKWARDS=3;
int8_t intakeColorSortState;
uint32_t intakeLastChangeTime;

bool intakeOppDetected() {
    return (intakeOpticalSensor.get_hue() > oppHueRange[0]) && (intakeOpticalSensor.get_hue() < oppHueRange[1]);
}

double distprt = 0;

void intakeColorSortLoop() {
    double hue = intakeOpticalSensor.get_hue();
    double brightness = intakeOpticalSensor.get_brightness();
    double distance = intakeDistanceSensor.get();

    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE, 0, "dist %f", distprt);

    switch (intakeColorSortState) {
        case INTAKE_RUNNING:
            runIntake(0.8);
            if (intakeOppDetected()) {
                intakeColorSortState = INTAKE_FLICK_WAIT_DISTANCE;
                intakeLastChangeTime = pros::millis();
            }
            break;
        case INTAKE_FLICK_WAIT_DISTANCE:
            runIntake(1);
            distprt = distance;
            if (distance < 130) { //! <-- distance at which the ring turns around the top
                intakeColorSortState = INTAKE_FLICK_AFTER_DISTANCE;
                intakeLastChangeTime = pros::millis();
            }
            break;
        case INTAKE_FLICK_AFTER_DISTANCE:
            runIntake(1);
            if (pros::millis() - intakeLastChangeTime >= 50) { //! <-- delay (ms) for waiting to start moving backwards
                intakeColorSortState = INTAKE_FLICK_BACKWARDS;
                intakeLastChangeTime = pros::millis();
            }
            break;
        case INTAKE_FLICK_BACKWARDS:
            runIntake(-0.5);
             if (pros::millis() - intakeLastChangeTime >= 300) { //! <-- amount of time (ms) it moves backwards for
                intakeColorSortState = INTAKE_RUNNING;
                intakeLastChangeTime = pros::millis();
            }
            break;
        default:
            runIntake(1);
            break;
    }
}

//-------------

void initialize() {
    if (opp_is_blue) {
        oppHueRange[0] = 200;
        oppHueRange[1] = 230;
    } else {
        oppHueRange[0] = 355;
        oppHueRange[1] = 40;
    }
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    wallStakeMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading;
            pros::lcd::print(3, "ldbrwn err %f", err); // wall stake error
            pros::lcd::print(4, "ring %s h %f", intakeOppDetected()?"opp":"---", intakeOpticalSensor.get_hue());
            pros::lcd::print(5, "Rotation Sensor Horizontal: %i", horizontal_encoder.get_position());
            pros::lcd::print(6, "Rotation Sensor Vertical: %i", vertical_encoder.get_position());

            pros::delay(1);
        }
    });
}

void disabled() {};

void competition_initialize() {};

void auton_init()
{
    // initiatlize ladybrown
    intakeDown;
};

void auton_pid_tuning_angular() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 100000000);
    chassis.turnToHeading(180, 100000000);
    chassis.turnToHeading(270, 100000000);
    chassis.turnToHeading(0, 100000000);

}

void auton_pid_tuning_lateral() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 10000);
}

void red_pos() {
    doinkerClawOpen.set_value(true);
    chassis.setPose(0,0,0);
    chassis.moveToPoint(1.5,34,920,{.forwards = true, .maxSpeed=127},true);
    doinkerArm.set_value(true);
    pros::delay(800);
    doinkerClawOpen.set_value(false);
    doinkerClawDown.set_value(true); //goalrush
    chassis.moveToPoint(5,18,500,{.forwards = false, .maxSpeed=127},true);
    pros::delay(600);
    doinkerArm.set_value(false);
    doinkerClawOpen.set_value(true);
    doinkerClawDown.set_value(false);
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(1.5, 40, 1000, {.forwards = false, .maxSpeed=127},true);
    pros::delay(600);  
    clamp.set_value(true);  //clamp open





    // chassis.turnToHeading(24, 10000);
};

void autonomous()
{
    
    auton_init();

    // auton_pid_tuning_angular();
    red_pos();
};

void drive_init() {
    clamp.set_value(false);
    doinkerArm.set_value(false);
    doinkerClawOpen.set_value(true);
    intakeLift.set_value(false);
    currentStage = 0;  // Start at first stage
}

void opcontrol()
{
    drive_init();
    // loop forever
    while (true)
    {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            clampStatus = !clampStatus;
            clamp.set_value(clampStatus);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            doinkerStatus = !doinkerStatus;
            doinkerArm.set_value(doinkerStatus);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            intakeLiftStatus = !intakeLiftStatus;
            intakeLift.set_value(intakeLiftStatus);
        }
        if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakeColorSortState = INTAKE_RUNNING;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            // intakeColorSortLoop();
            runIntake(1);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            runIntake(-1);
        } else {
            runIntake(0);
        };

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            currentStage = currentStage == 1 ? 2 : 1;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            currentStage = 0;  // Instantly reset to first stage
        }

         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            currentStage = 4;  // Instantly reset to first stage
        }

         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            currentStage = currentStage == 3 ? 5 : 3;
        }

        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        // delay to save resources
        wallStakeLoop();

        pros::delay(1);
    };
}