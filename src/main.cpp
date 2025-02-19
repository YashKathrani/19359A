#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "main.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut clamp{'A'};
pros::adi::DigitalOut intakeLift{'D'};
pros::adi::DigitalOut doinkerArm{'B'};
pros::adi::DigitalOut doinkerClawClose{'E'};
pros::adi::DigitalOut doinkerClawOpen{'C'};
pros::Motor wallStakeMotor(6);
pros::Motor frontIntake(9);
pros::Motor conveyorLift(10);
pros::Imu inertialSensor(7);

// rotation sensors
pros::Rotation horizontal_encoder(19);
pros::Rotation vertical_encoder(20);
pros::Rotation wallStakeRotation(8);

pros::Optical intakeOpticalSensor(11);
pros::Distance intakeDistanceSensor(12);
pros::Distance autonResetDistanceSensor(17);

int wallStakeCurrentStage = 0; // Start at first stage
int wallStakePositions[] = {250, 213, 200, 85, 24, 170};
const int wallStakePosCount = sizeof(wallStakePositions) / sizeof(wallStakePositions[0]);
double wallStakePCoeff = 2.6; //if it undershot increase
double wallStakeDCoeff = 100; //if it occolates chnage
int wallStakeTargetPosition;

bool clampStatus = false;
bool intakeLiftStatus = false;
bool doinkerArmStatus = false;
bool doinkerClawStatus = false;

auto drive_gearset = pros::v5::MotorGears::ratio_6_to_1;

// motor groups
pros::MotorGroup left_motor_group({-5, 3, -4}, drive_gearset);
pros::MotorGroup right_motor_group({-21, 1, 2}, drive_gearset);

// tracking wheels
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 1.5);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              12.5,                       // inch track width
                              lemlib::Omniwheel::NEW_325, // omniwheel size
                              450,                        // drivetrain rpm is 450
                              2                           // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,   // vertical tracking wheel 1
                            nullptr,                    // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr,                    // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertialSensor             // inertial sensor
);

lemlib::ControllerSettings lateral_controller(6,    // proportional gain (kP)
                                              0,    // integral gain (kI)
                                              28.5, // derivative gain (kD)
                                              3,    // anti windup
                                              1,    // small error range, in inches
                                              100,  // small error range timeout, in milliseconds
                                              3,    // large error range, in inches
                                              500,  // large error range timeout, in milliseconds
                                              20    // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.77,             // proportional gain (kP)
                                              0,                // integral gain (kI)
                                              17.5,             // derivative gain (kD)
                                              2.71166666666667, // anti windup
                                              1,                // small error range, in inches
                                              100,              // small error range timeout, in milliseconds
                                              3,                // large error range, in inches
                                              500,              // large error range timeout, in milliseconds
                                              0                 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

double err = 0;
double previousPosition = 0;

void wallStakeLoop(float speedMultiplier = 1)
{
    wallStakeTargetPosition = wallStakePositions[wallStakeCurrentStage];
    // Get current position from the rotational sensor
    float angle_in_centidegrees = wallStakeRotation.get_angle();
    float angle_in_degrees = angle_in_centidegrees / 100.0;
    double current_position = angle_in_degrees;
    if (current_position > 340)
        current_position -= 360;
    double derivative;
    if (previousPosition != 0)
    {
        derivative = current_position - previousPosition;
    }
    // PID variables
    double error = current_position - wallStakeTargetPosition;
    double velocity = wallStakePCoeff * error + wallStakeDCoeff * derivative;
    // Apply motor power
    wallStakeMotor.move(velocity * speedMultiplier);

    err = error;
};

#pragma region quick_type

void doinkerClawSet(bool set)
{
    doinkerClawClose.set_value(set);
    doinkerClawOpen.set_value(!set);
}

#define clamp_on clamp.set_value(true);
#define clamp_off clamp.set_value(false);
#define doinker_down doinkerArm.set_value(true);
#define doinker_up doinkerArm.set_value(false);
#define doinker_open doinkerClawSet(false);
#define doinker_close doinkerClawSet(true);

void runIntake(float direction)
{
    frontIntake.move_velocity(200 * direction);
    conveyorLift.move_velocity(-200 * direction);
}

#define INTAKE_FRONT 1
#define INTAKE_CONVEYOR 2

void runIntakeSolo(float direction, int8_t which)
{
    if (which == INTAKE_FRONT)
    {
        frontIntake.move_velocity(600 * direction);
        conveyorLift.move_velocity(0);
    }
    else if (which == INTAKE_CONVEYOR)
    {
        frontIntake.move_velocity(0);
        conveyorLift.move_velocity(-600 * direction);
    }
}

#define intakeDown intakeLift.set_value(false);
#define intakeUp intakeLift.set_value(true);

auto dir_ccw = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
auto dir_cw = lemlib::AngularDirection::CW_CLOCKWISE;

void ladybrownSetWait(int cycle = 0, uint32_t timeout = 3000, bool reset_velocity = true, uint32_t loopWaitTime = 1, float speedMultiplier = 1)
{
    wallStakeCurrentStage = cycle;
    uint32_t start = pros::millis();
    while ((pros::millis() - start) < timeout)
    {
        wallStakeLoop(speedMultiplier);
        pros::delay(loopWaitTime);
    }
    if (reset_velocity)
    {
        wallStakeMotor.move(0);
    }
}

void resetAutonPositionWithDistance(bool x, float offsetFromCenter=5.875) {
    lemlib::Pose pose = chassis.getPose();
    float dist = autonResetDistanceSensor.get() / 25.4;
    float offset = dist + offsetFromCenter;
    if (x) {
        chassis.setPose(pose.x < 0 ? (-72 + offset) : (72 - offset), pose.y, pose.theta);
    }
    else {
        chassis.setPose(pose.x, pose.y < 0 ? (-72 + offset) : (72 - offset), pose.theta);
    }
}

#pragma endregion quick_type

bool opp_is_blue = true; //!<-- CHANGE ON UPLOAD
int oppHueRange[2];

const int8_t INTAKE_RUNNING = 0, INTAKE_FLICK_WAIT_DISTANCE = 1, INTAKE_FLICK_AFTER_DISTANCE = 2, INTAKE_FLICK_BACKWARDS = 3;
int8_t intakeColorSortState;
uint32_t intakeLastChangeTime;

bool intakeOppDetected()
{
    return (intakeOpticalSensor.get_hue() > oppHueRange[0]) && (intakeOpticalSensor.get_hue() < oppHueRange[1]);
}

double distprt = 0;

void intakeColorSortLoop()
{
    double hue = intakeOpticalSensor.get_hue();
    double brightness = intakeOpticalSensor.get_brightness();
    double distance = intakeDistanceSensor.get();

    pros::screen::print(pros::text_format_e_t::E_TEXT_LARGE, 0, "dist %f", distprt);

    switch (intakeColorSortState)
    {
    case INTAKE_RUNNING:
        runIntake(0.8);
        if (intakeOppDetected())
        {
            intakeColorSortState = INTAKE_FLICK_WAIT_DISTANCE;
            intakeLastChangeTime = pros::millis();
        }
        break;
    case INTAKE_FLICK_WAIT_DISTANCE:
        runIntake(1);
        distprt = distance;
        if (distance < 130)
        { //* <-- distance at which the ring turns around the top
            intakeColorSortState = INTAKE_FLICK_AFTER_DISTANCE;
            intakeLastChangeTime = pros::millis();
        }
        break;
    case INTAKE_FLICK_AFTER_DISTANCE:
        runIntake(1);
        if (pros::millis() - intakeLastChangeTime >= 50)
        { //* <-- delay (ms) for waiting to start moving backwards
            intakeColorSortState = INTAKE_FLICK_BACKWARDS;
            intakeLastChangeTime = pros::millis();
        }
        break;
    case INTAKE_FLICK_BACKWARDS:
        runIntake(-0.5);
        if (pros::millis() - intakeLastChangeTime >= 300)
        { //* <-- amount of time (ms) it moves backwards for
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

void initialize()
{
    if (opp_is_blue)
    {
        oppHueRange[0] = 200;
        oppHueRange[1] = 230;
    }
    else
    {
        oppHueRange[0] = 355;
        oppHueRange[1] = 40;
    }
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();     // calibrate sensors
    wallStakeMotor.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    // print position to brain screen
    pros::Task screen_task([&]()
                           {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading;
            pros::lcd::print(4, "Hor Enc: %i", horizontal_encoder.get_position());
            pros::lcd::print(5, "Ver Enc: %i", vertical_encoder.get_position());
            // pros::lcd::print(3, "ldbrwn err %f", err); // wall stake error
            // pros::lcd::print(4, "ring %s h %f", intakeOppDetected()?"opp":"---", intakeOpticalSensor.get_hue());
            // pros::lcd::print(5, "Rotation Sensor Horizontal: %i", horizontal_encoder.get_position());
            // pros::lcd::print(6, "Rotation Sensor Vertical: %i", vertical_encoder.get_position());

            pros::delay(20);
        } });
}

void disabled() {};

void competition_initialize() {};

void auton_init()
{
    // initiatlize ladybrown
    intakeDown;
}

void auton_pid_tuning_lateral()
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 10000);
}

void red_pos()
{
    runIntakeSolo(1,INTAKE_FRONT);
    doinkerClawOpen.set_value(true);                                              // open doinker claw at start
    chassis.setPose(0, 0, 0);                                                     // reset odometry
    chassis.moveToPoint(0, 38, 1000, {.forwards = true, .maxSpeed = 127}, true); // move to goal rush
    doinker_down;
    pros::delay(900);
    doinker_close;
    chassis.moveToPoint(3, 10, 900, {.forwards = false, .maxSpeed = 127}, true); // move back with mogo
    pros::delay(1000);
    doinker_open;
    chassis.moveToPoint(1, 7, 500, {.forwards = false, .maxSpeed = 127}, true); // move back after mogo
    pros::delay(400);
    doinker_up;
    chassis.turnToHeading(202.1, 400,{.direction = dir_cw});    // 180 turn   
    chassis.moveToPoint(14, 32.5, 950, {.forwards = false, .maxSpeed = 100}, true); // move to mogo 1
    pros::delay(1050);
    clamp_on;
    ladybrownSetWait(5, 500); //queue ladybrown
    runIntake(1);
    pros::delay(900);
    chassis.turnToHeading(31, 800,{.direction = dir_ccw});  //180 turn
    pros::delay(500);
    runIntake(0);
    clamp_off; //drop mogo 1
    chassis.turnToHeading(35, 300,{.direction = dir_cw});  //turn toward wallstake
    chassis.moveToPose(21, 40,80, 1000, {.forwards = true, .maxSpeed = 100}, false); //align for wallstake mech
    wallStakeMotor.move_velocity(200); //ladybrown score
    pros::delay(700);
    wallStakeMotor.move_velocity(-150); //ladybrown back
    chassis.moveToPoint(-28, 37.3, 1600, {.forwards = false, .maxSpeed = 127}, false); //move to mogo 2
    wallStakeMotor.move_velocity(0);
    pros::delay(200);
    clamp_on; //clamp mogo 2
    chassis.moveToPoint(-2, -13, 2500, {.forwards = true, .maxSpeed = 127}, false); //go to corner ring
    runIntake(1); //intake corner ring
    wallStakeMotor.move_velocity(0); 
    
};

// part stuff for testing
#define part1 true
#define part2 true
#define part3 true

void auton_skills()
{
    if (part1)
    {
        chassis.setPose(-63.25, 0, 90);
        runIntake(1);
        pros::delay(400);
        runIntake(0);
        chassis.moveToPoint(-56, 0, 900);
        chassis.moveToPose(-48, -16, 0, 1400, {.forwards = false, .lead = 0});
        chassis.moveToPoint(-48, -21, 800, {.forwards = false}, false);
        clamp_on;
        runIntake(1);
        chassis.moveToPose(-24, -24, 135, 2000, {.lead = 0}); // score ring at -24, -24
        chassis.moveToPose(0, -42, 110, 2000, {.lead = 0.2}); // to ring
        chassis.moveToPose(30, -48, 90, 2000, {.lead = 0.4});   // to ring
        pros::delay(200);
        ladybrownSetWait(2, 2000);
        chassis.moveToPose(24, -48, 90, 700, {.forwards=false});   // to ring
        chassis.moveToPoint(24, -48, 2000);   // to ring
        // pros::delay(300);                                    // tweak timing so ring at 24, -48 is in lb
        // turn to 90 heading and use a distance sensor to reset position
        chassis.turnToHeading(90, 800, {}, false);
        ladybrownSetWait(2, 800);
        resetAutonPositionWithDistance(false);
    }
    if (part2)
    {
        if (!part1)
        { // 1 is scored, 1 is being put into ladybrown grip
            chassis.setPose(24, -48, 90);
            clamp_on;
            ladybrownSetWait(2, 2000);
            chassis.turnToHeading(97, 1000);
            runIntake(1);
        }
        chassis.moveToPose(0, -42, 110, 2000, {.forwards = false, .lead = 0}); // back to middle
        ladybrownSetWait(2, 2000);
        chassis.turnToHeading(180, 1000);
        runIntake(-0.1);
        pros::delay(200);
        ladybrownSetWait(2, 500);
        runIntake(0);
        chassis.moveToPoint(0, -66, 1000, {.maxSpeed = 30}, false); // move to score ladybrown position
        ladybrownSetWait(5, 500);
        runIntake(1);                                          // while intaking the ring at 0, -60
        ladybrownSetWait(3, 1500);
        ladybrownSetWait(4, 1000);
        chassis.moveToPose(0, -48, 180, 2000, {.forwards = false, .lead = 0}); // back up for bottom left rings
        ladybrownSetWait(0, 1000);
    }
    if (part3)
    {
        if (!part2)
        {
            chassis.setPose(0, -48, 180);
            clamp_on;
            runIntake(-0.5);
        }
        chassis.turnToHeading(-90, 800);
        chassis.moveToPose(-48, -48, -90, 4000, {.lead=0, .maxSpeed=90});
        runIntake(1);
        chassis.moveToPose(-60, -48, -90, 3000, {.lead=0});
        chassis.moveToPose(-48, -60, 135, 1000, {.lead=0, .maxSpeed=90});
        chassis.turnToPoint(-48, -60, 1000);
        chassis.moveToPoint(-48, -60, 1000);
        chassis.moveToPoint(-54, -54, 1000, {.forwards = false});
        chassis.turnToPoint(0, 0, 1000);
        chassis.moveToPose(-60, -60, 45, 1000, {.forwards = false}, false);
        clamp_off;
        pros::delay(200);
        runIntake(-0.5);
        chassis.moveToPose(-48, -48, 45, 1000, {}, false);
        runIntake(0);
    }
}

void auton_curve_test()
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(-24, 48, -90, 4000, {.lead = 0});
}

void auton_tuning_angular()
{
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 10000);
}

void auton_tuning_angular_steady_state_error()
{
    for (float i = 100; i <= 180; i += 10)
    {
        chassis.setPose(0, 0, 0);
        chassis.turnToHeading(i, 2000, {}, false);
        pros::delay(5000);
    }
}

void auton_tuning_lateral()
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 48, 100000);
}

void auton_ladybrown_move_test() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 48, 0, 2000, {});
    chassis.moveToPose(0, 96, 0, 2000, {});
    ladybrownSetWait(2, 2000);
    chassis.moveToPose(0, 48, 0, 2000, {.forwards=false});
    ladybrownSetWait(5, 2000);
    chassis.moveToPose(0, 0, 0, 2000, {.forwards=false});
    ladybrownSetWait(2, 2000);
    ladybrownSetWait(5, 2000);
}

void autonomous()
{
    auton_init();

    // auton_tuning_angular();
    // auton_tuning_angular_steady_state_error();
    // auton_tuning_lateral();
    // auton_curve_test();
    // auton_ladybrown_move_test();
    red_pos();
    // auton_skills();

};

void drive_init()
{
    clamp.set_value(false);
    doinkerArm.set_value(false);
    doinkerClawOpen.set_value(true);
    intakeLift.set_value(false);
    wallStakeCurrentStage = 0; // Start at first stage
}

void opcontrol()
{
    drive_init();
    // loop forever
    while (true)
    {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        {
            clampStatus = !clampStatus;
            clamp.set_value(clampStatus);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        {
            doinkerArmStatus = !doinkerArmStatus;
            doinkerArm.set_value(doinkerArmStatus);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            intakeLiftStatus = !intakeLiftStatus;
            intakeLift.set_value(intakeLiftStatus);
        }
        if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intakeColorSortState = INTAKE_RUNNING;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            // intakeColorSortLoop();
            runIntake(1);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            runIntake(-1);
        }
        else
        {
            runIntake(0);
        };

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            runIntakeSolo(-0.1,INTAKE_CONVEYOR);
            pros::delay(300);
            runIntakeSolo(0,INTAKE_CONVEYOR);
            wallStakeCurrentStage = wallStakeCurrentStage == 1 ? 2 : 1;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            wallStakeCurrentStage = 0; // Instantly reset to first stage
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            wallStakeCurrentStage = 4; // Instantly reset to first stage
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
        
        {
            runIntakeSolo(-0.1,INTAKE_CONVEYOR);
            pros::delay(300);
            runIntakeSolo(0,INTAKE_CONVEYOR);
            wallStakeCurrentStage = wallStakeCurrentStage == 3 ? 5 : 3;
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