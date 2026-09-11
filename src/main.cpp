#include "main.h"

#include "evolib/api.hpp"
#include "evolib/timer.hpp"
pros::Motor rightFrontMotor(-17);
pros::Motor rightMiddleMotor(14);
pros::Motor rightBackMotor(15);
pros::Motor leftFrontMotor(5);
pros::Motor leftMiddleMotor(-13);
pros::Motor leftBackMotor(-12);
pros::MotorGroup leftDrive({5, -13, -12}, pros::v5::MotorGears::blue);
pros::MotorGroup rightDrive({-17, 14, 15}, pros::v5::MotorGears::blue);
pros::MotorGroup evodrive({5, -13, -12, -17, 14, 15},
                          pros::v5::MotorGears::blue);
pros::Motor intakeBottom(19, pros::v5::MotorGears::blue);
pros::Motor intakeTop(-20, pros::v5::MotorGears::blue);
pros::Distance leftDist(16);
pros::Distance forwardDist(7);
pros::Distance rightDist(9);
pros::Distance goalDist(8);
pros::Optical blocksensLeft(23);
pros::Optical blocksensRight(2);
pros::Optical blocksensIntake(3);
pros::Rotation verticalWheelRot(18);
pros::Rotation horizontalWheelRot(9);

pros::adi::Pneumatics wing('H', false, true);
pros::adi::Pneumatics topRoller('E', true, false);
pros::adi::Pneumatics pistake('F', true, true);
pros::adi::Pneumatics ml('D', false, false);
pros::adi::Pneumatics descore('G', false, false);

pros::Imu imu(21);
// down: 358 (intake), primed: 28.38 (right), scoring:  127.5 (down) no past 135
// doinker: b
pros::Controller controller(pros::E_CONTROLLER_MASTER);
/**
 * Odometry Configuration
 */

evolib::TrackingWheel verticalTrackingWheel(&verticalWheelRot, 0.375, 2);
evolib::TrackingWheel horizontalTrackingWheel(&horizontalWheelRot, 100, 2);
evolib::OdomSensors sensors(&verticalTrackingWheel, &horizontalTrackingWheel,
                            &imu);
evolib::Drivetrain drivetrain(&leftDrive, &rightDrive, 8);
// evolib::ControllerSettings linearController(
//     6,  // proportional gain (kP)
//     0,
//     15,  // derivative gain (kD)
//     3,
//     1,  // small error range, in inches
//     100,  // small error range timeout, in milliseconds
//     3,    // large error range, in inches
//     300,  // large error range timeout, in milliseconds
//     0     // maximum acceleration (slew)
// );
evolib::ControllerSettings linearController(
    10,  // proportional gain (kP)
    0,
    65,  // derivative gain (kD)
    3,
    0.5,  // 0.5small error range, in inches 1
    100,  // 100small error range timeout, in milliseconds
    1.5,  // 1.5large error range, in inches 3
    500,  // 500 large error range timeout, in milliseconds
    0     // maximum acceleration (slew)
);

// turning PID
evolib::ControllerSettings angularController(
    3.4,              // proportional gain (kP)4
    300 * (1 / 100),  // integral gain (kI    ) 400
    0.27 * (100),     // derivative gain (kD) 0.305
    5,
    1,    // 1small error range, in degrees
    80,   // 50small error range timeout, in milliseconds
    3,    // 3large error range, in degrees
    200,  // 200large error range timeout, in milliseconds
    0     // maximum acceleration (slew). 0 means no limit
);

evolib::Robot robot(drivetrain, linearController, angularController, sensors);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    new pros::Task{[=] {
        while (true) {
            controller.print(0, 0, "%.1f, %.1f, %.2f", robot.getPose().x,
                             robot.getPose().y, robot.getPose().theta);
            // pros::lcd::print(2, "%.2f", robot.getPose().theta);
            // pros::lcd::print(1, "%.2f,%.2f\n", robot.getPose().x,
            //                  robot.getPose().y);
            pros::lcd::print(3, "%.2f, %.2f", rightDist.get() / 25.4,
                             forwardDist.get() / 25.4);
            // printf("%.2f,%.2f\n", robot.getPose().x, robot.getPose().y);
            // printf("%.2f\n", imu.get_rotation());

            pros::delay(10);
        }
    }};
    pros::lcd::register_btn1_cb(on_center_button);
    robot.calibrate();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
// 6.68" portable
float getLeftDist(float theta) {
    return ((leftDist.get() / 25.4) + 5.77) * cos(theta) + 3.844 * sin(theta);
}
// 6.13"  portable
float getRightDist(float theta) {
    return ((rightDist.get() / 25.4) + 5.77) * cos(theta) - 3.844 * sin(theta);
}
// 6.5
// 7.46" portable
float getForwardDist(float theta) {
    return ((forwardDist.get() / 25.4) + 7.06) * cos(theta) +
           4.375 * sin(theta);
}

// 4.31" portable
float getBackwardDist(float theta) {
    return ((goalDist.get() / 25.4) + 4.08) * cos(theta) + 0.25 * sin(theta);
}
void waitUntilLongGoal() {
    evolib::Timer timeout(2000);
    while (goalDist.get() > 150) {
        if (timeout.isDone()) {
            break;
        }
        pros::delay(20);
    }
}
void autonomous() {
    float theta;
    wing.extend();
    intakeTop.move(127);
    intakeBottom.move(127);
    /**
     * START LOW GOAL
     */
    robot.setPose(0, 0, 180);
    pros::delay(0);
    robot.moveForDistance(43, 3000,
                          {.forwards = true, .maxSpeed = 70, .minSpeed = 0});
    robot.waitUntil(20);
    ml.extend();
    // robot.waitUntil(32);
    // ml.retract();
    robot.waitUntilDone();
    pros::delay(200);
    robot.setX(
        -(72 - getRightDist((robot.getPose().theta - 180) * M_PI / 180)));
    robot.setY(
        -(72 - getForwardDist((robot.getPose().theta - 180) * M_PI / 180)));
    pros::delay(200);
    robot.turnToPoint(0, -8, 1500,
                      {.forwards = true, .maxSpeed = 127, .minSpeed = 15});
    robot.waitUntilDone();
    theta = (robot.getPose().theta * M_PI / 180);
    theta = M_PI / 2 - theta;
    pros::delay(10);
    robot.setX(-(72 - getBackwardDist(theta)));
    robot.setY(-(72 - getRightDist(theta)));
    pros::delay(200);
    ml.retract();
    robot.boomerang(-24, -19, 60, 1500,
                    {.forwards = true, .maxSpeed = 80, .minSpeed = 10});
    robot.boomerang(
        -8, -10, 45, 1500,
        {.forwards = true, .lead = 0.7, .maxSpeed = 127, .minSpeed = 15});
    robot.waitUntilDone();
    ml.extend();
    evodrive.move(40);
    intakeTop.move(-127);
    intakeBottom.brake();
    pros::delay(200);
    pistake.retract();
    intakeTop.move(-1 * 127);
    intakeBottom.move(-0.9 * 127);
    pros::delay(400);
    intakeTop.move(-1 * 127);
    intakeBottom.move(-0.7 * 127);
    pros::delay(500);
    intakeBottom.move(-0.4 * 127);
    pros::delay(1800);
    ml.retract();
    robot.moveToPoint(-24, -24, 1500,
                      {.forwards = false, .maxSpeed = 127, .minSpeed = 0});
    robot.waitUntilDone();
    pistake.extend();
    wing.extend();
    intakeTop.move(1 * 127);
    intakeBottom.move(1 * 127);
    robot.moveToPoint2(-24, 18, 3000,
                       {.forwards = true,
                        .maxSpeed = 0.6 * 127,
                        .minSpeed = 15,
                        .earlyExitRange = 3});
    robot.turnToHeading(-45, 1500,
                        {.maxSpeed = 127, .minSpeed = 30, .earlyExitRange = 3});
    robot.waitUntil(5);
    ml.extend();
    robot.moveToPoint2(-44, 42, 3000,
                       {.forwards = true,
                        .maxSpeed = 80,
                        .minSpeed = 15,
                        .earlyExitRange = 3});
    robot.turnToHeading(-90, 1500, {.maxSpeed = 127, .minSpeed = 0});
    // robot.moveToPoint2(-100, 42, 2000,
    //                    {.forwards = true,
    //                     .maxSpeed = 40,
    //                     .minSpeed = 15,
    //                     .earlyExitRange = 0});
    robot.waitUntilDone();
    robot.moveForDistance(20, 2300,
                          {.forwards = true, .maxSpeed = 40, .minSpeed = 30});
    // pros::delay(1800);
    robot.waitUntilDone();
    robot.setX(
        -(72 - getForwardDist((robot.getPose().theta + 90) * M_PI / 180)));
    robot.setY(72 - getRightDist((robot.getPose().theta + 90) * M_PI / 180));
    pros::delay(200);
    /**
     * END START LOW GOAL
     */
    /**
     * ML 1 + LONG GOAL 1
     */
    robot.moveToPoint2(-30, 58, 3000,
                       {.forwards = false,
                        .maxSpeed = 127,
                        .minSpeed = 60,
                        .earlyExitRange = 3});
    robot.moveToPoint2(27, 57, 6000,
                       {.forwards = false,
                        .maxSpeed = 127,
                        .minSpeed = 60,
                        .earlyExitRange = 3});
    robot.moveToPoint2(40, 48, 4000,
                       {.forwards = false,
                        .maxSpeed = 127,
                        .minSpeed = 40,
                        .earlyExitRange = 3});
    robot.moveToPoint2(-48, 44, 4000,
                       {.forwards = false, .maxSpeed = 127, .minSpeed = 30});
    waitUntilLongGoal();
    intakeTop.move(-127);
    intakeBottom.move(-127);
    pros::delay(300);
    intakeTop.move(127);
    intakeBottom.move(127);
    // scoring.retract();
    wing.retract();
    pros::delay(1200);
    robot.setX(29);
    robot.setY(48);
    pros::delay(300);
    // robot.cancelMotion();
    // pros::delay(100);

    robot.moveForDistance(22, 1000,
                          {.forwards = true, .maxSpeed = 127, .minSpeed = 60});
    robot.waitUntil(10);
    wing.extend();
    robot.waitUntilDone();
    robot.moveForDistance(20, 2000,
                          {.forwards = true, .maxSpeed = 40, .minSpeed = 30});
    // pros::delay(2000);
    robot.moveToPoint2(-48, 48, 3000,
                       {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
    robot.waitUntil(4);
    evolib::Timer timeout(2000);
    while (goalDist.get() > 500) {
        if (timeout.isDone()) {
            break;
        }
        pros::delay(20);
    }
    intakeTop.brake();
    wing.retract();
    waitUntilLongGoal();
    intakeTop.move(127);
    // scoring.retract();
    pros::delay(900);
    robot.setX(29);
    robot.setY(48);
    pros::delay(100);
    ml.retract();
    /**
     * END ML 1 + LONG GOAL 1
     */
    /**
     * TOP MIDDLE GOAL FROM LONG GOAL 1
     */
    robot.boomerang(64, 16, 180, 1600, {.lead = 0.5});
    robot.waitUntil(30);
    wing.extend();
    robot.moveForDistance(37.5, 3000,
                          {.forwards = true, .maxSpeed = 70, .minSpeed = 0});
    robot.waitUntil(15);
    ml.extend();
    robot.turnToHeading(280, 1500, {.maxSpeed = 127, .minSpeed = 0});
    robot.waitUntilDone();
    ml.retract();
    pros::delay(100);
    theta = (robot.getPose().theta * M_PI / 180);
    theta -= 3 * M_PI / 2;
    robot.setX(72 - getBackwardDist(theta));
    robot.setY(-(72 - getLeftDist(theta)));
    robot.moveToPoint(36, -13.5, 1500,
                      {.forwards = true, .maxSpeed = 127, .minSpeed = 10});
    robot.turnToPoint(24, -25.5, 1500, {.maxSpeed = 127, .minSpeed = 0});
    robot.moveToPoint(24, -25.5, 1500, {.forwards = true, .maxSpeed = 50});
    robot.waitUntilDone();
    // pros::delay(2000);
    // robot.waitUntil(5);
    ml.extend();
    robot.turnToPoint(0, -1.5, 1500,
                      {.forwards = false,
                       .maxSpeed = 127,
                       .minSpeed = 50,
                       .earlyExitRange = 5});
    // robot.moveForDistance(23, 1500,
    //                       {.forwards = false, .maxSpeed = 90, .minSpeed =
    //                       30});
    // robot.boomerang(10, -10, 135, 1500,
    //                 {.forwards = false, .lead = 0.4, .maxSpeed = 127,
    //                 .minSpeed = 60});
    robot.moveToPoint(10, -11.5, 1500,
                      {.forwards = false, .maxSpeed = 127, .minSpeed = 80});
    // robot.waitUntilDone();
    // descore.extend();
    // robot.moveForDistance(4, 1000,
    //                       {.forwards = true, .maxSpeed = 127, .minSpeed =
    //                       30});
    // robot.waitUntilDone();
    // descore.retract();
    robot.moveForDistance(10, 1000,
                          {.forwards = false, .maxSpeed = 127, .minSpeed = 50});
    robot.waitUntilDone();
    evodrive.move(-10);
    // new pros::Task([] {
    //     while (intakeColor != Color::BLUE) {
    //         pros::delay(10);
    //     }
    //     pros::delay(300);
    //     intakeBottom.move(-0.5 * 127);
    //     pros::delay(600);
    //     intakeBottom.move(0);
    // });
    intakeTop.move(-127);
    pros::delay(400);
    topRoller.retract();
    // pros::delay(300);
    intakeTop.move(1 * 127);
    intakeBottom.move(1 * 127);
    // pros::delay(500);
    intakeTop.move(0.5 * 127);
    intakeBottom.move(1 * 127);
    pros::delay(4000);
    topRoller.extend();
    pros::delay(200);
    intakeTop.move(127);
    robot.moveToPoint(48, -47, 1500,
                      {.forwards = true, .maxSpeed = 127, .minSpeed = 10});
    robot.turnToHeading(86.5, 1500, {.maxSpeed = 127, .minSpeed = 0});

    // robot.moveToPoint2(100, -47, 2000,
    //                    {.forwards = true,
    //                     .maxSpeed = 40,
    //                     .minSpeed = 15,
    //                     .earlyExitRange = 0});

    /**
     * END TOP MIDDLE GOAL FROM LONG GOAL 1
     */
    /**
     * START ML 2 + LONG GOAL 2
     */
    // robot.waitUntil(15);
    // wing.retract();
    // pros::delay(100);
    // wing.extend();
    // robot.turnToHeading(-95, 1500,
    //                     {.maxSpeed = 127, .minSpeed = 0, .earlyExitRange =
    //                     3});
    // robot.waitUntilDone();
    // evodrive.move(50);
    // pros::delay(800);
    robot.moveForDistance(20, 1700,
                          {.forwards = true, .maxSpeed = 60, .minSpeed = 30});
    robot.setX(
        (72 - getForwardDist((robot.getPose().theta - 90) * M_PI / 180)));
    robot.setY(-(72 - getRightDist((robot.getPose().theta - 90) * M_PI / 180)));
    pros::delay(200);
    robot.moveToPoint2(34, -58, 3000,
                       {.forwards = false,
                        .maxSpeed = 127,
                        .minSpeed = 60,
                        .earlyExitRange = 3});
    robot.moveToPoint2(-27, -59, 6000,
                       {.forwards = false,
                        .maxSpeed = 127,
                        .minSpeed = 60,
                        .earlyExitRange = 3});
    robot.moveToPoint2(-40, -48, 4000,
                       {.forwards = false,
                        .maxSpeed = 127,
                        .minSpeed = 40,
                        .earlyExitRange = 3});
    robot.moveToPoint2(48, -44, 4000,
                       {.forwards = false, .maxSpeed = 127, .minSpeed = 30});
    waitUntilLongGoal();
    intakeTop.move(-127);
    intakeBottom.move(-127);
    pros::delay(300);
    intakeTop.move(127);
    intakeBottom.move(127);
    // scoring.retract();
    wing.retract();
    pros::delay(1000);
    robot.setX(-29);
    robot.setY(-48);
    pros::delay(1000);
    // robot.cancelMotion();
    // pros::delay(100);

    robot.moveForDistance(22, 1000,
                          {.forwards = true, .maxSpeed = 127, .minSpeed = 30});
    robot.waitUntil(10);
    wing.extend();
    robot.waitUntilDone();
    // evodrive.move(50);
    robot.moveForDistance(20, 2000,
                          {.forwards = true, .maxSpeed = 40, .minSpeed = 30});
    // pros::delay(2000);
    robot.moveToPoint2(48, -48, 3000,
                       {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
    waitUntilLongGoal();
    wing.retract();
    pros::delay(1500);
    ml.retract();
    robot.boomerang(-69, 0, 0, 1800, {.lead = 0.5});
    /**
     * END START LOW GOAL
     */
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::MotorGroup left_mg(
        {1, -2, 3});  // Creates a motor group with forwards ports 1 & 3 and
                      // reversed port 2
    pros::MotorGroup right_mg(
        {-4, 5, -6});  // Creates a motor group with forwards port 5 and
                       // reversed ports 4 & 6
    intakeBottom.move(-127);
    intakeTop.move(-127);
    while (true) {
        pros::lcd::print(0, "%d %d %d",
                         (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
                             0);  // Prints status of the emulated screen LCDs

        // Arcade control scheme
        int dir = master.get_analog(
            ANALOG_LEFT_Y);  // Gets amount forward/backward from left joystick
        int turn = master.get_analog(
            ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
        left_mg.move(dir - turn);   // Sets left motor voltage
        right_mg.move(dir + turn);  // Sets right motor voltage
        pros::delay(20);            // Run for 20 ms then update
    }
}