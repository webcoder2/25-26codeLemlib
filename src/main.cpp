#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include <sys/_intsup.h>
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

int autonNum = 0;
int intakeRun = 0;
bool outtake = false;
bool pivotVar = false;
bool loaderVar = false;
pros::MotorGroup left_motors({-7, 8, -9}, pros::MotorGearset::blue); // Left motors on ports 20, 3, 5
pros::MotorGroup right_motors({1, -2, 3}, pros::MotorGearset::blue); // Right motors on ports 13, 16, 17
pros::Imu imu(11);
pros::Motor lowerMotor(-15);
pros::v5::Distance back(12);
//pros::Motor upperMotor(-19);
//pros::MotorGroup intake({lowerMotor,upperMotor});
pros::Motor endIntake(18);
pros::Rotation horizontal_sensor(8);
pros::Rotation vert_sensor(10);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut descore('H');
pros::adi::DigitalOut loader('A');
pros::adi::DigitalOut outTake('G');
lemlib::Drivetrain drivetrain(&left_motors, // Left motor group
                              &right_motors, // Right motor group
                              12.25, // 12.675 inch track width
                              lemlib::Omniwheel::NEW_325, // Using new 3.25" omnis
                              450, // Drivetrain RPM is 450
                              2 // Horizontal drift is 2 (for now)
);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, -2.65);
lemlib::TrackingWheel vert_tracking_wheel(&vert_sensor, lemlib::Omniwheel::NEW_2, 0);
lemlib::OdomSensors sensors(&vert_tracking_wheel,
                            nullptr,
                            /*&horizontal_tracking_wheel*/nullptr, 
                            nullptr,
                            &imu);
// Lateral PID controller
lemlib::ControllerSettings lateral_controller(11, // Proportional gain (kP)
                                              .1, // Integral gain (kI)
                                              23, // Derivative gain (kD)
                                              1.5, // Anti windup 3
                                              0.25, // Small error range, in inches .25
                                              200, //200 Small error range timeout, in milliseconds
                                              .75, // 10 Large error range, in inches
                                              500, //500 Large error range timeout, in milliseconds
                                              100 // 110Maximum acceleration (slew)
);

// Angular PID controller
lemlib::ControllerSettings angular_controller(4, // Proportional gain (kP)
                                              0, //.23 Integral gain (kI)
                                              28, // Derivative gain (kD)
                                              0, //6 Anti windup
                                              .38, // Small error range, in degrees.225
                                              200, // 400Small error range timeout, in milliseconds
                                              .75, // .75Large error range, in degrees
                                              500, // 1000Large error range timeout, in milliseconds
                                              0 // Maximum acceleration (slew)
);
lemlib::ExpoDriveCurve throttle_curve(5, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(5, // joystick deadband out of 127
 10, // minimum output where drivetrain will move out of 127
 1.019 // expo curve gain
);	
lemlib::Chassis chassis(drivetrain, // Drivetrain settings
                        lateral_controller, // Lateral PID settings
                        angular_controller, // Angular PID settings
                        sensors, // Odometry sensors
                        &throttle_curve,
                        &steer_curve
);			
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate();
    loader.set_value(loaderVar);
    descore.set_value(pivotVar);
    outTake.set_value(outtake);
    pros::lcd::initialize();
    //endIntake.set_brake_mode(pros::MotorBrake::hold);
    pros:: Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Distance:  %d\n", back.get()); // heading
            pros::delay(20);
        }
    });
    pros:: Task InTaKETask([&]() {
        while(1){
            if(!pros::competition::is_autonomous())
            {
                if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
                {
                    intakeRun = 1;
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                    intakeRun = -1;

                }
                else{
                    intakeRun = 0;
                }
                lowerMotor.move_velocity(600*intakeRun);
                //upperMotor.move_velocity(600*intakeRun);
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                    endIntake.move_velocity(intakeRun*600);
                }
                else {
                    endIntake.move_velocity(0);
                }
                //endIntake.move_velocity(600*outtake);
            }
            pros::delay(20);

        }
    });
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
void autonomous() 
{
    //left Auto only loader
    if(autonNum == 1)
    {
        chassis.setPose(-45.5, 14,0);
        chassis.moveToPose(-45.5, 48, 0, 1500);
        chassis.turnToHeading(270, 1000);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        chassis.moveToPoint(-56.6, 48, 1000, {.minSpeed= 100,.earlyExitRange=4},false);
        lowerMotor.move_velocity(600);
        pros::delay(3000);
        lowerMotor.move_velocity(0);
        chassis.moveToPoint(-25, 48, 2500,{.forwards=false},false);
        lowerMotor.move_velocity(600);
        endIntake.move_velocity(600);
        pros::delay(2000);
        lowerMotor.move_velocity(0);
    }
    //Right auto only loader
    else if (autonNum == 2)
    {
        chassis.setPose(-45.5, -14,180);
        chassis.moveToPose(-45.5, -48, 180, 1500);
        chassis.turnToHeading(270, 1000);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        chassis.moveToPoint(-56.6, -48, 1000, {.minSpeed= 100,.earlyExitRange=4},false);
        lowerMotor.move_velocity(600);
        pros::delay(3000);
        lowerMotor.move_velocity(0);
        chassis.moveToPoint(-25, -48, 2500,{.forwards=false},false);
        lowerMotor.move_velocity(600);
        endIntake.move_velocity(600);
        pros::delay(2000);
        lowerMotor.move_velocity(0);
    }
    //Skills AUTO 
    else if(autonNum ==0){
        chassis.setPose(-45.5, -14,180);
        chassis.moveToPose(-45.5, -48, 180, 1500);
        chassis.turnToHeading(270, 1000);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        lowerMotor.move_velocity(600);
        chassis.moveToPoint(-59, -48, 1250, {},false);
        //lowerMotor.move_velocity(600);
        /*chassis.turnToHeading(260, 300);
        chassis.turnToHeading(280, 300);
        chassis.turnToHeading(260, 300);
        chassis.turnToHeading(280, 300);
        chassis.turnToHeading(260, 300);
        chassis.turnToHeading(280, 300);
        chassis.turnToHeading(260, 300);
        chassis.turnToHeading(280, 300);
        chassis.turnToHeading(270, 300);*/
        pros::delay(3000);
        //lowerMotor.move_velocity(0);
        chassis.moveToPoint(-42, -48, 1000,{.forwards=false},false);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        chassis.turnToHeading(135, 1000);
        chassis.moveToPoint(-29, -62, 1000);
        chassis.turnToHeading(90,1000);
        chassis.moveToPoint(45, -62,3500, {.maxSpeed=80});
        chassis.turnToHeading(0, 1000);
        chassis.moveToPoint(45, -48, 1000);
        chassis.setPose(45,-(67.5-(back.get()/25.4)),0);
        chassis.turnToHeading(90, 1000);
        chassis.moveToPoint(25, -48, 1500,{.forwards=false},false);
        lowerMotor.move_velocity(600);
        endIntake.move_velocity(600);
        pros::delay(3000);
        
        lowerMotor.move_velocity(0);
        endIntake.move_velocity(0);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        chassis.moveToPoint(58, -48, 1500,{},false);
        lowerMotor.move_velocity(600);
        pros::delay(3000);
        lowerMotor.move_velocity(0);
        chassis.moveToPoint(25, -48, 1500,{.forwards=false},false);
        lowerMotor.move_velocity(600);
        endIntake.move_velocity(600);
        pros::delay(3000);
        
        lowerMotor.move_velocity(0);
        chassis.moveToPoint(45, -48, 1000);
        chassis.turnToHeading(0, 1000);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        chassis.moveToPoint(45, -34, 1000);
        chassis.turnToHeading(270,1000);
        chassis.moveToPose(-62,-34 , 270, 4000);
        chassis.turnToHeading(0, 1000);
        chassis.moveToPoint(-62,0 , 4000);

        

        /*chassis.setPose(-54,-17,90);
        lowerMotor.move_velocity(600);
        chassis.moveToPose(-40, -17, 90, 750,{.minSpeed = 90,.earlyExitRange=9});
        chassis.moveToPose(-24, -24, 135, 1250,{.lead=.001},false);
        pros::delay(500);
        lowerMotor.move_velocity(0);
        chassis.turnToHeading(45, 1000);
        chassis.moveToPose(-11.5, -13.5 , 45, 1500, {.minSpeed=50},false);
        lowerMotor.move_velocity(-600);
        upperMotor.move_velocity(-600);
        pros::delay(2000);
        chassis.moveToPose(-24, -24, 45, 1250,{.forwards=false},false);
        chassis.turnToHeading(225, 1000);
        lowerMotor.move_velocity(600);
        upperMotor.move_velocity(600);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        chassis.moveToPose(-63, -49, 270, 3250,{.lead=.62});
        //pros::delay(1000);
        chassis.moveToPose(-48, -48, 270, 1000, {.forwards=false});
        chassis.turnToHeading(90, 800);
        chassis.moveToPose(-30.75, -48, 90, 2500);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        pivotVar = !pivotVar; // toggle
        descore.set_value(pivotVar);
        outtake=!outtake;
        outTake.set_value(outtake);*/
    }
    //riht side with intaking balls
    else if (autonNum==2) {
        chassis.setPose(-48.5,-17,90);
        chassis.moveToPoint(-34, -17, 750);
        chassis.turnToHeading(115, 750);
        lowerMotor.move_velocity(600);
        chassis.moveToPose(-20, -28,145, 2000, {},false);
        lowerMotor.move_velocity(0);
        chassis.turnToHeading(225, 1000);
        chassis.moveToPose(-48, -48, 225, 1500);
        chassis.turnToHeading(270, 1000);
        loader.set_value(true);
        chassis.moveToPoint(-56.6, -48, 1000, {.minSpeed= 100,.earlyExitRange=4},false);
        lowerMotor.move_velocity(600);
        pros::delay(2750);
        lowerMotor.move_velocity(0);
        loader.set_value(false);
        chassis.moveToPoint(-27, -48, 2000,{.forwards=false});
        pros::delay(1750);
        lowerMotor.move_velocity(600);
        endIntake.move_velocity(600);
        /*chassis.setPose(-54,-17,90);
        lowerMotor.move_velocity(600);
        chassis.moveToPose(-40, -17, 90, 750,{.minSpeed = 90,.earlyExitRange=9});
        chassis.moveToPose(-24, -24, 135, 2500,{.lead=.001},false);
        pros::delay(500);
        lowerMotor.move_velocity(0);
        chassis.turnToHeading(45, 1000);
        chassis.moveToPose(-11.5, -13.5 , 45, 1500, {.minSpeed=50},false);
        lowerMotor.move_velocity(-600);
        upperMotor.move_velocity(-600);
        pros::delay(4000);
        chassis.moveToPose(-24, -24, 45, 1250,{.forwards=false},false);
        chassis.turnToHeading(225, 1000);
        lowerMotor.move_velocity(600);
        upperMotor.move_velocity(600);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        chassis.moveToPose(-63, -49, 270, 3250,{.lead=.62});
        pros::delay(1000);
        chassis.moveToPose(-48, -48, 270, 1000, {.forwards=false});
        chassis.turnToHeading(90, 800);
        chassis.moveToPose(-30.75, -48, 90, 2500);
        loaderVar = !loaderVar;
        loader.set_value(loaderVar);
        pivotVar = !pivotVar; // toggle
        descore.set_value(pivotVar);
        outtake=!outtake;
        outTake.set_value(outtake);
        pros::delay(10000);
        chassis.moveToPose(-48, -48, 90, 1000, {.forwards=false},false);
        chassis.turnToHeading(-45, 1250);
        chassis.moveToPose(-65, 0,0, 8000);*/
    }
    else if(autonNum ==2){
        chassis.setPose(-48.5,17,90);
        chassis.moveToPoint(-34, 17, 750);
        chassis.turnToHeading(65, 750);
        lowerMotor.move_velocity(600);
        chassis.moveToPose(-20, 28,35, 2000, {},false);
        lowerMotor.move_velocity(0);
        chassis.turnToHeading(315, 1000);
        chassis.moveToPose(-48, 48, 315, 1500);
        chassis.turnToHeading(270, 1000);
        loader.set_value(true);
        chassis.moveToPoint(-57.3, 48, 1000, {.minSpeed= 100,.earlyExitRange=4},false);
        lowerMotor.move_velocity(600);
        pros::delay(2750);
        lowerMotor.move_velocity(0);
        loader.set_value(false);
        chassis.moveToPoint(-27, 48, 2000,{.forwards=false});
        pros::delay(1750);
        lowerMotor.move_velocity(600);
        endIntake.move_velocity(600);
        /*chassis.setPose(-54,16,90);
        lowerMotor.move_velocity(600);
        upperMotor.move_velocity(600);
        chassis.moveToPose(-35, 16, 90, 1750);
        chassis.turnToHeading(55, 1500);
        chassis.moveToPose(-24, 24, 55, 3500,{.maxSpeed =70},false);
        pros::delay(100);
        lowerMotor.move_velocity(0);
        chassis.turnToHeading(135, 3000);
        chassis.moveToPose(-13, 13, 45, 2000);
        lowerMotor.move_velocity(600);
        upperMotor.move_velocity(600);
        outtake=!outtake;
        outTake.set_value(outtake);
        pros::delay(3500);
        chassis.moveToPose(-24, 24, 135, 2000,{.forwards=false},false);
        outtake=!outtake;
        outTake.set_value(outtake);
        chassis.turnToHeading(-45, 1500);
        lowerMotor.move_velocity(600);
        upperMotor.move_velocity(600);
        chassis.moveToPose(-59, 48, -90, 5000);
        pros::delay(1000);
        chassis.moveToPose(-48, 48, -90, 1000, {.forwards=false});
        chassis.turnToHeading(90, 1000);
        chassis.moveToPose(-31.75, 48, 90, 1500);
        outtake=!outtake;
        outTake.set_value(outtake);*/
    }
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
	
	while (true) {
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.arcade(leftY, rightY);
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            pivotVar = !pivotVar; // toggle
            descore.set_value(pivotVar);
        }
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            loaderVar = !loaderVar;
            loader.set_value(loaderVar);
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
        {
            outtake = !outtake;
            outTake.set_value(outtake);
        }
        pros::delay(20);
	}
}