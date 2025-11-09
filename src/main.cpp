#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include <sys/_intsup.h>
#include "pros/llemu.hpp"

int autonNum = 0;
int intakeRun = 0;
int outtake = 0;
bool pivotVar = true;
bool loaderVar = true;
pros::MotorGroup left_motors({-6, 16, -17}, pros::MotorGearset::blue); // Left motors on ports 20, 3, 5
pros::MotorGroup right_motors({7, -9, 8}, pros::MotorGearset::blue); // Right motors on ports 13, 16, 17
pros::Imu imu(10);
pros::MotorGroup intake({-18,-19});
pros::Motor endIntake(20);
pros::Rotation horizontal_sensor(8);
pros::Rotation vert_sensor(12);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut pivot('A');
pros::adi::DigitalOut loader('C');
lemlib::Drivetrain drivetrain(&left_motors, // Left motor group
                              &right_motors, // Right motor group
                              9.85, // 12.675 inch track width
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
lemlib::ControllerSettings lateral_controller(3, // Proportional gain (kP)
                                              0, // Integral gain (kI)
                                              15, // Derivative gain (kD)
                                              0, // Anti windup 3
                                              0, // Small error range, in inches .25
                                              00, //100 Small error range timeout, in milliseconds
                                              0, // 1Large error range, in inches
                                              00, //500 Large error range timeout, in milliseconds
                                              20 // 20Maximum acceleration (slew)
);

// Angular PID controller
lemlib::ControllerSettings angular_controller(2.05, // Proportional gain (kP)
                                              .2, // Integral gain (kI)
                                              20, // Derivative gain (kD)
                                              6, // Anti windup
                                              .225, // Small error range, in degrees.5
                                              500, // 500Small error range timeout, in milliseconds
                                              .75, // 1Large error range, in degrees
                                              1000, // 1000Large error range timeout, in milliseconds
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
    loader.set_value(true);
    pros::lcd::initialize();
    //endIntake.set_brake_mode(pros::MotorBrake::hold);
    pros:: Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
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
                intake.move_velocity(600*intakeRun);
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                    outtake = 1;
                }
                else {
                    outtake=0;
                }
                endIntake.move_velocity(600*outtake);
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
    if(autonNum == 0)
    {
        
        chassis.setPose(0,0,0);
        //intake.move_velocity(600);
        chassis.moveToPoint(0, 48, 100000);
    }
    else if(autonNum ==1){
        chassis.setPose(-65,-16,90);

        chassis.moveToPose(-35, -16, 90, 5000);
        chassis.turnToHeading(135, 3000);
        chassis.moveToPose(-14.08, -30, 135, 3000);
        chassis.turnToHeading(225, 3000);
        chassis.moveToPose(-48, -48, 225, 5000);
        chassis.turnToHeading(90, 3000);
        chassis.moveToPose(30, -48, 90, 5000);
    }
    else if(autonNum ==2){
        chassis.setPose(-65,15.375,90);

        chassis.moveToPose(-29, 15.375, 90, 5000);
        chassis.turnToHeading(45, 3000);
        chassis.moveToPose(-14.08, 30, 45, 3000);
        chassis.turnToHeading(-45, 3000);
        chassis.moveToPose(-48, 48, -45, 5000);
        chassis.turnToHeading(90, 3000);
        chassis.moveToPose(30, 48, 90, 5000);
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
            pivot.set_value(pivotVar);
        }
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            if(loaderVar)
            {  
                loader.set_value(true);                
            }
            else {
                loader.set_value(false);
            }
            loaderVar = !loaderVar;
        }
        pros::delay(20);
	}
}