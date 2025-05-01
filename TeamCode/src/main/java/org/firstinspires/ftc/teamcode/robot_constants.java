package org.firstinspires.ftc.teamcode;

public final class robot_constants {
    static final double P_TURN_GAIN = 0.015;
    static final double P_DRIVE_GAIN = 0.07;

    static final double COUNTS_PER_MOTOR_REV = (double) (3249 * 28) / 121 ;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 7.5 / 2.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double robot_length_inches = 18;

    static final double speed = 0.5;

    static final int elevator_position_up = 5000;
    static final int elevator_position_down = 25;

    static final int arm_motor_position_basket = 400;
    static final int arm_motor_position_rest = 200;

    static final double range_tolerance = 1.0;
    static final double bearing_tolerance = 4.0;
    static final double yaw_tolerance = 2.0;


    enum ElevatorState {
        TO_UP,
        TO_DOWN,
        UP,
        DOWN,
    }

    enum ArmState {
        MOTOR_TO_REST,
        REST,
        MOTOR_TO_BASKET,
        BASKET
    }
}
