package org.firstinspires.ftc.teamcode;

public final class robot_constants {
    static final double P_TURN_GAIN = 0.015;
    static final double P_DRIVE_GAIN = 0.07;

    static final double COUNTS_PER_MOTOR_REV = (double) (3249 * 28) / 121 ;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 5.51181;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double robot_length_inches = 18;

    static final double speed = 0.5;

    static final int elevator_position_up = 5000;
    static final int elevator_position_down = 25;

    static final int arm_position_basket = 400;
    static final int arm_position_up = 200;
    static final int arm_position_rest = 0;

    enum ElevatorState {
        TO_UP,
        TO_DOWN,
        UP,
        DOWN,
    }

    enum ArmState {
        TO_REST,
        REST,
        TO_UP,
        UP,
        TO_BASKET,
        BASKET,
    }
}
