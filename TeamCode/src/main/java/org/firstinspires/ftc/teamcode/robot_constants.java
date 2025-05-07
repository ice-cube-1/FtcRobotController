package org.firstinspires.ftc.teamcode;

public final class robot_constants {
    static final double P_TURN_GAIN = 0.015;
    static final double P_DRIVE_GAIN = 0.07;

    static final double COUNTS_PER_MOTOR_REV = (double) (3249 * 28) / 121 ;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 7.5 / 2.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double speed = 0.5;

    static final double range_tolerance = 1.0;
    static final double strafe_tolerance = 1.0;
    static final double yaw_tolerance = 4.0;
}
