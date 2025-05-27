package org.firstinspires.ftc.teamcode;

public final class RobotConstants {
    static final double COUNTS_PER_MOTOR_REV = (double) (3249 * 28) / 121 ;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 7.5 / 2.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double speed = 0.5;

    static final double camera_range_tolerance = 1.0;
    static final double camera_strafe_tolerance = 1.0;
    static final double camera_yaw_tolerance = 4.0;

    static final double kI_drive = 0;
    static final double kP_drive = 0.02;
    static final double kD_drive = 0;

    static final double kI_turn = 0;
    static final double kP_turn = 0.02;
    static final double kD_turn = 0;
}
