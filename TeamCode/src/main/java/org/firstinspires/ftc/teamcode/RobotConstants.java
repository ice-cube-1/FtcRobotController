package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class RobotConstants {
    static final double COUNTS_PER_MOTOR_REV = (double) (3249 * 28) / 121 ;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 7.5 / 2.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static double speed = 0.2;

    static final double camera_range_tolerance = 1.0;
    static final double camera_strafe_tolerance = 1.0;
    static final double camera_yaw_tolerance = 4.0;

    public static double kI_drive = 0;
    public static double kP_drive = 0.02;
    public static double kD_drive = 0;

    static final double kI_turn = 0;
    static final double kP_turn = 0.02;
    static final double kD_turn = 0;

    static final double encoder_tolerance = 4;

    static final double pincer_in_pos = 158;
    static final double pincer_out_pos = 100;

    static final double elevator_base_position = 0;
    static final double arm_base_position = 0;
    static final double claw_rotation_time = 500;
}
