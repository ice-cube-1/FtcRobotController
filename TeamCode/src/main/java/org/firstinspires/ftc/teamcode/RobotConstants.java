package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class RobotConstants {
    static final double COUNTS_PER_MOTOR_REV = (double) (3249 * 28) / 121 ;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 7.5 / 2.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static double speed = 0.4;

    static final double camera_range_tolerance = 1.0;
    static final double camera_strafe_tolerance = 1.0;
    static final double camera_yaw_tolerance = 4.0;

    public static double kI_drive = -0.02;
    public static double kP_drive = -0.5;
    public static double kD_drive = -0.04;

    public static double kI_turn = 0;
    public static double kP_turn = 0.02;
    public static double kD_turn = 0;

    static final double encoder_tolerance = 4;

    public static double pincer_open_pos = 94-15;
    public static double pincer_closed_pos = 113;

    public static int elevator_base_position = 0;
    public static int elevator_up_position = 0;
    public static double elevator_speed = 0.2;

    public static int arm_base_position = 0;
    public static int arm_out_position = 0;
    public static double arm_speed = 0.5;

    public static double claw_rotation_time = 500;
    public static double pincer_rotation_time = 1000;
    public static double star_time = 500;
    public static double cr_speed = 1.0;
}
