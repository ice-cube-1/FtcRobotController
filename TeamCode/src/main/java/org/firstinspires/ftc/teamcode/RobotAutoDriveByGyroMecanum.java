package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;




@Autonomous(name ="Robot: Mecanum drive by gyro", group="Robot")
public class RobotAutoDriveByGyroMecanum extends LinearOpMode {
    private final double speed = 0.2;

    static final double COUNTS_PER_MOTOR_REV = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.1;


    private Motor[] motors = null;
    private IMU imu = null;
    private double x = 0;
    private double y = 0;
    private double rotation_deg = 0;
    private double turnSpeed = 0;
    private double drive_x = 0;
    private double drive_y = 0;

    @Override
    public void runOpMode() {
        motors = new Motor[]{new Motor("left_front_drive", DcMotor.Direction.REVERSE),
                new Motor("right_front_drive", DcMotor.Direction.FORWARD),
                new Motor("left_back_drive", DcMotor.Direction.REVERSE),
                new Motor("right_back_drive", DcMotor.Direction.FORWARD)};
        imu = InitIMU();
        waitForStart();
        for (Motor motor: motors) {
            motor.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        imu.resetYaw();
        /*
        * PUT DRIVE CODE HERE
        * BASE IT ON DRIVE TO POINT, TURN TO POSITION (RELATIVE TO START) ETC
        * SHOULD CURRENTLY BE IN INCHES / DEGREES */
        driveToPoint(5,0);
        /*
        * rotate(40);
        * driveToPoint(0,0);
        */
    }

    void driveToPoint(double new_x, double new_y) {
        double delta_x = new_x - x;
        double delta_y = new_y - y;
        double crow_flies = Math.sqrt(Math.pow(delta_x, 2)+Math.pow(delta_y, 2));
        double angle = Math.toDegrees(Math.atan2(delta_y,delta_x));
        driveDistance(crow_flies, angle - rotation_deg, P_DRIVE_GAIN);
        x = new_x;
        y = new_y;
    }

    void rotate(double angle_deg) {
        driveDistance(-0.0001, rotation_deg-angle_deg, P_TURN_GAIN);
        rotation_deg = angle_deg;
    }

    void driveDistance(double distance, double heading, double proportionalGain) {
        if (opModeIsActive()) {
            drive_x = Math.cos(Math.toRadians(heading))*distance;
            drive_y = Math.sin(Math.toRadians(heading))*distance;
            motors[0].target = motors[0].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            motors[1].target = motors[1].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            motors[2].target = motors[2].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            motors[3].target = motors[3].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            for (Motor motor: motors) {
                motor.drive.setTargetPosition(motor.target);
                motor.drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            moveRobot(drive_x/distance, drive_y/distance, 0);
            while (opModeIsActive() && Arrays.stream(motors).allMatch(motor -> motor.drive.isBusy())) {
                this.turnSpeed = getSteeringCorrection(rotation_deg, proportionalGain);
                if (drive_y <= 0) {
                    turnSpeed *= -1.0;
                }
                moveRobot(drive_x/distance,drive_y/distance, turnSpeed);
                updateTelemetry();
            }
            moveRobot(0,0,0);
            for (Motor motor: motors) {
                motor.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("LF", motors[0].drive.getCurrentPosition());
        telemetry.addData("RF", motors[1].drive.getCurrentPosition());
        telemetry.addData("LB", motors[2].drive.getCurrentPosition());
        telemetry.addData("RB", motors[3].drive.getCurrentPosition());
        telemetry.addData("Turn speed",turnSpeed);
        telemetry.addData("move_x", drive_x);
        telemetry.addData("move_y", drive_y);
        telemetry.addData("LFT", motors[0].target);
        telemetry.addData("RFT", motors[1].target);
        telemetry.addData("LBT", motors[2].target);
        telemetry.addData("RBT", motors[3].target);
        telemetry.update();
    }

    class Motor {
        DcMotor drive;
        double speed = 0;
        int target = 0;
        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    void moveRobot(double drive_x, double drive_y, double turn) {
        motors[0].speed = drive_y + drive_x + turn;
        motors[1].speed = drive_y - drive_x - turn;
        motors[2].speed = drive_y - drive_x + turn;
        motors[3].speed = drive_y + drive_x - turn;
        double max = Arrays.stream(motors).mapToDouble(motor -> motor.speed).max().getAsDouble();
        for (Motor motor: motors) {
            motor.speed = motor.speed*speed/max;
            motor.drive.setPower(motor.speed);
        }
    }

    double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = desiredHeading - getHeading();
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    IMU InitIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        return imu;
    }

    double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}
