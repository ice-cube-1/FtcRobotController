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
    private final double speed = 0.5;

    static final double COUNTS_PER_MOTOR_REV = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double P_TURN_GAIN = 0.015;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;


    private Motor[] motors = null;
    private IMU imu = null;
    private double x = 0;
    private double y = 0;
    private double rotation_deg = 0;
    private double turnSpeed = 0;
    private double drive_x = 0;
    private double drive_y = 0;
    private double angle = 0;
    private double yaw = 0;

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
        x=0;
        y=0;
        /*
        * PUT DRIVE CODE HERE
        * BASE IT ON DRIVE TO POINT, TURN TO POSITION (RELATIVE TO START) CLOCKWISE ETC
        * SHOULD CURRENTLY BE IN INCHES / DEGREES */
        driveToPoint(0,30);
        driveToPoint(30,30);
        rotate(90, P_TURN_GAIN);
    }

    void rotate(double degrees, double gain) {
        if (opModeIsActive()) {
            double delta_deg = degrees - rotation_deg;
            double initialHeading = getHeading();
            double targetHeading = initialHeading + delta_deg;
            while (targetHeading > 180) targetHeading -= 360;
            while (targetHeading <= -180) targetHeading += 360;

            double proportionalGain = gain;
            while (opModeIsActive() && Math.abs(getHeading() - targetHeading) > 0.5) {
                turnSpeed = getSteeringCorrection(targetHeading, proportionalGain);
                moveRobot(0, 0, turnSpeed);
                updateTelemetry();
            }
            moveRobot(0, 0, 0);
        }
        rotation_deg = degrees;

        sleep(1000);
    }

    void driveToPoint(double new_x, double new_y) {
        double delta_x = new_x - x;
        double delta_y = new_y - y;

        double rotation_radians = Math.toRadians(rotation_deg);

        double transformed_delta_x = delta_x * Math.cos(rotation_radians) + delta_y * Math.sin(rotation_radians);
        double transformed_delta_y = -delta_x * Math.sin(rotation_radians) + delta_y * Math.cos(rotation_radians);

        double crow_flies = Math.sqrt(Math.pow(transformed_delta_x, 2) + Math.pow(transformed_delta_y, 2));
        angle = Math.toDegrees(Math.atan2(transformed_delta_y, transformed_delta_x));
        rotate(0,0.015);
        updateTelemetry();
        driveDistance(transformed_delta_x, transformed_delta_y, crow_flies);
        updateTelemetry();
        x = new_x;
        y = new_y;
        moveRobot(0,0,0);
    }


    void driveDistance(double x, double y, double distance) {
        if (opModeIsActive()) {
            drive_x = x;
            drive_y = y;
            motors[0].target = motors[0].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            motors[1].target = motors[1].drive.getCurrentPosition() + (int)((drive_y - drive_x)  * COUNTS_PER_INCH);
            motors[2].target = motors[2].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            motors[3].target = motors[3].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            for (Motor motor: motors) {
                motor.drive.setTargetPosition(motor.target);
                motor.drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            moveRobot(drive_x/distance, drive_y/distance, 0);
            while (opModeIsActive() && Arrays.stream(motors).anyMatch(motor -> motor.drive.isBusy())) {
                this.turnSpeed = getSteeringCorrection(rotation_deg, RobotAutoDriveByGyroMecanum.P_DRIVE_GAIN);
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
        telemetry.addData("rotation", rotation_deg);
        telemetry.addData("yaw", yaw);
        telemetry.addData("angle", angle);
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
        double headingError = getHeading() - desiredHeading;
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    IMU InitIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        return imu;
    }

    double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw(AngleUnit.DEGREES);
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}
