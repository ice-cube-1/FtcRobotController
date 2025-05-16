package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robot_constants.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

abstract public class auto_new extends LinearOpMode {

    protected double start_x = 0;
    protected double start_y = 0;
    protected double initial_rotation = 90;

    protected Motor[] motors = null;
    private IMU imu = null;

    protected double x;
    protected double y;
    protected double rotation_deg;
    private double turnSpeed = 0;
    private double drive_x = 0;
    private double drive_y = 0;
    private double yaw = 0;

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;


    void init_stuff() {
        x = start_x;
        y = start_y;
        rotation_deg = initial_rotation;
        motors = new Motor[] {new Motor("left_front_drive", DcMotor.Direction.FORWARD),
                new Motor("right_front_drive", DcMotor.Direction.REVERSE),
                new Motor("left_back_drive", DcMotor.Direction.FORWARD),
                new Motor("right_back_drive", DcMotor.Direction.REVERSE)};
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        imu = InitIMU();
        waitForStart();
        for (Motor motor: motors) {
            motor.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        imu.resetYaw();

    }


    void rotate(double targetHeading) {
        double headingError = getHeading() - targetHeading;
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        double integral = 0;
        double lastError = headingError;
        long lastTime = System.nanoTime();

        while (opModeIsActive() && Math.abs(headingError) > 1.0) {
            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;  // in seconds
            lastTime = now;

            headingError = getHeading() - targetHeading;
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            integral += headingError * deltaTime;
            double derivative = (headingError - lastError) / deltaTime;
            lastError = headingError;

            double turn = kP_turn * headingError + kI_turn * integral + kD_turn * derivative;
            turn = clip(turn);
            moveRobot(0, 0, turn);
        }

        moveRobot(0, 0, 0);
        rotation_deg = targetHeading;
    }

    private double clip(double val) {
        return Math.max(-1.0, Math.min(1.0, val));
    }

    void centerOnAprilTag(double target_range, double target_angle_degrees) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("detection size ",detections.size());
        double forward = 0;
        double strafe = 0;
        double turn = 0;

        double target_angle_rad = Math.toRadians(target_angle_degrees);
        double desiredX = -target_range * Math.sin(target_angle_rad);
        double desiredY = target_range * Math.cos(target_angle_rad);

        while (true) {
            if (detections.size() == 1) {
                AprilTagDetection tag = detections.get(0);

                double errorX = -tag.ftcPose.x+desiredX;
                double errorY = tag.ftcPose.y - desiredY;
                double yawError = tag.ftcPose.yaw - target_angle_degrees;

                telemetry.addLine(String.format("X Error: %.1f, Y Error: %.1f, Yaw Error: %.1f°", errorX, errorY, yawError));
                telemetry.update();

                boolean centered = Math.abs(errorX) < strafe_tolerance &&
                        Math.abs(errorY) < range_tolerance &&
                        Math.abs(yawError) < yaw_tolerance;

                if (centered) break;

                forward = clip(errorY / 2.0);
                strafe = clip(errorX / 15.0);
                turn = clip(yawError / 15.0);

            }

            motors[0].drive.setPower(0.2 * (forward + strafe + turn));
            motors[1].drive.setPower(0.2 * (forward - strafe - turn));
            motors[2].drive.setPower(0.2 * (forward - strafe + turn));
            motors[3].drive.setPower(0.2 * (forward + strafe - turn));

            detections = aprilTag.getDetections();

        }
        for (Motor motor: motors) {
            motor.drive.setPower(0);
        }
    }

    void driveToPoint(double new_x, double new_y) {
        double delta_x = -new_x + x;
        double delta_y = new_y - y;

        double rotation_radians = Math.toRadians(rotation_deg);

        double transformed_delta_x = delta_x * Math.cos(rotation_radians) + delta_y * Math.sin(rotation_radians);
        double transformed_delta_y = -delta_x * Math.sin(rotation_radians) + delta_y * Math.cos(rotation_radians);

        double crow_flies = Math.sqrt(Math.pow(transformed_delta_x, 2) + Math.pow(transformed_delta_y, 2));
        driveDistance(transformed_delta_x, transformed_delta_y, crow_flies);
        x = new_x;
        y = new_y;
    }


    void driveDistance(double x, double y, double distance) {
        if (opModeIsActive()) {
            drive_x = x;
            drive_y = y;
            motors[0].target = motors[0].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            motors[1].target = motors[1].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            motors[2].target = motors[2].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            motors[3].target = motors[3].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            for (Motor motor: motors) {
                motor.drive.setTargetPosition(motor.target);
                motor.drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            moveRobot(drive_x/distance, drive_y/distance, 0);
            double integralTurn = 0, lastErrorTurn = 0;
            long lastTime = System.nanoTime();
            while (opModeIsActive() && (motors[0].drive.isBusy() || motors[1].drive.isBusy())) {
                double headingError = getHeading() - rotation_deg;
                while (headingError > 180) headingError -= 360;
                while (headingError <= -180) headingError += 360;

                long now = System.nanoTime();
                double deltaTime = (now - lastTime) / 1e9;
                lastTime = now;
                integralTurn += headingError * deltaTime;
                double derivativeTurn = (headingError - lastErrorTurn) / deltaTime;
                double pidTurn = kP_drive * headingError + kI_drive * integralTurn + kD_drive * derivativeTurn;
                lastErrorTurn = headingError;
                moveRobot(drive_x/distance, drive_y/distance, -pidTurn);
            }

            for (Motor motor: motors) {
                motor.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            moveRobot(0,0,0);
            sleep(100);
            rotate(rotation_deg);
            moveRobot(0,0,0);
        }
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
            motor.speed = motor.speed * speed / max;
            motor.drive.setPower(motor.speed);
        }
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
        yaw = orientation.getYaw(AngleUnit.DEGREES);
        return yaw + initial_rotation;
    }

}
