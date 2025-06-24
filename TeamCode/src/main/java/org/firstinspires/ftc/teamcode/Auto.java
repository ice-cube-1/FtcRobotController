package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

abstract public class Auto extends LinearOpMode {
    Drivetrain drivetrain;
    OtherMotors otherMotors;
    Sensors sensors;
    protected double initial_rotation = 90;

    void init_stuff(double start_x, double start_y, double initial_rotation) {
        this.initial_rotation = initial_rotation;
        drivetrain = new Drivetrain(hardwareMap, start_x, start_y, initial_rotation);
        otherMotors = new OtherMotors(hardwareMap);
        sensors = new Sensors(hardwareMap);
        waitForStart();
        sensors.imu.resetYaw();
    }


    void rotate(double targetHeading) {
        double headingError = sensors.getHeading() + initial_rotation - targetHeading;
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        double integral = 0;
        double lastError = headingError;
        long lastTime = System.nanoTime();

        while (opModeIsActive() && Math.abs(headingError) > 1.0) {
            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            headingError = sensors.getHeading() + initial_rotation - targetHeading;
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            integral += headingError * deltaTime;
            double derivative = (headingError - lastError) / deltaTime;
            lastError = headingError;

            double turn = kP_turn * headingError + kI_turn * integral + kD_turn * derivative;
            turn = clip(turn);
            drivetrain.move(0, 0, turn);
        }

        drivetrain.move(0, 0, 0);
        drivetrain.rotation_deg = targetHeading;
    }

    private double clip(double val) {
        return Math.max(-1.0, Math.min(1.0, val));
    }

    void centerOnAprilTag(double target_range, double target_angle_degrees) {
        List<AprilTagDetection> detections = sensors.aprilTag.getDetections();
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

                telemetry.addLine(String.format(Locale.UK,"X Error: %.1f, Y Error: %.1f, Yaw Error: %.1f°", errorX, errorY, yawError));
                telemetry.update();

                boolean centered = Math.abs(errorX) < camera_strafe_tolerance &&
                        Math.abs(errorY) < camera_range_tolerance &&
                        Math.abs(yawError) < camera_yaw_tolerance;

                if (centered) break;

                forward = clip(errorY / 2.0);
                strafe = clip(errorX / 15.0);
                turn = clip(yawError / 15.0);

            }

            this.drivetrain.motors[0].drive.setPower(0.2 * (forward + strafe + turn));
            this.drivetrain.motors[1].drive.setPower(0.2 * (forward - strafe - turn));
            this.drivetrain.motors[2].drive.setPower(0.2 * (forward - strafe + turn));
            this.drivetrain.motors[3].drive.setPower(0.2 * (forward + strafe - turn));

            detections = sensors.aprilTag.getDetections();

        }
        for (Motor motor: drivetrain.motors) {
            motor.drive.setPower(0);
        }
    }

    void driveToPoint(double new_x, double new_y) {
        double delta_x = -new_x + drivetrain.x;
        double delta_y = new_y - drivetrain.y;

        double rotation_radians = Math.toRadians(drivetrain.rotation_deg);

        double transformed_delta_x = delta_x * Math.cos(rotation_radians) + delta_y * Math.sin(rotation_radians);
        double transformed_delta_y = -delta_x * Math.sin(rotation_radians) + delta_y * Math.cos(rotation_radians);

        double crow_flies = Math.sqrt(Math.pow(transformed_delta_x, 2) + Math.pow(transformed_delta_y, 2));
        driveDistance(transformed_delta_x, transformed_delta_y, crow_flies);
        drivetrain.x = new_x;
        drivetrain.y = new_y;
    }


    void driveDistance(double drive_x, double drive_y, double distance) {
        if (opModeIsActive()) {
            drivetrain.motors[0].target = drivetrain.motors[0].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            drivetrain.motors[1].target = drivetrain.motors[1].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            drivetrain.motors[2].target = drivetrain.motors[2].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            drivetrain.motors[3].target = drivetrain.motors[3].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            for (Motor motor: drivetrain.motors) {
                motor.drive.setTargetPosition(motor.target);
                motor.drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            drivetrain.move(drive_x/distance, drive_y/distance, 0);
            double integralTurn = 0, lastErrorTurn = 0;
            long lastTime = System.nanoTime();
            while (opModeIsActive() && (drivetrain.motors[0].drive.isBusy() || drivetrain.motors[1].drive.isBusy())) {
                double headingError = sensors.getHeading() + initial_rotation - drivetrain.rotation_deg;
                while (headingError > 180) headingError -= 360;
                while (headingError <= -180) headingError += 360;

                long now = System.nanoTime();
                double deltaTime = (now - lastTime) / 1e9;
                lastTime = now;
                integralTurn += headingError * deltaTime;
                double derivativeTurn = (headingError - lastErrorTurn) / deltaTime;
                double pidTurn = kP_drive * headingError + kI_drive * integralTurn + kD_drive * derivativeTurn;
                telemetry.addLine(String.valueOf(headingError));
                telemetry.update();
                lastErrorTurn = headingError;
                drivetrain.move(drive_x/distance, drive_y/distance, -pidTurn);
            }

            for (Motor motor: drivetrain.motors) {
                motor.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            drivetrain.move(0,0,0);
            sleep(100);
            rotate(drivetrain.rotation_deg);
            drivetrain.move(0,0,0);
        }
    }
}