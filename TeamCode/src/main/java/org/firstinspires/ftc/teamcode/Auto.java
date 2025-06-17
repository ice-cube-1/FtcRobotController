package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

abstract public class Auto extends LinearOpMode {
    Motor[] drivetrain_motors;
    Othermotors otherMotors;
    Sensors sensors;

    protected double x;
    protected double y;
    protected double rotation_deg;
    protected double initial_rotation = 90;

    void init_stuff(double start_x, double start_y, double initial_rotation) {
        x = start_x;
        y = start_y;
        rotation_deg = initial_rotation;
        this.initial_rotation = initial_rotation;
        drivetrain_motors = new Motor[] {new Motor(hardwareMap, "left_front_drive", DcMotor.Direction.REVERSE),
                new Motor(hardwareMap, "left_back_drive", DcMotor.Direction.REVERSE),
                new Motor(hardwareMap, "right_front_drive", DcMotor.Direction.FORWARD),
                new Motor(hardwareMap, "right_back_drive", DcMotor.Direction.FORWARD)};
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
            moveRobot(0, 0, turn);
        }

        moveRobot(0, 0, 0);
        rotation_deg = targetHeading;
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

            drivetrain_motors[0].drive.setPower(0.2 * (forward + strafe + turn));
            drivetrain_motors[1].drive.setPower(0.2 * (forward - strafe - turn));
            drivetrain_motors[2].drive.setPower(0.2 * (forward - strafe + turn));
            drivetrain_motors[3].drive.setPower(0.2 * (forward + strafe - turn));

            detections = sensors.aprilTag.getDetections();

        }
        for (Motor motor: drivetrain_motors) {
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


    void driveDistance(double drive_x, double drive_y, double distance) {
        if (opModeIsActive()) {
            drivetrain_motors[0].target = drivetrain_motors[0].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            drivetrain_motors[1].target = drivetrain_motors[1].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            drivetrain_motors[2].target = drivetrain_motors[2].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            drivetrain_motors[3].target = drivetrain_motors[3].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            for (Motor motor: drivetrain_motors) {
                motor.drive.setTargetPosition(motor.target);
                motor.drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            moveRobot(drive_x/distance, drive_y/distance, 0);
            double integralTurn = 0, lastErrorTurn = 0;
            long lastTime = System.nanoTime();
            while (opModeIsActive() && (drivetrain_motors[0].drive.isBusy() || drivetrain_motors[1].drive.isBusy())) {
                double headingError = sensors.getHeading() + initial_rotation - rotation_deg;
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

            for (Motor motor: drivetrain_motors) {
                motor.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            moveRobot(0,0,0);
            sleep(100);
            rotate(rotation_deg);
            moveRobot(0,0,0);
        }
    }

    void moveRobot(double drive_x, double drive_y, double turn) {
        otherMotors.check_FSMs();
        telemetry.addLine(drive_y+" "+ drive_x +" " + turn);
        telemetry.addLine(String.valueOf(drivetrain_motors[0].drive.getCurrentPosition()));
        telemetry.addLine(String.valueOf(drivetrain_motors[1].drive.getCurrentPosition()));
        telemetry.addLine(String.valueOf(drivetrain_motors[2].drive.getCurrentPosition()));
        telemetry.addLine(String.valueOf(drivetrain_motors[3].drive.getCurrentPosition()));
        telemetry.update();
        drivetrain_motors[0].speed = drive_y - drive_x - turn;
        drivetrain_motors[1].speed = drive_y + drive_x + turn;
        drivetrain_motors[2].speed = drive_y + drive_x - turn;
        drivetrain_motors[3].speed = drive_y - drive_x + turn;
        double max = Arrays.stream(drivetrain_motors).mapToDouble(motor -> motor.speed).max().getAsDouble();
        for (Motor motor: drivetrain_motors) {
            motor.speed = motor.speed * speed / max;
            motor.drive.setPower(motor.speed);
        }
    }
}