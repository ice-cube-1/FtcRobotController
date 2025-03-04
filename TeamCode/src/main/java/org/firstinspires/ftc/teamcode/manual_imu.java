package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

@TeleOp(name="Manual IMU", group="Linear OpMode")
public class manual_imu extends LinearOpMode {

    Motor[] motors;
    IMU imu;
    double imu_yaw = 0;
    double target_imu = 0;
    double lateral_multiplier = 0.01;
    double proportional_gain = 6;
    double derivative_gain = 0.4; // New derivative gain
    double power_multiplier = 0.5;
    double previous_error = 0; // Previous error for derivative calculation

    @Override
    public void runOpMode() {
        motors = new Motor[]{
                new Motor("left_front_drive", DcMotor.Direction.REVERSE),
                new Motor("left_back_drive", DcMotor.Direction.REVERSE),
                new Motor("right_front_drive", DcMotor.Direction.FORWARD),
                new Motor("right_back_drive", DcMotor.Direction.FORWARD),
        };
        init_imu();

        waitForStart();
        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;

            target_imu = target_imu + lateral * lateral_multiplier;

            imu_yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double error = imu_yaw - target_imu;
            double derivative = error - previous_error; // Calculate derivative
            previous_error = error; // Update previous error

            motors[0].power = axial + yaw + lateral;
            motors[1].power = axial - yaw + lateral;
            motors[2].power = axial - yaw - lateral;
            motors[3].power = axial + yaw - lateral;
            /*
            double rotate_adjustment = error * proportional_gain * (1+Math.abs(lateral)) + derivative * derivative_gain; // PID control
            rotate_adjustment = Math.atan2(Math.sin(rotate_adjustment), Math.cos(rotate_adjustment));
            if (Math.abs(error) > 0.08)
                rotate_slightly(rotate_adjustment);

            if (yaw != 0) {
                for (Motor motor : motors) {
                    motor.power += rotate_adjustment / Math.abs(rotate_adjustment) * 0.8;
                }
            }

            motors[0].power *= 1.2;
            motors[2].power *= 1.2;

            */
            double max = Arrays.stream(motors).mapToDouble(motor -> Math.abs(motor.power)).max().orElse(1);
            if (max > 1.0) {
                for (Motor motor : motors) {
                    motor.power /= max;
                }
            }

            for (Motor motor : motors) {
                motor.drive.setPower(motor.power * power_multiplier);
            }

            update_telemetry();
        }
    }

    class Motor {
        DcMotor drive;
        double power;
        String name;

        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.name = name;
        }
    }

    void init_imu() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    void update_telemetry() {
        for (Motor motor : motors) {
            telemetry.addData(motor.name, motor.drive.getCurrentPosition());
        }
        telemetry.addData("target", target_imu);
        telemetry.addData("yaw", imu_yaw);
        telemetry.update();
    }

    void rotate_slightly(double rad) {
        motors[0].power += rad;
        motors[1].power -= rad;
        motors[2].power -= rad;
        motors[3].power += rad;
    }
}
