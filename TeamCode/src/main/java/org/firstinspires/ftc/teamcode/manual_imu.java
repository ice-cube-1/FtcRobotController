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
    Motor[] elevators;
    double imu_yaw = 0;
    double target_imu = 0;
    double lateral_multiplier = 0.01;
    double power_multiplier = 0.5;

    @Override
    public void runOpMode() {
        motors = new Motor[]{
                new Motor("left_front_drive", DcMotor.Direction.REVERSE),
                new Motor("left_back_drive", DcMotor.Direction.REVERSE),
                new Motor("right_front_drive", DcMotor.Direction.FORWARD),
                new Motor("right_back_drive", DcMotor.Direction.FORWARD),
        };
        elevators = new Motor[]{
                new Motor("left_elevator", DcMotor.Direction.FORWARD),
                new Motor("right_elevator", DcMotor.Direction.REVERSE)
        };
        waitForStart();
        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;

            if (gamepad1.dpad_up) {
                for (Motor elevator: elevators) {
                    elevator.drive.setPower(0.25);
                }
            }

            if (gamepad1.dpad_down) {
                for (Motor elevator: elevators) {
                    elevator.drive.setPower(-0.25);
                }
            }

            target_imu = target_imu + lateral * lateral_multiplier;


            motors[0].power = axial + yaw + lateral;
            motors[1].power = axial - yaw + lateral;
            motors[2].power = axial - yaw - lateral;
            motors[3].power = axial + yaw - lateral;
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

    void update_telemetry() {
        for (Motor motor : motors) {
            telemetry.addData(motor.name, motor.drive.getCurrentPosition());
        }
        telemetry.addData("target", target_imu);
        telemetry.addData("yaw", imu_yaw);
        telemetry.update();
    }
}
