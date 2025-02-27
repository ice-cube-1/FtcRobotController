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
    double lateral_multiplier = 0.1;
    double proportional_gain = 12;
    double power_multiplier = 0.5;

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
            double yaw = -gamepad1.left_stick_x;

            target_imu = target_imu + lateral * lateral_multiplier;

            imu_yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (Math.abs(yaw) > Math.abs(axial)) axial = 0;
            else yaw = 0;

            motors[0].power = axial + yaw;
            motors[1].power = axial - yaw;
            motors[2].power = axial + yaw;
            motors[3].power = axial - yaw;
            double rotate_adjustment = imu_yaw - target_imu;
            rotate_adjustment = Math.atan2(Math.sin(rotate_adjustment), Math.cos(rotate_adjustment));
            if (Math.abs(rotate_adjustment) > 0.1)
                rotate_slightly(rotate_adjustment, proportional_gain);
            if (yaw != 0) {
                for (Motor motor : motors) {
                    motor.power += rotate_adjustment / Math.abs(rotate_adjustment)*0.2;
                }
            }
            motors[0].power *= 1.1;
            motors[2].power *= 1.1;
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

    void rotate_slightly(double rad, double pg) {
        motors[0].power += rad * pg;
        motors[1].power -= rad * pg;
        motors[2].power -= rad * pg;
        motors[3].power += rad * pg;
    }
}
