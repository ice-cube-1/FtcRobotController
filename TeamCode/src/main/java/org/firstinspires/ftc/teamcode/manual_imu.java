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
    double prev_imu_yaw = 0;
    double lateral_multiplier = 0.1;
    double proportional_gain = 0.1;
    double power_multiplier = 0.25;

    @Override
    public void runOpMode() {
        motors = new Motor[] {
                new Motor("left_front_drive", DcMotor.Direction.REVERSE),
                new Motor("left_back_drive", DcMotor.Direction.REVERSE),
                new Motor("right_front_drive", DcMotor.Direction.FORWARD),
                new Motor("right_back_drive", DcMotor.Direction.FORWARD),
        };

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {
            double max;

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.right_stick_x;
            double yaw     =  gamepad1.left_stick_x;

            prev_imu_yaw = imu_yaw + lateral*lateral_multiplier;

            imu_yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (Math.abs(yaw) > Math.abs(axial)) axial = 0;
            else yaw = 0;

            motors[0].power = axial + yaw;
            motors[1].power = axial - yaw;
            motors[2].power = axial + yaw;
            motors[3].power = axial - yaw;

            rotate_slightly(imu_yaw-prev_imu_yaw, proportional_gain);

            max = Arrays.stream(motors).mapToDouble(motor -> motor.power).max().orElse(1);
            if (max > 1.0) {
                for (Motor motor: motors) {
                    motor.power/=max;
                }
            }

            for (Motor motor: motors) {
                motor.drive.setPower(motor.power*power_multiplier);
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
        for (Motor motor: motors) {
            telemetry.addData(motor.name, motor.drive.getCurrentPosition());
        }
        telemetry.update();
    }

    void rotate_slightly(double rad, double pg) {
        motors[0].power+=rad*pg;
        motors[1].power-=rad*pg;
        motors[2].power-=rad*pg;
        motors[3].power+=rad*pg;
    }

}
