package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;

@TeleOp(name="Field Centric", group="Linear OpMode")
public class field_centric extends LinearOpMode {

    Motor[] motors;
    private IMU imu;

    @Override
    public void runOpMode() {
        this.init_hardware();
        initIMU();
        waitForStart();
        while (opModeIsActive()) {
            this.move_drivetrain();
        }
    }

    void move_drivetrain() {
        // Read joystick values
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double heading = getHeadingRadians();

        double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);
        telemetry.addLine(heading + " "+ rotatedY + " "+rotatedX);
        telemetry.update();
        motors[0].power = rotatedY - rotatedX - turn;
        motors[1].power = rotatedY + rotatedX + turn;
        motors[2].power = rotatedY + rotatedX - turn;
        motors[3].power = rotatedY - rotatedX + turn;

        double max = Arrays.stream(motors).mapToDouble(motor -> Math.abs(motor.power)).max().orElse(1);
        for (Motor motor : motors) {
            double scaledPower = motor.power;
            if (max > 1.0) {
                scaledPower /= max;
            }
            motor.drive.setPower(scaledPower);
        }

        telemetry.addData("Heading", Math.toDegrees(heading));
        telemetry.update();
    }

    void init_hardware() {
        motors = new Motor[]{
                new Motor("left_front_drive", DcMotor.Direction.FORWARD),
                new Motor("left_back_drive", DcMotor.Direction.FORWARD),
                new Motor("right_front_drive", DcMotor.Direction.REVERSE),
                new Motor("right_back_drive", DcMotor.Direction.REVERSE),
        };
    }

    private void initIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
    }

    private double getHeadingRadians() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return -orientation.getYaw(AngleUnit.RADIANS);
    }

    class Motor {
        DcMotor drive;
        double power;

        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
