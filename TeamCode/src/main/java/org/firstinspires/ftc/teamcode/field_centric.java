package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Field Centric", group="Linear OpMode")
public class field_centric extends LinearOpMode {

    Drivetrain drivetrain;
    OtherMotors otherMotors;
    private IMU imu;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap,0,0,0);
        otherMotors = new OtherMotors(hardwareMap);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {
            this.move_drivetrain();
            this.fsm_stuff();
        }
    }

    void move_drivetrain() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double heading = getHeadingRadians();
        double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);
        drivetrain.move(rotatedX, rotatedY, turn);
    }

    private double getHeadingRadians() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    void fsm_stuff() {
        if (gamepad1.x) {
            if (otherMotors.spinning_star_a.state != OtherMotors.StarState.X_THING) {
                otherMotors.spinning_star_a.target_time = timer.milliseconds()+500;
                otherMotors.spinning_star_a.state = OtherMotors.StarState.X_THING;
            }
        }
        if (gamepad1.left_trigger > 0.2) {
            otherMotors.arm.state = OtherMotors.MotorState.GOING_OUT;
        } else if (gamepad1.right_trigger > 0.2) {
            otherMotors.arm.state = OtherMotors.MotorState.GOING_IN;
        }  else if (otherMotors.arm.state != OtherMotors.MotorState.OUT) {
            otherMotors.arm.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            otherMotors.arm.motor.target = otherMotors.arm.motor.drive.getCurrentPosition();
            otherMotors.arm.state = OtherMotors.MotorState.OUT;
        }
        if (gamepad1.dpad_down) {
            otherMotors.claw_rotation.state = OtherMotors.MotorState.GOING_IN;
        } else if (gamepad1.dpad_up) {
            otherMotors.claw_rotation.state = OtherMotors.MotorState.GOING_OUT;
        } else if (otherMotors.claw_rotation.state != OtherMotors.MotorState.OUT) {
            otherMotors.claw_rotation.state = OtherMotors.MotorState.OUT;
        }
        if (gamepad1.a) {
            otherMotors.spinning_star_a.state = OtherMotors.StarState.IN;
        } else if (gamepad1.b) {
            otherMotors.spinning_star_a.state = OtherMotors.StarState.OUT;
        } else if (otherMotors.spinning_star_a.state != OtherMotors.StarState.X_THING) {
            otherMotors.spinning_star_a.state = OtherMotors.StarState.STOP;
        }
        if (gamepad2.left_stick_y > 0.2) {
            otherMotors.pincer_rotation.state = OtherMotors.MotorState.GOING_OUT;
        } else if (gamepad2.left_stick_y < -0.2) {
            otherMotors.pincer_rotation.state = OtherMotors.MotorState.GOING_IN;
        } else {
            otherMotors.pincer_rotation.state = OtherMotors.MotorState.IN;
        }
        if (gamepad2.b) {
            if (otherMotors.pincer.state == OtherMotors.ServoState.OPEN)
                otherMotors.pincer.state = OtherMotors.ServoState.CLOSED;
            else otherMotors.pincer.state = OtherMotors.ServoState.OPEN;
        }
        if (gamepad2.left_trigger > 0.5) {
            otherMotors.elevator.state = OtherMotors.MotorState.GOING_OUT;
        } else if (gamepad2.right_trigger > 0.5) {
            otherMotors.elevator.state = OtherMotors.MotorState.GOING_IN;
        } else if (otherMotors.elevator.state != OtherMotors.MotorState.OUT) {
            otherMotors.elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            otherMotors.other_elevator.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            otherMotors.elevator.motor.target = otherMotors.elevator.motor.drive.getCurrentPosition();
            otherMotors.elevator.state = OtherMotors.MotorState.IN;
        }
        otherMotors.check_FSMs();
        telemetry.addLine(otherMotors.elevator.motor.target+ " - "+ otherMotors.elevator.motor.drive.getCurrentPosition() + " - " + otherMotors.elevator.state);
        telemetry.update();
    }
}
