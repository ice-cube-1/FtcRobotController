package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="non drivetrain only", group="Linear OpMode")
public class nonDrivetrainTest extends LinearOpMode {

    private OtherMotors otherMotors;

    @Override
    public void runOpMode() {
        otherMotors = new OtherMotors(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                telemetry.addLine("up");
                otherMotors.elevator.state = OtherMotors.MotorState.GOING_OUT;
            } else if (gamepad1.dpad_down) {
                telemetry.addLine("down");
                otherMotors.elevator.state = OtherMotors.MotorState.GOING_IN;
            } else if (otherMotors.elevator.state != OtherMotors.MotorState.OUT) {
                otherMotors.elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                otherMotors.elevator.motor.target = otherMotors.elevator.motor.drive.getCurrentPosition();
                otherMotors.elevator.state = OtherMotors.MotorState.OUT;
            }
            if (gamepad1.dpad_right) {
                otherMotors.arm.state = OtherMotors.MotorState.GOING_OUT;
            } else if (gamepad1.dpad_left) {
                otherMotors.arm.state = OtherMotors.MotorState.GOING_IN;
            }  else if (otherMotors.arm.state != OtherMotors.MotorState.OUT) {
                otherMotors.arm.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                otherMotors.arm.motor.target = otherMotors.arm.motor.drive.getCurrentPosition();
                otherMotors.arm.state = OtherMotors.MotorState.OUT;
            }
            if (gamepad1.a) {
                otherMotors.pincer_rotation.state = OtherMotors.MotorState.GOING_OUT;
            } else if (gamepad1.b) {
                otherMotors.pincer_rotation.state = OtherMotors.MotorState.GOING_IN;
            }
            telemetry.addLine(String.valueOf(otherMotors.claw_rotation.state));
            if (gamepad1.left_bumper) {
                otherMotors.claw_rotation.state = OtherMotors.MotorState.GOING_IN;
            } else if (gamepad1.right_bumper) {
                otherMotors.claw_rotation.state = OtherMotors.MotorState.GOING_OUT;
            } else if (otherMotors.claw_rotation.state != OtherMotors.MotorState.OUT) {
                otherMotors.claw_rotation.state = OtherMotors.MotorState.OUT;
            }
            if (gamepad1.left_trigger > 0.5) {
                otherMotors.spinning_star_a.state = OtherMotors.StarState.IN;
            } else if (gamepad1.right_trigger > 0.5) {
                otherMotors.spinning_star_a.state = OtherMotors.StarState.OUT;
            } else {
                otherMotors.spinning_star_a.state = OtherMotors.StarState.STOP;
            }
            if (gamepad1.x) {
                otherMotors.pincer.state = OtherMotors.ServoState.OPEN;
            } else if (gamepad1.y) {
                otherMotors.pincer.state = OtherMotors.ServoState.CLOSED;
            }
            otherMotors.check_FSMs();
            telemetry.addLine(String.valueOf(otherMotors.elevator.motor.drive.getCurrentPosition()));
            telemetry.update();
        }
    }
}
