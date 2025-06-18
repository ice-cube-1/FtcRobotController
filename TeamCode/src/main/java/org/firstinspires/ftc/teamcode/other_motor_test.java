package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.arm_base_position;
import static org.firstinspires.ftc.teamcode.RobotConstants.claw_rotation_time;
import static org.firstinspires.ftc.teamcode.RobotConstants.elevator_base_position;
import static org.firstinspires.ftc.teamcode.RobotConstants.encoder_tolerance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Field Centric", group="Linear OpMode")
public class other_motor_test extends LinearOpMode {
    OtherMotors otherMotors;
    @Override
    public void runOpMode() {
        otherMotors = new OtherMotors(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.2) {
                otherMotors.spinning_star_a.state = OtherMotors.StarState.IN;
            } else if (gamepad1.right_trigger > 0.2) {
                otherMotors.spinning_star_a.state = OtherMotors.StarState.OUT;
            } else {
                otherMotors.spinning_star_a.state = OtherMotors.StarState.STOP;
            }
            if (gamepad1.dpad_up) {
                otherMotors.elevator.state = OtherMotors.MotorState.GOING_OUT;
            } else if (gamepad1.dpad_down) {
                otherMotors.elevator.state = OtherMotors.MotorState.GOING_IN;
            } else {
                if (otherMotors.elevator.state == OtherMotors.MotorState.GOING_IN || otherMotors.elevator.state == OtherMotors.MotorState.GOING_OUT) {
                    if (Math.abs(otherMotors.elevator.motor.drive.getCurrentPosition() - elevator_base_position) > encoder_tolerance) {
                        otherMotors.elevator.state = OtherMotors.MotorState.OUT;
                    } else {
                        otherMotors.elevator.state = OtherMotors.MotorState.IN;
                    }
                }
            }
            if (gamepad1.dpad_right) {
                otherMotors.arm.state = OtherMotors.MotorState.GOING_OUT;
            } else if (gamepad1.dpad_left) {
                otherMotors.arm.state = OtherMotors.MotorState.GOING_IN;
            } else {
                if (Math.abs(otherMotors.arm.motor.drive.getCurrentPosition() - arm_base_position) > encoder_tolerance) {
                    otherMotors.arm.state = OtherMotors.MotorState.OUT;
                } else {
                    otherMotors.arm.state = OtherMotors.MotorState.IN;
                }
            }
            if (gamepad1.left_bumper) {
                otherMotors.pincer_rotation.state = OtherMotors.ServoState.DOWN;
            } else if (gamepad1.right_bumper) {
                otherMotors.pincer_rotation.state = OtherMotors.ServoState.UP;
            }
            if (gamepad1.a) {
                otherMotors.claw_rotation.target_time = otherMotors.timer.milliseconds() + claw_rotation_time;
                if (otherMotors.claw_rotation.state == OtherMotors.MotorState.IN) {
                    otherMotors.claw_rotation.state = OtherMotors.MotorState.GOING_OUT;
                } else {
                    otherMotors.claw_rotation.state = OtherMotors.MotorState.GOING_IN;
                }
            }
            otherMotors.check_FSMs();
        }
    }
}