package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="elevator test only", group="Linear OpMode")
public class ElevatorTest extends LinearOpMode {

    private OtherMotors otherMotors;


    @Override
    public void runOpMode() {
        otherMotors = new OtherMotors(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                telemetry.addLine("up");
                otherMotors.elevator.motor.target = 600;
                otherMotors.elevator.state = OtherMotors.MotorState.GOING_OUT;
            } else if (gamepad1.dpad_down) {
                telemetry.addLine("down");
                otherMotors.elevator.motor.target = 0;
                otherMotors.elevator.state = OtherMotors.MotorState.GOING_IN;
            } else if (otherMotors.elevator.state != OtherMotors.MotorState.OUT) {
                otherMotors.elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                otherMotors.elevator.motor.target = otherMotors.elevator.motor.drive.getCurrentPosition();
                otherMotors.elevator.state = OtherMotors.MotorState.OUT;
            }
            otherMotors.check_FSMs();
            telemetry.addLine(String.valueOf(otherMotors.elevator.motor.drive.getCurrentPosition()));
            telemetry.update();
        }
    }
}
