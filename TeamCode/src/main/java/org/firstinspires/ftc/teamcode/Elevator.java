package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Elevator.ElevatorState.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="elevator", group="Linear OpMode")
public class Elevator extends LinearOpMode {

    private Motor elevator;
    private final double elevatorPower = 0.2;
    private final int elevatorMaxPosition = 500; // TODO Find actual value from encoder test
    private final double percentageDown = 0.4;
    private final int tolerance = 5; // TODO add tolerance if needed, same units as mas position
    private ElevatorState elevatorState = STOP;
    enum ElevatorState {HOLD, UP, DOWN, STOP, MANUAL_UP, MANUAL_DOWN}

    @Override
    public void runOpMode() {
        elevator = new Motor("elevator", DcMotor.Direction.FORWARD);
        elevator.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while (!isStarted()) {
            elevatorTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            teleOpControls();
            checkElevator();
            elevatorTelemetry();
        }
    }

    public void checkElevator() {
        switch (elevatorState) {
            case MANUAL_UP -> elevator.drive.setPower(elevatorPower);
            case MANUAL_DOWN -> elevator.drive.setPower(-elevatorPower);
            case HOLD -> {
                int difference = elevator.target - elevator.drive.getCurrentPosition();
                elevator.drive.setPower(elevatorPower*((double) difference/Math.abs(difference)));
            }
            case UP -> {
                if (Math.abs(elevator.drive.getCurrentPosition() - elevator.target) > tolerance) elevatorState = HOLD;
                else elevator.drive.setPower(elevatorPower);
            }
            case DOWN -> {
                if (Math.abs(elevator.drive.getCurrentPosition() - elevator.target) > tolerance) elevatorState = STOP;
                else elevator.drive.setPower(-elevatorPower);
            }
            case STOP -> elevator.drive.setPower(0);
        }
    }

    public void teleOpControls() {
        if (gamepad1.right_bumper) elevatorState = MANUAL_UP;
        else if (gamepad1.left_bumper) elevatorState = MANUAL_DOWN;
        else if (gamepad1.x) elevatorState = STOP;
        else if (gamepad1.a) {
            elevator.target = (int) (elevator.drive.getCurrentPosition()*percentageDown);
            elevatorState = DOWN;
        }
        else if (gamepad1.b) {
            elevator.target = elevatorMaxPosition;
            elevatorState = UP;
        }
        else {
            elevatorState = HOLD;
            elevator.target = elevator.drive.getCurrentPosition();
        }
    }

    void elevatorTelemetry() {
        telemetry.addData("Encoder position", elevator.drive.getCurrentPosition());
        telemetry.addData("Target", elevator.target);
        telemetry.update();
    }
}
