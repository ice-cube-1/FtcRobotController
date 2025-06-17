package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Elevator.ElevatorState.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="elevator", group="Linear OpMode")
public class Elevator extends LinearOpMode {

    private Motor elevator;
    private final double elevatorPower = 0.5;
    private final int elevatorMaxPosition = 500; // TODO Find actual value from encoder test
    private final double percentageDown = 0.4;
    private final int tolerance = 2; // TODO add tolerance if needed, same units as mas position
    private ElevatorState elevatorState = STOP;
    enum ElevatorState {HOLD, UP, DOWN, STOP, MANUAL_UP, MANUAL_DOWN}

    @Override
    public void runOpMode() {
        elevator = new Motor("elevator", DcMotor.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            teleOpControls();
            checkElevator();
            elevatorTelemetry();
        }
    }

    public void checkElevator() {
        telemetry.addLine(String.valueOf(elevator.drive.getCurrentPosition()));
        telemetry.update();
        switch (elevatorState) {
            case MANUAL_UP -> elevator.drive.setPower(elevatorPower);
            case MANUAL_DOWN -> elevator.drive.setPower(-elevatorPower);
            case HOLD -> {
                int difference = elevator.target - elevator.drive.getCurrentPosition();
                if (Math.abs(difference) > tolerance) {
                    elevator.drive.setPower(0.02*((double) difference/Math.abs(difference)));
                } else {
                    elevator.drive.setPower(0);
                }
            }
            case UP -> {
                if (Math.abs(elevator.drive.getCurrentPosition() - elevator.target) <= tolerance) {
                    elevator.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    elevatorState = HOLD;
                }
                else elevator.drive.setPower(elevatorPower);
            }
            case DOWN -> {
                if (Math.abs(elevator.drive.getCurrentPosition() - elevator.target) <= tolerance) {
                    elevatorState = STOP;
                    elevator.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
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
            if (elevatorState==DOWN) {
                elevator.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            elevator.target = elevatorMaxPosition;
            elevatorState = UP;
        }
        else if (gamepad1.y) {
            if (elevatorState != HOLD) {
                elevator.target = elevator.drive.getCurrentPosition();
                elevator.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    void elevatorTelemetry() {
        telemetry.addData("Encoder position", elevator.drive.getCurrentPosition());
        telemetry.addData("Target", elevator.target);
        telemetry.update();
    }
    public class Motor {
        DcMotor drive;
        int target = 0;
        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
