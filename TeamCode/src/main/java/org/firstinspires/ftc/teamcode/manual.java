package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;

@TeleOp(name="Manual", group="Linear OpMode")
public class manual extends LinearOpMode {

    Motor[] motors;
    Motor[] elevators;
    Motor box;
    Motor arm;
    int box_position = 0;

    @Override
    public void runOpMode() {
        motors = new Motor[]{
                new Motor("left_front_drive", DcMotor.Direction.REVERSE),
                new Motor("left_back_drive", DcMotor.Direction.REVERSE),
                new Motor("right_front_drive", DcMotor.Direction.FORWARD),
                new Motor("right_back_drive", DcMotor.Direction.FORWARD),
        };
        elevators = new Motor[]{
                new Motor("left_elevator", DcMotor.Direction.REVERSE),
                new Motor("right_elevator", DcMotor.Direction.FORWARD)
        };
        box = new Motor("box", DcMotor.Direction.FORWARD);
        arm = new Motor("arm", DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;
            /*
            if (gamepad1.dpad_up) {
                for (Motor elevator: elevators) {
                    elevator.power = elevator.position - elevator.drive.getCurrentPosition();
                    elevator.drive.setPower(0.25);
                }
            } else if (gamepad1.dpad_down) {
                for (Motor elevator: elevators) {
                    elevator.drive.setPower(-0.25);
                }
            } else {
                for (Motor elevator: elevators) {
                    elevator.drive.setPower(0);
                }
            }

            double leftElevatorPosition = elevators[0].drive.getCurrentPosition();
            double rightElevatorPosition = elevators[1].drive.getCurrentPosition();
            double positionDifference = leftElevatorPosition - rightElevatorPosition;
            double kP = 0.001; // Proportional gain

            elevators[0].drive.setPower(elevators[0].drive.getPower() - kP * positionDifference);
            elevators[1].drive.setPower(elevators[1].drive.getPower() + kP * positionDifference);

            double trigger = gamepad1.left_trigger;
            if (trigger == 1 || trigger == 0) {
                box_position = (int) trigger;
                box.drive.setPower(0);
            } else {
                box.drive.setPower(0.25 * (box_position - trigger) / Math.abs(box_position - trigger));
            }

            if (gamepad1.a) {
                arm.drive.setPower(0.25);
            } if (gamepad1.b) {
                arm.drive.setPower(-0.25);
            } else {
                arm.drive.setPower(0);
            }

             */


            motors[0].power = axial + yaw + lateral;
            motors[1].power = axial - yaw + lateral;
            motors[2].power = axial - yaw - lateral;
            motors[3].power = axial + yaw - lateral;

            double max = Arrays.stream(motors).mapToDouble(motor -> Math.abs(motor.power)).max().orElse(1);
            if (max > 1.0) {
                for (Motor motor : motors) {
                    motor.power /= max;
                    motor.drive.setPower(motor.power);
                }
            }
        }
    }

    class Motor {
        DcMotor drive;
        double power;
        String name;
        double position;

        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.name = name;
        }
    }
}
