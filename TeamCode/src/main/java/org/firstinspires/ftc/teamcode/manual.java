package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@TeleOp(name="Manual", group="Linear OpMode")
public class manual extends LinearOpMode {

    Motor[] motors;
    Motor[] elevators;
    Motor box;
    Motor arm;

    @Override
    public void runOpMode() {
        this.init_hardware();
        waitForStart();
        while (opModeIsActive()) {
            this.move_elevators();
            this.move_drivetrain();
            this.move_box();
            this.move_arm();
            this.updateTelemetry();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("arm pos", arm.drive.getCurrentPosition());
        telemetry.addData("elevator pos", elevators[0].drive.getCurrentPosition());
        telemetry.update();
    }

    void move_arm() {
        if (gamepad1.a) {
            arm.drive.setPower(1);
        } else if (gamepad1.b) {
            arm.drive.setPower(-1);
        } else {
            arm.drive.setPower(gamepad1.left_trigger);
        }
    }


    void move_box() {
        if (gamepad1.left_bumper) {
            box.drive.setPower(0.7);
        } else if (gamepad1.right_bumper) {
            box.drive.setPower(-0.7);
        } else {
            box.drive.setPower(0);
        }
    }

    void move_elevators() {
        if (gamepad1.dpad_up) {
            for (Motor elevator: elevators) {
                elevator.drive.setPower(0.4);
            }
        } else if (gamepad1.dpad_down) {
            for (Motor elevator: elevators) {
                elevator.drive.setPower(-0.4);
            }
        } else {
            for (Motor elevator: elevators) {
                elevator.drive.setPower(0.001);
            }
        }
        for (Motor elevator : elevators) {
            elevator.position = elevator.drive.getCurrentPosition();
        }
        double position_difference = elevators[0].position - elevators[1].position;
        double gain = 0.001;

        elevators[0].drive.setPower(elevators[0].drive.getPower() - gain * position_difference);
        elevators[1].drive.setPower(elevators[1].drive.getPower() + gain * position_difference);
    }

    void move_drivetrain() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.right_stick_x;
        double yaw = gamepad1.left_stick_x;

        motors[0].power = axial + yaw + lateral;
        motors[1].power = axial - yaw + lateral;
        motors[2].power = axial - yaw - lateral;
        motors[3].power = axial + yaw - lateral;

        double max = Arrays.stream(motors).mapToDouble(motor -> Math.abs(motor.power)).max().orElse(1);
        if (max > 1.0) {
            for (Motor motor : motors) {
                motor.power /= max;
            }
        }
        for (Motor motor : motors) {
            motor.drive.setPower(motor.power);
        }
    }

    void init_hardware() {
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
        box.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    class Motor {
        DcMotor drive;
        double power;
        double position;

        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
