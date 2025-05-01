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

    @Override
    public void runOpMode() {
        this.init_hardware();
        waitForStart();
        while (opModeIsActive()) {
            this.move_drivetrain();
        }
    }

    void move_drivetrain() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.right_stick_x;
        double yaw = gamepad1.left_stick_x;
        telemetry.update();

        motors[0].power = axial + yaw + lateral;
        motors[1].power = axial - yaw + lateral;
        motors[2].power = axial - yaw - lateral;
        motors[3].power = axial + yaw - lateral;

        double max = Arrays.stream(motors).mapToDouble(motor -> Math.abs(motor.power)).max().orElse(1);
        for (Motor motor : motors) {
            double scaledPower = motor.power;
            if (max > 1.0) {
                scaledPower /= max;
            }
            motor.drive.setPower(scaledPower*0.2);
        }
    }

    void init_hardware() {
        motors = new Motor[]{
                new Motor("left_front_drive", DcMotor.Direction.FORWARD),
                new Motor("left_back_drive", DcMotor.Direction.FORWARD),
                new Motor("right_front_drive", DcMotor.Direction.REVERSE),
                new Motor("right_back_drive", DcMotor.Direction.REVERSE),
        };
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
