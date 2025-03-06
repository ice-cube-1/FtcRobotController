package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="wheel test", group="Linear OpMode")
@Disabled
public class wheelTest extends LinearOpMode {
    private Motor[] motors = null;
    class Motor {
        DcMotor drive;
        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    @Override
    public void runOpMode() {
        motors = new Motor[]{new Motor("left_front_drive", DcMotor.Direction.REVERSE),
                new Motor("right_front_drive", DcMotor.Direction.FORWARD),
                new Motor("left_back_drive", DcMotor.Direction.REVERSE),
                new Motor("right_back_drive", DcMotor.Direction.FORWARD)};
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_stick_x < -0.5) {
                motors[0].drive.setPower(0.2);
            } else {
                motors[0].drive.setPower(0);
            }
            if (gamepad1.left_stick_x > 0.5) {
                motors[1].drive.setPower(0.2);
            } else {
                motors[1].drive.setPower(0);
            }
            if (gamepad1.right_stick_x < -0.5) {
                motors[2].drive.setPower(0.2);
            } else {
                motors[2].drive.setPower(0);
            }
            if (gamepad1.right_stick_x > 0.5) {
                motors[3].drive.setPower(0.2);
            } else {
                motors[3].drive.setPower(0);
            }
        }
    }
}
