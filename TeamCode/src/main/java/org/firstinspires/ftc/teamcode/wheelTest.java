package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="wheel test", group="Linear OpMode")
public class wheelTest extends LinearOpMode {
    private Motor motor = null;
    private double speed = 0.2;
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
        motor = new Motor("motor", DcMotor.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motor.drive.setPower(speed);
            } else if (gamepad1.dpad_down) {
                motor.drive.setPower(-speed);
            } else motor.drive.setPower(0);
        }
    }
}
