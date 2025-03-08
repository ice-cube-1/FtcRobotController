package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="wheel test", group="Linear OpMode")
@Disabled
public class motor_distance_test extends LinearOpMode {
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
        motors = new Motor[]{new Motor("left_elevator", DcMotor.Direction.REVERSE)};
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_stick_x < -0.5) {
                motors[0].drive.setPower(0.2);
            } else if (gamepad1.left_stick_x > 0.5) {
                motors[0].drive.setPower(-0.2);
            } else {
                motors[0].drive.setPower(0);
            }
            telemetry.addData("encoder pos", motors[0].drive.getCurrentPosition());
            telemetry.update();
        }
    }
}
