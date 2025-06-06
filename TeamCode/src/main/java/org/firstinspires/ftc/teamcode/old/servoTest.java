package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="servoTest", group="Linear OpMode")
@Disabled
public class servoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        CRServo a = hardwareMap.get(CRServo.class, "a");
        CRServo b = hardwareMap.get(CRServo.class, "b");
        CRServo c = hardwareMap.get(CRServo.class, "c");

        waitForStart();
        while (opModeIsActive()) {
            a.setPower(gamepad1.left_stick_x);
            b.setPower(-gamepad1.left_stick_x);
            c.setPower(gamepad1.right_stick_x);
        }
    }
}
