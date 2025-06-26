package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo test", group="Linear OpMode")
public class servoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo test = hardwareMap.get(Servo.class, "pincer");
        waitForStart();
        while (opModeIsActive()) {
            double currentPos = test.getPosition();
            telemetry.addLine(String.valueOf(currentPos*180));
            telemetry.update();
            if (gamepad1.dpad_left) {
                test.setPosition(currentPos+0.0005);
            } else if (gamepad1.dpad_right) {
                test.setPosition(currentPos-0.0005);
            }
        }
    }
}
