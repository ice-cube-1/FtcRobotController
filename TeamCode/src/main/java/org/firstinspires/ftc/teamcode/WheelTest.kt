package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx

@Autonomous
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val s = hardwareMap.get(ServoImplEx::class.java, "s").apply {
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        }

        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                s.position = 0.0
            }
            if (gamepad1.dpad_down) s.position = 1.0/4.0
            if (gamepad1.dpad_right) s.position = 2.0/4.0
            if (gamepad1.left_bumper) s.position = 3.0/4.0
            if (gamepad1.dpad_up) { s.position = 1.0 }
        }
    }
}