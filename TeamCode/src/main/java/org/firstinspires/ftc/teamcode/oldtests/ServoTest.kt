package org.firstinspires.ftc.teamcode.oldtests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE

@TeleOp(name = "servo test", group = "Linear OpMode")
@Disabled
class ServoTest : LinearOpMode() {
    override fun runOpMode() {
        val s = hardwareMap.get(Servo::class.java, "s")
        waitForStart()
        s.position = 0.5
        while (opModeIsActive()) {
            if (gamepad1.a) {s.position = KICKARM_RELEASE }
            else if (gamepad1.b) { s.position= KICKARM_DOWN }
        }
    }
}