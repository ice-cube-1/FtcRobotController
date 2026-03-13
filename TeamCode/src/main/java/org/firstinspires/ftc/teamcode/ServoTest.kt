package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "servo test", group = "Linear OpMode")
class ServoTest : LinearOpMode() {
    override fun runOpMode() {
        val s = hardwareMap.get(Servo::class.java, "s")
        waitForStart()
        while (opModeIsActive()) {
            s.position += (gamepad1.left_trigger - gamepad1.right_trigger)*0.005
            telemetry.addData("servo", s.position)
            telemetry.update()
        }
    }
}