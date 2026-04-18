package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.robotParts.Constants.HOOD_ANGLE

@TeleOp
class ZeroHood : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val s = hardwareMap.get(ServoImplEx::class.java, "hood")
        sleep(500)
        telemetry.addLine("Left bumper to move down once in, right bumper to reset")
        telemetry.update()
        var sPos = 0.0
        while (opModeIsActive()) {
            s.position = sPos
            if (gamepad1.left_bumper) {sPos = HOOD_ANGLE}
            if (gamepad1.right_bumper) {sPos = 0.0}

        }
    }

}