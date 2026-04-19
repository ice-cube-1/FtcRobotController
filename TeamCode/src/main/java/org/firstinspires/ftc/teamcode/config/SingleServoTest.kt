package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.robotParts.Constants.HOOD_ANGLE
import org.firstinspires.ftc.teamcode.robotParts.Constants.STOP_DOWN
import org.firstinspires.ftc.teamcode.robotParts.Constants.STOP_UP

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

@TeleOp
class CalibrateHardStop : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val s = hardwareMap.get(ServoImplEx::class.java, "stop")
        sleep(500)
        telemetry.addLine("LEFT bumper for STOP DOWN, RIGHT bumper for STOP UP")
        telemetry.update()
        var sPos = 0.0
        while (opModeIsActive()) {
            s.position = sPos
            if (gamepad1.left_bumper) {sPos = STOP_DOWN}
            if (gamepad1.right_bumper) {sPos = STOP_UP}

        }
    }
}