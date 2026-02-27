package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name="test motor encoder",group="test")
class TestMotorEncoder : LinearOpMode() {
    override fun runOpMode() {
        val m = hardwareMap.get(DcMotor::class.java, "m")
        waitForStart()
        var power = 0.2
        while (opModeIsActive()) {
            telemetry.addData("position",m.currentPosition)
            telemetry.addData("power", power)
            telemetry.update()
            m.power = gamepad1.left_stick_x * power
            if (gamepad1.left_bumper) {power+=0.01}
            if (gamepad1.right_bumper) {power-=0.01}

        }
    }
}