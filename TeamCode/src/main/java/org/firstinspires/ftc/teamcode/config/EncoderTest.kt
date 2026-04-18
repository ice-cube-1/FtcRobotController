package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp
class EncoderTest : LinearOpMode() {
    override fun runOpMode() {
        val motors = arrayOf(
            hardwareMap.get(DcMotor::class.java, "m1").apply { zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT },
            hardwareMap.get(DcMotor::class.java, "m2").apply { zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT }
        )

        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("m1", motors[0].currentPosition)
            telemetry.addData("m2",motors[1].currentPosition)
            telemetry.update()
        }
    }
}