package org.firstinspires.ftc.teamcode.oldtests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "motor encoder", group = "Linear OpMode")
@Disabled
class SingleMotorEncoder : LinearOpMode() {
    override fun runOpMode() {
        val m = hardwareMap.get(DcMotor::class.java, "m").apply {
            direction = DcMotorSimple.Direction.FORWARD
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }
        waitForStart()
        while (opModeIsActive()) {
            m.power = (gamepad1.left_trigger - gamepad1.right_trigger)*0.2
            telemetry.addData("pos",m.currentPosition)
            telemetry.update()
        }
    }
}