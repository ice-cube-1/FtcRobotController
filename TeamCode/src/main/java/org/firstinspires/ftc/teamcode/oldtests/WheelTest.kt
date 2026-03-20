package org.firstinspires.ftc.teamcode.oldtests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Wheel

@TeleOp(name = "wheel test", group = "Linear OpMode")
@Disabled
class WheelTest : LinearOpMode() {
    override fun runOpMode() {
        val wheels = arrayOf(
            Wheel("m1", hardwareMap, DcMotorSimple.Direction.REVERSE),
            Wheel("m2", hardwareMap, DcMotorSimple.Direction.FORWARD),
        )
        waitForStart()
        while (opModeIsActive()) {
            wheels[0].setPower(gamepad1.left_trigger * 0.1)
            wheels[1].setPower(gamepad1.right_trigger * 0.1)
            for (i in wheels) {
                telemetry.addData("position",i.getPosition())
            }
            telemetry.update()
        }
    }
}