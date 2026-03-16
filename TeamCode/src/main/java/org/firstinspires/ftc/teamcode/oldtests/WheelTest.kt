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
            Wheel("lf", hardwareMap, DcMotorSimple.Direction.REVERSE),
            Wheel("rf", hardwareMap, DcMotorSimple.Direction.FORWARD),
            Wheel("lb", hardwareMap, DcMotorSimple.Direction.REVERSE),
            Wheel("rb", hardwareMap, DcMotorSimple.Direction.FORWARD)
        )
        waitForStart()
        while (opModeIsActive()) {
            wheels[0].setPower(gamepad1.left_trigger * 0.2)
            wheels[1].setPower(gamepad1.right_trigger * 0.2)
            wheels[2].setPower(if (gamepad1.left_bumper) 0.2 else 0.0)
            wheels[3].setPower(if (gamepad1.right_bumper) 0.2 else 0.0)
            for (i in wheels) {
                telemetry.addData("position",i.getPosition())
            }
            telemetry.update()
        }
    }
}