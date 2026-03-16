package org.firstinspires.ftc.teamcode.oldtests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "spindex test", group = "Linear OpMode")
@Disabled
class SpindexTest : LinearOpMode() {
    override fun runOpMode() {
        val distanceSensor = hardwareMap.get(DistanceSensor::class.java, "distance_sensor")
        val s = hardwareMap.get(ServoImplEx::class.java, "s").apply {
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        }
        val intake = hardwareMap.get(DcMotor::class.java, "intake").apply {
            direction = DcMotorSimple.Direction.FORWARD
        }
        var pos = 0.0
        val timer = ElapsedTime()
        var timeToEnd = 0.0
        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.left_bumper && timeToEnd < timer.milliseconds()) {
                pos += 1.0/5.0
                timeToEnd = timer.milliseconds()+200
            }
            if (gamepad1.right_bumper && timeToEnd < timer.milliseconds()) {
                pos -= 1.0/5.0
                timeToEnd = timer.milliseconds()+200
            }
            intake.power = if (gamepad1.a) 1.0 else 0.0
            s.position = pos
            telemetry.addData("distance",distanceSensor.getDistance(DistanceUnit.MM))
            telemetry.addData("servo", s.position)
            telemetry.addData("pos", pos)
            telemetry.update()
        }
    }
}