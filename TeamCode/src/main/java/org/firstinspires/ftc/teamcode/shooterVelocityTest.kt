package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.max
import kotlin.math.min

@TeleOp
class ShooterVelocityTest : LinearOpMode() {
    private lateinit var motors: Array<Wheel>
    override fun runOpMode() {
        val turret = Wheel("slow",hardwareMap, DcMotorSimple.Direction.FORWARD, telemetry)
        val hoodAngle = hardwareMap.get(CRServo::class.java, "s")
        motors =  arrayOf(
            Wheel("m1", hardwareMap, DcMotorSimple.Direction.FORWARD, telemetry),
            Wheel("m2", hardwareMap, DcMotorSimple.Direction.REVERSE, telemetry)
        )
        var power = 0.0
        val timer = ElapsedTime()
        timer.reset()
        var shooterOn = false
        var targetV = 0
        while (opModeIsActive()) {
            if (gamepad1.x) { shooterOn = !shooterOn; sleep(200) }
            if (gamepad1.left_bumper) { targetV += 20; sleep(200) }
            if (gamepad1.right_bumper) { targetV -= 20; sleep(200) }
            turret.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*0.1)
            if (gamepad1.dpad_up) { hoodAngle.power = -0.2 }
            else if (gamepad1.dpad_down) { hoodAngle.power = 0.2 }
            else { hoodAngle.power = 0.0 }
            val error = targetV - motors.sumOf { it.getVelocity() / 2 }
            power = max(0.0, min(1.0, power + 0.04 * timer.seconds() * error))
            timer.reset()
            if (shooterOn) { for (m in motors) { m.setPower(power) } }
            else { for (m in motors) { m.setPower(0.0) } }
            telemetry.addData("power", power)
            telemetry.addData("gotoV", targetV)
            telemetry.update()
        }
    }
}