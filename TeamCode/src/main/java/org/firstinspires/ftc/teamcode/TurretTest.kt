package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.Constants.TURRET_KP
import org.firstinspires.ftc.teamcode.Constants.TURRET_MAX_DEGREES
import org.firstinspires.ftc.teamcode.Constants.TURRET_STEP
import org.firstinspires.ftc.teamcode.Constants.TURRET_ZERO_DEG
import kotlin.math.max
import kotlin.math.min

@TeleOp
class TurretTest : LinearOpMode() {
    private lateinit var turret: Array<ServoImplEx>
    override fun runOpMode() {
        waitForStart()
        turret = arrayOf(
            hardwareMap.get(ServoImplEx::class.java, "t1").apply {
                pwmRange = PwmControl.PwmRange(500.0, 2500.0)
            },
            hardwareMap.get(ServoImplEx::class.java, "t2").apply {
                pwmRange = PwmControl.PwmRange(500.0, 2500.0)
            }
        )
        sleep(500)
        var currentPos = 0.5
        while (opModeIsActive()) {
            currentPos = if (TURRET_KP > currentPos) {
                min(TURRET_KP, currentPos + TURRET_STEP)
            } else {
                max(TURRET_KP, currentPos - TURRET_STEP)
            }
            telemetry.addData("position",currentPos)
            telemetry.addData("angle", getTurretAngle())
            telemetry.addData("0",turret[0].position)
            telemetry.addData("1",turret[1].position)
            telemetry.update()
            turret[0].position = currentPos
            turret[1].position = currentPos
        }
    }
    private fun getTurretAngle() : Double {
        return ((turret[0].position + turret[1].position) / 2.0) * TURRET_MAX_DEGREES - TURRET_ZERO_DEG
    }
}