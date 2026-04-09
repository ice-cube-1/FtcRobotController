package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.Constants.TURRET_KP
import org.firstinspires.ftc.teamcode.Constants.TURRET_STEP
import kotlin.math.max
import kotlin.math.min

@TeleOp
class servoTest : LinearOpMode() {
    public override fun runOpMode() {
        waitForStart()
        val turret = arrayOf(
            hardwareMap.get(ServoImplEx::class.java, "t1").apply {
                pwmRange = PwmControl.PwmRange(500.0, 2500.0)
            },
            hardwareMap.get(ServoImplEx::class.java, "t2").apply {
                pwmRange = PwmControl.PwmRange(500.0, 2500.0)
            }
        )
        sleep(500)
        var currentPos = (turret[0].position + turret[0].position)/2.0
        while (opModeIsActive()) {
            currentPos = if (TURRET_KP > turret[0].position) {
                min(TURRET_KP, currentPos + TURRET_STEP)
            } else {
                max(TURRET_KP, currentPos - TURRET_STEP)
            }
            telemetry.addData("position",currentPos)
            telemetry.addData("0",turret[0].position)
            telemetry.addData("1",turret[1].position)
            telemetry.update()
            turret[0].position = currentPos
            turret[1].position = currentPos
        }
    }

}