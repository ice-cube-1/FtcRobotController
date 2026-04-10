package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.Constants.STOP_DOWN

@TeleOp
class SingleServoTest : LinearOpMode() {
    private lateinit var turret: Array<ServoImplEx>
    public override fun runOpMode() {
        waitForStart()
        val s = hardwareMap.get(ServoImplEx::class.java, "s")
        sleep(500)
        while (opModeIsActive()) {
            s.position = STOP_DOWN
        }
    }

}