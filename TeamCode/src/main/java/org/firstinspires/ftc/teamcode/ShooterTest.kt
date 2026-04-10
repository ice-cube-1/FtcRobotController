package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class ShooterTest : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val shooter = ShooterNew(hardwareMap,20)
        while (opModeIsActive()) {
            shooter.moveTurret()
            telemetry.addLine(shooter.getData())
            telemetry.update()
        }
    }
}