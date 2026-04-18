package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robotParts.Shooter

@TeleOp
class ShooterTest : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val shooter = Shooter(hardwareMap,20)
        while (opModeIsActive()) {
            shooter.moveTurret()
            telemetry.addLine(shooter.getData())
            telemetry.update()
        }
    }
}