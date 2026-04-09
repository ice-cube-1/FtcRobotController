package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class ShooterTest : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val shooter = ShooterNew(hardwareMap,20)
        shooter.turnOnShooter()
        while (opModeIsActive()) {
            shooter.spin()
            telemetry.addLine(shooter.getData())
            telemetry.update()
        }
    }
}