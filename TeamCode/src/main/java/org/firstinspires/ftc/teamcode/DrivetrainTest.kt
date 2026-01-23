package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = DriveTrain(hardwareMap,telemetry)
        waitForStart()
        drivetrain.updateTargets(0.0, 0.0)
        while (opModeIsActive()) { drivetrain.update() }
    }
}