package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = DriveTrain(hardwareMap)
        waitForStart()
        while (opModeIsActive()) {
            drivetrain.updateTargets(2.0, 2.0)
            while (!drivetrain.update()) {}
            drivetrain.updateTargets(-2.0,-2.0)
            while (!drivetrain.update()) {}
        }
    }
}