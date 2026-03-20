package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous
class AutoLeave : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = DriveTrain(hardwareMap)
        waitForStart()
        drivetrain.updateRotation()
        drivetrain.startDrive(0.0,-63.0)
        while (!drivetrain.updateDrive()) {}
        drivetrain.stop()
    }
}