package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous
class AutoLeaveFar : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = DriveTrain(hardwareMap)
        waitForStart()
        drivetrain.updateRotation()
        drivetrain.startDrive(0.0,30.0)
        while (!drivetrain.updateDrive()) {}
        drivetrain.stop()
    }
}