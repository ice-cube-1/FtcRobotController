package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = DriveTrain(hardwareMap,telemetry)
        waitForStart()
        while (opModeIsActive()) {
            drivetrain.startDrive(10.0,0.0)
            while (!drivetrain.updateDrive()) {}
            drivetrain.stop()
            drivetrain.updateRotation()
            while (!drivetrain.updateRotation()) {}
            drivetrain.stop()
            drivetrain.startDrive(10.0,10.0)
            while (!drivetrain.updateDrive()) {}
            drivetrain.stop()
            drivetrain.updateRotation()
            while (!drivetrain.updateRotation()) {}
            drivetrain.stop()
            drivetrain.startDrive(0.0,10.0)
            while (!drivetrain.updateDrive()) {}
            drivetrain.stop()
            drivetrain.updateRotation()
            while (!drivetrain.updateRotation()) {}
            drivetrain.stop()
            drivetrain.startDrive(0.0,0.0)
            while (!drivetrain.updateDrive()) {}
            drivetrain.stop()
            drivetrain.updateRotation()
            while (!drivetrain.updateRotation()) {}
            drivetrain.stop()
        }
    }
}