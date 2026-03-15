package org.firstinspires.ftc.teamcode.oldtests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.DriveTrain

@Autonomous
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = DriveTrain(hardwareMap,0.0,0.0,0.0)
        waitForStart()
        while (opModeIsActive()) {
            drivetrain.updateRotation()
            if (gamepad1.dpad_left) {
                drivetrain.startDrive(-60.0,0.0)
                while (!drivetrain.updateDrive()) {}
                drivetrain.stop()
                drivetrain.startDrive(.0,0.0)
                while (!drivetrain.updateDrive()) {}
                drivetrain.stop()
            }
            if (gamepad1.dpad_up) {
                drivetrain.startDrive(0.0,60.0)
                while (!drivetrain.updateDrive()) {}
                drivetrain.stop()
                drivetrain.startDrive(.0,0.0)
                while (!drivetrain.updateDrive()) {}
                drivetrain.stop()
            }
        }
    }
}