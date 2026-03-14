package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = DriveTrain(hardwareMap,telemetry,0.0,0.0,0.0)
        waitForStart()
        while (opModeIsActive()) {
            drivetrain.updateRotation()

            if (gamepad1.a) {
                drivetrain.startDrive(36.0,0.0)
                while (!drivetrain.updateDrive()) {}
                drivetrain.stop()
                drivetrain.targetAngle = 90.0
                while (!drivetrain.updateRotation()) {}
                drivetrain.stop()
                drivetrain.startDrive(36.0,36.0)
                while (!drivetrain.updateDrive()) {}
                drivetrain.stop()
                drivetrain.targetAngle = 45.0
                while (!drivetrain.updateRotation()) {}
                drivetrain.stop()
                drivetrain.startDrive(0.0,36.0)
                while (!drivetrain.updateDrive()) {}
                drivetrain.stop()
                drivetrain.targetAngle = 0.0
                while (!drivetrain.updateRotation()) {}
                drivetrain.stop()
                drivetrain.startDrive(0.0,0.0)
                while (!drivetrain.updateDrive()) {}
                drivetrain.stop()
                while (!drivetrain.updateRotation()) {}
                drivetrain.stop()
            }
        }
    }
}