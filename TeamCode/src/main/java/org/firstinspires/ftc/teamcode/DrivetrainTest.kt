package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous
class DrivetrainTest : LinearOpMode() {
    override fun runOpMode() {
        val drivetrain = DriveTrain(hardwareMap,telemetry)
        waitForStart()
        while (opModeIsActive()) {
            drivetrain.targetAngle += gamepad1.left_stick_x
            telemetry.addData("angle",drivetrain.targetAngle)
            drivetrain.updateRotation()
        }
    }
}