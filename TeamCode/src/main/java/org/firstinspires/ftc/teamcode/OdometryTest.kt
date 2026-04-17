package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Odometry manual test")
class OdometryTest : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    override fun runOpMode() {
        drivetrain = OdometryDrivetrain(hardwareMap)
        waitForStart()
        while (opModeIsActive()) {
            drivetrain.continueDriving()
            telemetry.addLine(drivetrain.getData())
            telemetry.update()
        }

    }
}