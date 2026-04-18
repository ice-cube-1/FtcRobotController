package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robotParts.OdometryDrivetrain

@TeleOp(name = "Odometry manual test")
class OdometryTest : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    override fun runOpMode() {
        drivetrain = OdometryDrivetrain(hardwareMap,0.0,0.0,0.0)
        waitForStart()
        while (opModeIsActive()) {
            drivetrain.continueDriving()
            telemetry.addLine(drivetrain.getData())
            telemetry.update()
        }
    }
}