package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robotParts.Constants.POWER_MAX
import org.firstinspires.ftc.teamcode.robotParts.OdometryDrivetrain
import kotlin.math.PI

@TeleOp(name = "Square Test (Odometry)")
class SquareTest : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    private val timer = ElapsedTime()
    override fun runOpMode() {
        drivetrain = OdometryDrivetrain(hardwareMap, 0.0,0.0,0.0)
        waitForStart()
        for (i in 0..4) {
            drivetrain.updateGoto(0.0,0.0, PI/2*i)
            while (!drivetrain.continueDriving()) {
                telemetry.addLine(drivetrain.getData())
                telemetry.update()
            }
            timer.reset()
            while (opModeIsActive() && (timer.milliseconds() < 500 || !drivetrain.continueDriving())) {
                telemetry.addLine(drivetrain.getData())
                telemetry.update()
            }
        }
        drivetrain.driveManual(0.0, 0.0, 0.0,POWER_MAX)
    }
}