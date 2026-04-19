package org.firstinspires.ftc.teamcode.config

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robotParts.Constants.POWER_MAX
import org.firstinspires.ftc.teamcode.robotParts.OdometryDrivetrain
import kotlin.math.PI

@TeleOp(name = "rotate Test (Odometry)")
class RotateTest : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    private val timer = ElapsedTime()
    override fun runOpMode() {
        drivetrain = OdometryDrivetrain(hardwareMap, 0.0,0.0,0.0, FtcDashboard.getInstance().telemetry)
        waitForStart()
        while (opModeIsActive()) {
        if (gamepad1.left_bumper) {
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
        drivetrain.driveManual(0.0, 0.0, 0.0,POWER_MAX, false)
    } }}
}

@TeleOp(name = "Square & rotate Test (Odometry)")
class SquareTest : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    private val timer = ElapsedTime()
    override fun runOpMode() {
        drivetrain = OdometryDrivetrain(hardwareMap, 0.0,0.0,0.0, FtcDashboard.getInstance().telemetry)
        waitForStart()
        val points = arrayOf(Pair(0.0,40.0),Pair(40.0,40.0),Pair(40.0,0.0),Pair(0.0,0.0))
        while (opModeIsActive()) {
        if (gamepad1.left_bumper) {
        for (i in 0..<4) {
            drivetrain.updateGoto(points[i].first,points[i].second, PI/2*(i+1))
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
        drivetrain.driveManual(0.0, 0.0, 0.0,POWER_MAX, false)
    }}}
}

@TeleOp(name = "diagonal square Test (Odometry)")
class DiagonalSquareTest : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    private val timer = ElapsedTime()
    override fun runOpMode() {
        drivetrain = OdometryDrivetrain(hardwareMap, 0.0,0.0,PI/4, FtcDashboard.getInstance().telemetry)
        waitForStart()
        val points = arrayOf(Pair(0.0,0.0),Pair(0.0,40.0),Pair(40.0,40.0),Pair(40.0,0.0),Pair(0.0,0.0))
        while (opModeIsActive()) {
        if (gamepad1.left_bumper) {
        for (i in 0..4) {
            drivetrain.updateGoto(points[i].first,points[i].second, PI/4)
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
        drivetrain.driveManual(0.0, 0.0, 0.0,POWER_MAX, false)
    } } }
}
