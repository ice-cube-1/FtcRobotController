package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime


@TeleOp
class TestMotorEncoder : LinearOpMode() {
    private val timer = ElapsedTime()
    private lateinit var driveTrain: DriveTrain
    private var timeToEnd = 0.0
    override fun runOpMode() {
        driveTrain = DriveTrain(hardwareMap,telemetry,0.0,0.0,0.0)
        waitForStart()
        timer.reset()
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {driveTrain.driveManual(0.0F,0.2F,0.0F)}
            else if (gamepad1.dpad_down) {driveTrain.driveManual(0.0F,-0.2F,0.0F)}
            else if (gamepad1.dpad_left) {driveTrain.driveManual(0.2F,0.0F,0.0F)}
            else if (gamepad1.dpad_right) {driveTrain.driveManual(-0.2F,0.0F,0.0F)}
            else {driveTrain.driveManual(0.0F,0.0F,0.0F)}
            getTelemetry()
        }
    }
    private fun getTelemetry() {
        telemetry.addLine("-----DRIVETRAIN-----")
        telemetry.addLine(driveTrain.getData())
        telemetry.update()
    }
}