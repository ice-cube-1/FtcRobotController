package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class BaseManual : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val drivetrain = DriveTrain(hardwareMap,telemetry)
        while (opModeIsActive()) { drivetrain.driveManual(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x) }
    }
}