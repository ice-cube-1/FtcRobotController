package org.firstinspires.ftc.teamcode.oldtests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants.MANUAL_MULTIPLIER
import org.firstinspires.ftc.teamcode.DriveTrain
import org.firstinspires.ftc.teamcode.Intake

@TeleOp
@Disabled
class BaseManual : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val intake = Intake(hardwareMap)
        val drivetrain = DriveTrain(hardwareMap,0.0,0.0,180.0)
        while (opModeIsActive()) {
            /** expected field centric control **/
            drivetrain.driveManual(gamepad1.left_stick_x * MANUAL_MULTIPLIER,
                -gamepad1.left_stick_y * MANUAL_MULTIPLIER, gamepad1.right_stick_x)
            intake.on = gamepad1.b
            intake.run()
        }
    }
}