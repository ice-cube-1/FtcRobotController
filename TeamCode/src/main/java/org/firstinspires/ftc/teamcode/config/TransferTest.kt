package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robotParts.Constants.STOP_DOWN
import org.firstinspires.ftc.teamcode.robotParts.Constants.STOP_UP
import org.firstinspires.ftc.teamcode.robotParts.TransferIntake

@TeleOp
class TransferTest : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val transferIntake = TransferIntake(hardwareMap)
        while (opModeIsActive()) {
            transferIntake.intake.power = (gamepad1.left_trigger - gamepad1.right_trigger).toDouble()
            transferIntake.transfer.power = (gamepad1.left_stick_x).toDouble()
            if (gamepad1.dpad_up) transferIntake.stop.position = STOP_UP
            if (gamepad1.dpad_down) transferIntake.stop.position = STOP_DOWN
        }
    }
}