package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants.MANUAL_MULTIPLIER

@TeleOp
class BaseManual : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val intake = Intake(hardwareMap)
        val drivetrain = DriveTrain(hardwareMap,telemetry)
        /**
        val spindexer = Spindexer(hardwareMap)
        val shooter = Shooter(hardwareMap, telemetry)
        **/
        while (opModeIsActive()) {
            /** expected field centric control **/
            drivetrain.driveManual(gamepad1.left_stick_x * MANUAL_MULTIPLIER,
                -gamepad1.left_stick_y * MANUAL_MULTIPLIER, -gamepad1.right_stick_x)
            /**
            /** spindexer: kickarm is gamepad 2 a, left stick rotates spindex **/
            if (gamepad2.a) { spindexer.kick() }
            spindexer.manualRotate(gamepad2.left_stick_x * 0.2f)
            /** shooter: right stick rotates turret, right trigger turns shooter on, dpad moves back **/
            shooter.manual(
                gamepad2.right_stick_x * 0.5,
                if (gamepad2.dpad_up) 50F else if (gamepad2.dpad_down) -50F else 0F,
                gamepad2.right_trigger > 0.5
            )
             **/
            /** intake: b toggles on on gamepad 1**/
            intake.on = gamepad1.b
            intake.run()
        }
    }
}