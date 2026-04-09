package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants.MANUAL_MULTIPLIER

enum class RobotState {INTAKE, SHOOTER_SPIN_UP, SHOOTER_ON}

/**
 * CONTROLS:
 * LEFT / RIGHT joystick -> movement
 * LEFT BUMPER TO GO INTO SHOOTER MODE (SPIN UP)
 *
 * LEFT BUMPER to enter INTAKE mode
 * RIGHT BUMPER to enter CAN_SHOOT mode
 * B/X -> shooter ON/OFF
 * DPAD LEFT/RIGHT -> SPIN
 * Y -> Kickarm
 * Triggers -> intake (forwards / back)
 **/

@TeleOp
class Manual : LinearOpMode() {
    private lateinit var shooter: ShooterNew
    private lateinit var drivetrain: OdometryDrivetrain
    private lateinit var transferIntake: TransferIntake
    private var robotState = RobotState.INTAKE
    override fun runOpMode() {
        waitForStart()
        shooter = ShooterNew(hardwareMap, 20)
        drivetrain = OdometryDrivetrain(hardwareMap)
        transferIntake = TransferIntake(hardwareMap)
        while (opModeIsActive()) {
            /** expected field centric control **/
            drivetrain.driveManual(
                gamepad1.left_stick_x * MANUAL_MULTIPLIER,
                -gamepad1.left_stick_y * MANUAL_MULTIPLIER,
                gamepad1.right_stick_x.toDouble()
            )

        }
    }
}