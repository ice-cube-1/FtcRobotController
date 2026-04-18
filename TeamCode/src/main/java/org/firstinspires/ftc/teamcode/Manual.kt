package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robotParts.Constants.MANUAL_MULTIPLIER
import org.firstinspires.ftc.teamcode.robotParts.OdometryDrivetrain
import org.firstinspires.ftc.teamcode.robotParts.Shooter
import org.firstinspires.ftc.teamcode.robotParts.TransferIntake
import kotlin.math.abs

enum class RobotState {INTAKE, SHOOTER_SPIN_UP, SHOOTER_ON}

/**
 * CONTROLS:
 * LEFT / RIGHT joystick -> movement
 * LEFT BUMPER to go into SHOOTER mode (SPIN UP) - only do this if you want to shoot from there
 * RIGHT BUMPER to enter INTAKE mode
 * Triggers -> intake (forwards / back)
 * DPAD UP/DOWN to move the goal 10cm FURTHER/NEARER (virtually)
 **/

abstract class Manual(private val orientation: Double, private val tag: Int) : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    private lateinit var shooter: Shooter
    private lateinit var transferIntake: TransferIntake
    private var robotState = RobotState.INTAKE
    private var timeToEnd = 0.0
    private val timer = ElapsedTime()
    private val atSpeed = ElapsedTime()
    private val dpadTimer = ElapsedTime()
    override fun runOpMode() {
        waitForStart()
        drivetrain = OdometryDrivetrain(hardwareMap, 0.0,0.0,orientation)
        shooter = Shooter(hardwareMap, tag)
        transferIntake = TransferIntake(hardwareMap)
        while (opModeIsActive()) {
            /** expected field centric control **/
            drivetrain.driveManual(
                gamepad1.left_stick_x*abs(gamepad1.left_stick_x).toDouble(),
                -gamepad1.left_stick_y*abs(gamepad1.left_stick_y).toDouble(),
                gamepad1.right_stick_x*abs(gamepad1.right_stick_x).toDouble(), MANUAL_MULTIPLIER
            )
            if (gamepad1.left_bumper && timer.milliseconds() > timeToEnd) {
                robotState = RobotState.SHOOTER_SPIN_UP
                transferIntake.prepShooter()
                shooter.turnOnShooter()
                timeToEnd = timer.milliseconds() + 500
            }
            if (gamepad1.right_bumper && timer.milliseconds() > timeToEnd) {
                robotState = RobotState.INTAKE
                transferIntake.shoot(false)
                shooter.turnOffShooter()
                timeToEnd = timer.milliseconds() + 200
            }
            if (gamepad1.dpad_up && dpadTimer.milliseconds() > 200) {
                shooter.turretOffset += 0.1
                dpadTimer.reset()
            }
            if (gamepad1.dpad_down && dpadTimer.milliseconds() > 200) {
                shooter.turretOffset -= 0.1
                dpadTimer.reset()
            }
            if (robotState == RobotState.INTAKE) { transferIntake.intake(gamepad1.left_trigger - gamepad1.right_trigger) }
            if (!shooter.atSpeed) atSpeed.reset()
            if (atSpeed.milliseconds() > 500 && (robotState == RobotState.SHOOTER_SPIN_UP) && shooter.canShoot() && timer.milliseconds() > timeToEnd) {
                robotState = RobotState.SHOOTER_ON
                transferIntake.shoot(true)
            }
            transferIntake.update()
            shooter.moveTurret()
            shooter.spin()
            telemetry.addLine(shooter.getData())
            telemetry.addLine(transferIntake.getData())
            telemetry.addLine(drivetrain.getData())
            telemetry.update()
        }
    }
}