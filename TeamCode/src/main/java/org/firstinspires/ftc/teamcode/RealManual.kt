package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE
import org.firstinspires.ftc.teamcode.Constants.MANUAL_MULTIPLIER

enum class RobotState {INTAKE, IDLE, SHOOTER_ON, OVERRIDDEN}
enum class DoubleTap {OFF, FIRST_ON, MED_OFF, FINAL_ON}

/**
 * CONTROLS:
 * LEFT / RIGHT joystick -> movement
 * DPAD left -> to INTAKE mode
 * DPAD right -> to IDLE mode
 * B -> shooter ON/OFF
 * X -> SHOOT
 * Triggers -> intake (forwards / back)
 *
 * Left bumper double tap to enter OVERRIDE mode
 * sets all positions to containing balls (must then be "shot")
 * spindexer can be moved manually by 60 degrees using dpad left / right
 * kickarm can manually be moved to top / bottom using dpad up / down
 * double tap again (but on right bumper) to go back into IDLE mode (sets to "zero out")
 * turret, intake & drivetrain moves as normal throughout all
 **/

@TeleOp
class RealManual : LinearOpMode() {
    private val timer = ElapsedTime()
    private lateinit var spindexer: Spindexer
    private lateinit var intake: Intake
    private lateinit var driveTrain: DriveTrain
    private lateinit var shooter: ShooterUnconfigured
    private var timeToEnd = 0.0
    private var robotState = RobotState.IDLE
    private var doubleTapButton = DoubleTap.OFF
    private val doubleTapTimer = ElapsedTime()
    override fun runOpMode() {
        var initialised = false
        telemetry.addLine("Dpad LEFT for BLUE alliance, RIGHT for RED")
        telemetry.update()
        driveTrain = DriveTrain(hardwareMap)
        shooter = ShooterUnconfigured(hardwareMap)
        while (!initialised && opModeInInit()) {
            if (gamepad1.dpad_left) { /** BLUE !!! **/
                driveTrain.setStart(0.0,0.0,90.0)
                shooter.setStart(45.0)
                initialised = true
                telemetry.addLine("BLUE ALLIANCE")
            }
            if (gamepad1.dpad_right) { /** RED !!! **/
                driveTrain.setStart(0.0,0.0,-90.0)
                shooter.setStart(-45.0)
                initialised = true
                telemetry.addLine("RED ALLIANCE")
            }
            telemetry.update()
        }
        intake = Intake(hardwareMap)
        spindexer = Spindexer(hardwareMap)
        waitForStart()
        timer.reset()
        while (opModeIsActive()) {
            /** expected field centric control **/
            driveTrain.driveManual(gamepad1.left_stick_x * MANUAL_MULTIPLIER,
                -gamepad1.left_stick_y * MANUAL_MULTIPLIER, gamepad1.right_stick_x)
            if (gamepad1.dpad_left && robotState != RobotState.INTAKE && robotState != RobotState.OVERRIDDEN) {
                timeToEnd = timer.milliseconds()+500
                spindexer.emptyIntake()
                shooter.turnOffShooter()
                robotState = RobotState.INTAKE
            }
            if (gamepad1.dpad_right && robotState != RobotState.OVERRIDDEN) {
                spindexer.release()
                shooter.turnOffShooter()
                timeToEnd = timer.milliseconds()+500
                robotState = RobotState.IDLE
            }
            if (gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5) {
                intake.on = true
                intake.reversed = gamepad1.right_trigger > 0.5
            } else intake.on = false
            if (robotState == RobotState.INTAKE && spindexer.detect() && timer.milliseconds() > timeToEnd) {
                timeToEnd = timer.milliseconds()+1000
                if (!spindexer.emptyIntake()) {
                    spindexer.release()
                    robotState = RobotState.IDLE
                }
            }
            if (gamepad1.b && timer.milliseconds() > timeToEnd && robotState == RobotState.IDLE) {
                robotState = RobotState.SHOOTER_ON
                shooter.turnOnShooter()
                timeToEnd = timer.milliseconds()+2000
            }
            if (gamepad1.x && timer.milliseconds() > timeToEnd && robotState == RobotState.SHOOTER_ON && shooter.canShoot()) {
                release()
                spindexer.removeItem()
                timeToEnd = timer.milliseconds()+500
                spindexer.release()
            }
            if (robotState == RobotState.OVERRIDDEN) {
                doubleTap(gamepad1.right_bumper)
                if (doubleTapButton == DoubleTap.FINAL_ON) {
                    doubleTapButton = DoubleTap.OFF
                    robotState = RobotState.IDLE
                    spindexer.release()
                }
            } else {
                doubleTap(gamepad1.left_bumper)
                if (doubleTapButton == DoubleTap.FINAL_ON) {
                    robotState = RobotState.OVERRIDDEN
                    doubleTapButton = DoubleTap.OFF
                    spindexer.setALl(true)
                }
            }
            if (robotState == RobotState.OVERRIDDEN) {
                if (gamepad1.dpad_left) spindexer.rotateSpindexer(-0.2)
                if (gamepad1.dpad_right) spindexer.rotateSpindexer(0.2)
                if (gamepad1.dpad_up) spindexer.moveKickarm(KICKARM_RELEASE)
                if (gamepad1.dpad_down) spindexer.moveKickarm(KICKARM_DOWN)
            }
            update()
        }
    }

    private fun doubleTap(value: Boolean) {
        doubleTapButton = when (doubleTapButton) {
            DoubleTap.OFF -> {
                if (value) {
                    DoubleTap.FIRST_ON
                } else { DoubleTap.OFF }
            }
            DoubleTap.FIRST_ON -> {
                if (!value) {
                    doubleTapTimer.reset()
                    DoubleTap.MED_OFF
                } else { DoubleTap.FIRST_ON }
            }
            DoubleTap.MED_OFF -> {
                when {
                    value && doubleTapTimer.milliseconds() < 250 -> { DoubleTap.FINAL_ON }
                    doubleTapTimer.milliseconds() >= 250 -> { DoubleTap.OFF }
                    else -> { DoubleTap.MED_OFF }
                }
            }
            DoubleTap.FINAL_ON -> DoubleTap.OFF
        }
    }
    private fun update() {
        shooter.moveTurret(driveTrain.getOrientationDeg())
        shooter.spin()
        intake.run()
        getTelemetry()
    }
    private fun release() {
        /** blocking as robot should not move during release process **/
        while (timer.milliseconds() < timeToEnd && opModeIsActive()) { update() }
        spindexer.kickarm.position = KICKARM_RELEASE
        timeToEnd = timer.milliseconds()+3000
        while (timer.milliseconds() < timeToEnd && opModeIsActive()) { update() }
        spindexer.kickarm.position = KICKARM_DOWN
        timeToEnd = timer.milliseconds()+1000
        while (timer.milliseconds() < timeToEnd && opModeIsActive()) { update() }
    }
    private fun getTelemetry() {
        telemetry.addData("current state", robotState)
        telemetry.addLine("-----DRIVETRAIN-----")
        telemetry.addLine(driveTrain.getData())
        telemetry.addLine("-----SPINDEXER------")
        telemetry.addLine(spindexer.getData())
        telemetry.addLine("-------INTAKE-------")
        telemetry.addLine(intake.getData())
        telemetry.addLine("-------SHOOTER------")
        telemetry.addLine(shooter.getData())
        telemetry.update()
    }
}