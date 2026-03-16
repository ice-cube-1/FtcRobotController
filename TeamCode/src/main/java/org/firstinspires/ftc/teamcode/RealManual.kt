package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE
import org.firstinspires.ftc.teamcode.Constants.MANUAL_MULTIPLIER

enum class RobotState {INTAKE, IDLE, SHOOTER_ON}

/**
 * CONTROLS:
 * LEFT / RIGHT joystick -> movement
 * DPAD left -> to INTAKE
 * DPAD right -> to IDLE
 * B -> shooter ON/OFF
 * X -> SHOOT
 * Triggers -> intake
 **/

@TeleOp
class RealManual : LinearOpMode() {
    private val timer = ElapsedTime()
    private lateinit var spindexer: Spindexer
    private lateinit var intake: Intake
    private lateinit var driveTrain: DriveTrain
    private lateinit var shooter: Shooter
    private var timeToEnd = 0.0
    private var robotState = RobotState.IDLE
    override fun runOpMode() {
        var initialised = false
        telemetry.addLine("Dpad LEFT for BLUE alliance, RIGHT for RED")
        telemetry.update()
        while (!initialised && opModeInInit()) {
            if (gamepad1.dpad_left) { /** BLUE !!! **/
                driveTrain = DriveTrain(hardwareMap,0.0,0.0,-90.0)
                shooter = Shooter(hardwareMap, 45.0)
                initialised = true
                telemetry.addLine("BLUE ALLIANCE")
            }
            if (gamepad1.dpad_right) { /** RED !!! **/
                driveTrain = DriveTrain(hardwareMap,0.0,0.0,90.0)
                shooter = Shooter(hardwareMap, -45.0)
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
            if (gamepad1.dpad_left && robotState != RobotState.INTAKE) {
                timeToEnd = timer.milliseconds()+500
                spindexer.emptyIntake()
                shooter.turnOffShooter()
                robotState = RobotState.INTAKE
            }
            if (gamepad1.dpad_right) {
                spindexer.release()
                shooter.turnOffShooter()
                timeToEnd = timer.milliseconds()+500
                robotState = RobotState.IDLE
            }
            intake.on = robotState == RobotState.INTAKE && (gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5)
            if (robotState == RobotState.INTAKE && spindexer.detect() && timer.milliseconds() > timeToEnd) {
                timeToEnd = timer.milliseconds()+1000
                if (!spindexer.emptyIntake()) {
                    spindexer.release()
                    robotState = RobotState.IDLE
                }
            }
            if (gamepad1.b && timer.milliseconds() > timeToEnd) {
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
            update()
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
        timeToEnd = timer.milliseconds()+800
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