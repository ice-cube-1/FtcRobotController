package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE
import org.firstinspires.ftc.teamcode.Constants.MANUAL_MULTIPLIER

enum class RobotStateNew {INTAKE, CAN_SHOOT}

/**
 * CONTROLS:
 * LEFT / RIGHT joystick -> movement
 * LEFT BUMPER to enter INTAKE mode
 * RIGHT BUMPER to enter CAN_SHOOT mode
 * B/X -> shooter ON/OFF
 * DPAD LEFT/RIGHT -> SPIN
 * Y/A -> Kickarm
 * Triggers -> intake (forwards / back)
 **/

@TeleOp
class ManualRed : LinearOpMode() {
    private val timer = ElapsedTime()
    private lateinit var spindexer: Spindexer
    private lateinit var intake: Intake
    private lateinit var driveTrain: DriveTrain
    private lateinit var shooter: Shooter
    private var timeToEnd = 0.0
    private var robotState = RobotStateNew.INTAKE
    override fun runOpMode() {
        driveTrain = DriveTrain(hardwareMap)
        shooter = Shooter(hardwareMap)
        driveTrain.setStart(0.0, 0.0, -90.0)
        shooter.setStart(24, true)
        intake = Intake(hardwareMap)
        spindexer = Spindexer(hardwareMap)
        waitForStart()
        timer.reset()
        timeToEnd = timer.milliseconds() + 500
        spindexer.emptyIntake()
        while (opModeIsActive()) {
            /** expected field centric control **/
            driveTrain.driveManual(
                gamepad1.left_stick_x * MANUAL_MULTIPLIER,
                -gamepad1.left_stick_y * MANUAL_MULTIPLIER, gamepad1.right_stick_x
            )
            /** swaps between states **/
            if (gamepad1.left_bumper) {
                timeToEnd = timer.milliseconds() + 500
                spindexer.setALl(false)
                spindexer.emptyIntake()
                shooter.turnOffShooter()
                robotState = RobotStateNew.INTAKE
                spindexer.setIO(true)
            }
            if (gamepad1.right_bumper) {
                spindexer.release()
                shooter.turnOffShooter()
                robotState = RobotStateNew.CAN_SHOOT
                spindexer.setIO(false)
            }
            /** manually rotate spindexer -> can do anytime **/
            if (gamepad1.dpad_left && timeToEnd < timer.milliseconds()) {
                spindexer.rotateSpindexer(-0.4)
                timeToEnd = timer.milliseconds() + 200
            }
            if (gamepad1.dpad_right && timeToEnd < timer.milliseconds()) {
                spindexer.rotateSpindexer(0.4)
                timeToEnd = timer.milliseconds() + 200
            }
            /** intake & detection **/
            if (robotState == RobotStateNew.INTAKE) {
                if (gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5) {
                    intake.on = true
                    intake.reversed = gamepad1.right_trigger > 0.5
                } else intake.on = false
                if (spindexer.detect() && timer.milliseconds() > timeToEnd) {
                    timeToEnd = timer.milliseconds() + 1000
                    if (!spindexer.emptyIntake()) {
                        spindexer.release()
                    }
                }
            }
            /** shooting logic **/
            if (robotState == RobotStateNew.CAN_SHOOT) {
                if (gamepad1.y) release()
                if (gamepad1.b) shooter.turnOnShooter()
                if (gamepad1.x) shooter.turnOffShooter()
            }
            update()
        }
    }
    fun update() {
        shooter.moveTurret()
        shooter.spin()
        intake.run()
        telemetry.addData("current state", robotState)
        telemetry.addLine("-----SPINDEXER------")
        telemetry.addLine(spindexer.getData())
        telemetry.addLine("-------SHOOTER------")
        telemetry.addLine(shooter.getData())
        telemetry.addLine("-----DRIVETRAIN-----")
        telemetry.addLine(driveTrain.getData())
        telemetry.update()
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
}