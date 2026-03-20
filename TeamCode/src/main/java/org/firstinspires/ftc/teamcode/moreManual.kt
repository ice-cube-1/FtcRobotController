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
 * A/B -> shooter ON/OFF
 * DPAD LEFT/RIGHT -> SPIN
 * DPAD UP/DOWN -> Kickarm
 * Triggers -> intake (forwards / back)
 **/

@TeleOp
class MoreManual : LinearOpMode() {
    private val timer = ElapsedTime()
    private lateinit var spindexer: Spindexer
    private lateinit var intake: Intake
    private lateinit var driveTrain: DriveTrain
    private lateinit var shooter: Shooter
    private var timeToEnd = 0.0
    private var robotState = RobotStateNew.INTAKE
    override fun runOpMode() {
        var initialised = false
        telemetry.addLine("Dpad LEFT for BLUE alliance, RIGHT for RED")
        telemetry.update()
        driveTrain = DriveTrain(hardwareMap)
        shooter = Shooter(hardwareMap)
        while (!initialised && opModeInInit()) {
            if (gamepad1.dpad_left) {
                /** BLUE !!! **/
                driveTrain.setStart(0.0, 0.0, 90.0)
                shooter.setStart(45.0, 20)
                initialised = true
                telemetry.addLine("BLUE ALLIANCE")
            }
            if (gamepad1.dpad_right) {
                /** RED !!! **/
                driveTrain.setStart(0.0, 0.0, -90.0)
                shooter.setStart(-45.0, 24)
                initialised = true
                telemetry.addLine("RED ALLIANCE")
            }
            telemetry.update()
        }
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
                if (gamepad1.y) spindexer.moveKickarm(KICKARM_RELEASE)
                if (gamepad1.a) spindexer.moveKickarm(KICKARM_DOWN)
                if (gamepad1.b) shooter.turnOnShooter()
                if (gamepad1.x) shooter.turnOffShooter()
            }
            shooter.moveTurret(driveTrain.getOrientationDeg())
            shooter.spin()
            intake.run()
            telemetry.addData("current state", robotState)
            telemetry.addLine("-----SPINDEXER------")
            telemetry.addLine(spindexer.getData())
            telemetry.addLine("-------SHOOTER------")
            telemetry.addLine(shooter.getData())
            telemetry.update()
        }
    }
}