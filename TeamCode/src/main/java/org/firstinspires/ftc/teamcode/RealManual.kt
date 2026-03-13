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
    private var timeToEnd = 0.0
    private var robotState = RobotState.IDLE
    override fun runOpMode() {
        intake = Intake(hardwareMap)
        spindexer = Spindexer(hardwareMap)
        driveTrain = DriveTrain(hardwareMap,telemetry,0.0,0.0,0.0)
        waitForStart()
        timer.reset()
        while (opModeIsActive()) {
            /** expected field centric control **/
            driveTrain.driveManual(gamepad1.left_stick_x * MANUAL_MULTIPLIER,
                -gamepad1.left_stick_y * MANUAL_MULTIPLIER, -gamepad1.right_stick_x)
            if (gamepad1.dpad_left && robotState == RobotState.IDLE) {
                spindexer.emptyIntake()
                robotState = RobotState.INTAKE
            }
            if (gamepad1.dpad_right) {
                spindexer.release()
                timeToEnd = timer.milliseconds()+500
                robotState = RobotState.IDLE
            }
            intake.on = robotState == RobotState.INTAKE && timer.milliseconds() > timeToEnd && (gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5)
            if (gamepad1.b && timer.milliseconds() > timeToEnd) {
                robotState = RobotState.SHOOTER_ON
                /** TODO: TURN ON SHOOTER **/
                timeToEnd = timer.milliseconds()+1000
            }
            /** TODO: Add shooter ready check **/
            if (gamepad1.x && timer.milliseconds() > timeToEnd && robotState == RobotState.SHOOTER_ON) {
                release()
            }
        }
    }
    private fun release() {
        /** blocking as robot should not move during release process **/
        spindexer.kickarm.position = KICKARM_RELEASE
        timeToEnd = timer.milliseconds()+800
        while (timer.milliseconds() < timeToEnd && opModeIsActive()) {}
        spindexer.kickarm.position = KICKARM_DOWN
        timeToEnd = timer.milliseconds()+1000
        while (timer.milliseconds() < timeToEnd && opModeIsActive()) {}
    }
}