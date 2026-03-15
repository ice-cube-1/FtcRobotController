package org.firstinspires.ftc.teamcode.oldtests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE
import org.firstinspires.ftc.teamcode.Intake
import org.firstinspires.ftc.teamcode.Spindexer

@TeleOp
class TestIntakeSpindexer : LinearOpMode() {
    private val timer = ElapsedTime()
    private lateinit var spindexer: Spindexer
    private lateinit var intake: Intake
    private var timeToEnd = 0.0
    override fun runOpMode() {
        intake = Intake(hardwareMap)
        spindexer = Spindexer(hardwareMap)
        waitForStart()
        spindexer.emptyIntake()
        timer.reset()
        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                spindexer.release()
                timeToEnd = timer.milliseconds()+500
            }
            intake.on = timer.milliseconds() > timeToEnd && (gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5)
            if (spindexer.detect() && timer.milliseconds() > timeToEnd) {
                timeToEnd = timer.milliseconds()+1500
                spindexer.emptyIntake()
            }
            /** TODO: Add shooter ready check **/
            if (gamepad1.x && timer.milliseconds() > timeToEnd) {
                release()
            }
            getTelemetry()
        }
    }
    private fun release() {
        /** blocking as robot should not move during release process **/
        while (timer.milliseconds() < timeToEnd && opModeIsActive()) {}
        spindexer.kickarm.position = KICKARM_RELEASE
        timeToEnd = timer.milliseconds()+800
        while (timer.milliseconds() < timeToEnd && opModeIsActive()) {}
        spindexer.kickarm.position = KICKARM_DOWN
        timeToEnd = timer.milliseconds()+1000
        while (timer.milliseconds() < timeToEnd && opModeIsActive()) {}
    }
    private fun getTelemetry() {
        telemetry.addLine("-----SPINDEXER-----")
        telemetry.addLine(spindexer.getData())
        telemetry.addLine("-----INTAKE-----")
        telemetry.addLine(intake.getData())
        telemetry.update()
    }
}