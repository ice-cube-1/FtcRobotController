package org.firstinspires.ftc.teamcode.config

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction

@TeleOp
class OdometryWheelsConfig : LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        val wheels = arrayOf(
            initOdoWheel("lf", Direction.REVERSE),
            initOdoWheel("rf", Direction.FORWARD),
            initOdoWheel("lb", Direction.REVERSE),
            initOdoWheel("rb", Direction.FORWARD)
        )
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0.5) {
                wheels[0].power = 0.5
            } else wheels[0].power = 0.0
            if (gamepad1.right_trigger > 0.5) {
                wheels[1].power = 0.5
            } else wheels[1].power = 0.0
            if (gamepad1.left_bumper) {
                wheels[2].power = 0.5
            } else wheels[2].power = 0.0
            if (gamepad1.right_bumper) {
                wheels[3].power = 0.5
            } else wheels[3].power = 0.0
            telemetry.addLine("LT - LF, RT - RF, LB - LB, RB - RB")
            telemetry.addData("LF encoder", wheels[0].currentPosition)
            telemetry.addData("RF encoder", wheels[1].currentPosition)
            telemetry.addData("LB encoder", wheels[2].currentPosition)
            telemetry.addData("RB encoder", wheels[3].currentPosition)
            telemetry.update()

        }
    }
    private fun initOdoWheel(name: String, direction: Direction): DcMotorEx {
        return hardwareMap.get(DcMotorEx::class.java, name).apply {
            setDirection(direction)
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

}