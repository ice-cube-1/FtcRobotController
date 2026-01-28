package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "shooter test", group = "Linear OpMode")
class shootertest : LinearOpMode() {
    fun init_motor(name: String?, direction: DcMotorSimple.Direction?): DcMotorEx {
        val out = hardwareMap.get(DcMotorEx::class.java, name)
        out.direction = direction
        out.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        out.mode = DcMotor.RunMode.RUN_USING_ENCODER
        out.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        return out
    }

    override fun runOpMode() {
        val motors = arrayOf(
            init_motor("m1", DcMotorSimple.Direction.FORWARD),
            init_motor("m2", DcMotorSimple.Direction.REVERSE)
        )
        val s = hardwareMap.get(CRServo::class.java, "s")
        var speed = 0.1
        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                speed = java.lang.Double.max(speed - 0.05, 0.1)
                sleep(200)
            }
            if (gamepad1.right_bumper) {
                speed = java.lang.Double.min(speed + 0.05, 1.0)
                sleep(200)
            }
            if (gamepad1.b) {
                for (motor in motors) { motor.power = speed }
            } else if (gamepad1.x || gamepad1.y) {
                for (motor in motors) { motor.power = -speed }
            } else {
                for (motor in motors) { motor.power = 0.0 }
            }
            if (gamepad1.dpad_up) { s.power = -0.2
            } else if (gamepad1.dpad_down) { s.power = 0.2
            } else { s.power = 0.0 }
            telemetry.addLine(speed.toString())
            for (motor in motors) { telemetry.addLine(motor.velocity.toString()) }
            telemetry.update()
        }
    }
}