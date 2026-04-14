package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER
import org.firstinspires.ftc.teamcode.Constants.STOP_DOWN
import org.firstinspires.ftc.teamcode.Constants.STOP_UP
import org.firstinspires.ftc.teamcode.Constants.TRANSFER_POWER

enum class IntakeStates {INTAKE, SHOOTING}

class TransferIntake(hardwareMap: HardwareMap) {
    private var intakePower = 0.0
    val intake: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "intake").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    val transfer: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "transfer").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }
    val stop: Servo = hardwareMap.get(Servo::class.java, "stop").apply { position = STOP_DOWN }
    private val timer = ElapsedTime()
    private var intakeState = IntakeStates.INTAKE
    fun prepShooter() {
        stop.position = STOP_UP
    }
    fun shoot(s : Boolean) {
        if (s) {
            stop.position = STOP_UP
            transfer.power = 0.0
            intake.power = 0.0
            timer.reset()
            intakeState = IntakeStates.SHOOTING
        } else {
            stop.position = STOP_DOWN
            transfer.power = 0.0
            intake.power = 0.0
            intakeState = IntakeStates.INTAKE
        }
    }
    fun getData(): String {
        return "intake: "+intake.power+", transfer: "+transfer.power
    }
    fun intake(i: Float) { intakePower = i.toDouble() }
    fun update(canShoot: Boolean) {
        when (intakeState) {
            IntakeStates.SHOOTING -> {
                if (200 > timer.milliseconds()) {
                    intake.power = 0.0
                } else if (400 > timer.milliseconds() && canShoot) {
                    intake.power = INTAKE_POWER
                } else timer.reset()
                transfer.power = TRANSFER_POWER
            }
            IntakeStates.INTAKE -> {
                intake.power = intakePower
                transfer.power = 0.0
            }
        }
    }
}