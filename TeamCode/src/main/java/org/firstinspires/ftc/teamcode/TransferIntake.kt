package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER
import org.firstinspires.ftc.teamcode.Constants.SHOOT_INITIAL
import org.firstinspires.ftc.teamcode.Constants.SHOOT_MULTIPLIER
import org.firstinspires.ftc.teamcode.Constants.STOP_DOWN
import org.firstinspires.ftc.teamcode.Constants.STOP_UP
import org.firstinspires.ftc.teamcode.Constants.TRANSFER_POWER
import kotlin.math.min

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
    private var lengthEachGo = 0.0
    fun prepShooter() {
        stop.position = STOP_UP
    }
    fun shoot(s : Boolean) {
        if (s) {
            stop.position = STOP_UP
            transfer.power = 0.0
            intake.power = 0.0
            lengthEachGo = SHOOT_INITIAL
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
                val t = timer.milliseconds()
                if (t < SHOOT_INITIAL) {
                    intake.power = if (lengthEachGo <= SHOOT_INITIAL * SHOOT_MULTIPLIER) 0.0 else INTAKE_POWER
                    transfer.power = TRANSFER_POWER
                } else if (t < lengthEachGo + SHOOT_INITIAL) {
                    intake.power = INTAKE_POWER
                    transfer.power = 0.0

                } else if (t < lengthEachGo * 4) {
                    intake.power = 0.0
                    transfer.power = 0.0
                } else {
                    timer.reset()
                    lengthEachGo = min(lengthEachGo* SHOOT_MULTIPLIER, 1400.0)
                    lengthEachGo *= SHOOT_MULTIPLIER
                }
            }
            IntakeStates.INTAKE -> {
                intake.power = intakePower
                transfer.power = 0.0
            }
        }
    }
}