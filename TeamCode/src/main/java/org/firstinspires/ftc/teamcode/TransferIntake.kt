package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER
import org.firstinspires.ftc.teamcode.Constants.STOP_DOWN
import org.firstinspires.ftc.teamcode.Constants.STOP_UP

enum class IntakeStates {INTAKE, SHOOTING}

class TransferIntake(hardwareMap: HardwareMap) {
    private var intakePower = 0.0
    val intake = hardwareMap.get(DcMotorEx::class.java, "intake").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    private val transfer = hardwareMap.get(DcMotorEx::class.java, "transfer").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }
    val stop = hardwareMap.get(Servo::class.java, "stop").apply {
        position = STOP_DOWN
    }
    private var intakeState = IntakeStates.INTAKE
    fun prepShooter() {
        stop.position = STOP_UP
    }
    fun shoot(s : Boolean) {
        if (s) {
            stop.position = STOP_UP
            transfer.power = 0.0
            intake.power = 0.0
            intakeState = IntakeStates.SHOOTING
        } else {
            stop.position = STOP_DOWN
            transfer.power = 0.0
            intake.power = 0.0
            intakeState = IntakeStates.INTAKE
        }
    }
    fun intake(i: Float) { intakePower = i.toDouble() }
    fun update() {
        when (intakeState) {
            IntakeStates.SHOOTING -> {
                intake.power = INTAKE_POWER
                transfer.power = INTAKE_POWER
            }
            IntakeStates.INTAKE -> {
                intake.power = intakePower
                transfer.power = 0.0
            }
        }
    }
}