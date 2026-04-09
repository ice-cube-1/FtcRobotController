package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER
import org.firstinspires.ftc.teamcode.Constants.STOP_DOWN
import org.firstinspires.ftc.teamcode.Constants.STOP_UP

enum class IntakeStates {INTAKE_ON, SHOOTING, IDLE}

class TransferIntake(hardwareMap: HardwareMap) {
    private val intake = hardwareMap.get(DcMotorEx::class.java, "intake").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    private val transfer = hardwareMap.get(DcMotorEx::class.java, "transfer").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }
    private val stop = hardwareMap.get(Servo::class.java, "stop").apply {
        position = STOP_DOWN
    }
    private var intakeState = IntakeStates.IDLE
    fun prepShooter() {
        stop.position = STOP_UP
    }
    fun shoot(s : Boolean) {
        if (s) {
            stop.position = STOP_UP
            intakeState = IntakeStates.SHOOTING
        } else {
            stop.position = STOP_DOWN
            intakeState = IntakeStates.IDLE
        }
    }
    fun intake(i: Boolean) {
        if (i) {
            stop.position = STOP_DOWN
            intakeState = IntakeStates.INTAKE_ON
        } else {
            stop.position = STOP_DOWN
            intakeState = IntakeStates.IDLE
        }
    }
    fun update() {
        when (intakeState) {
            IntakeStates.INTAKE_ON -> {
                intake.power = INTAKE_POWER
                transfer.power = 0.0
            } IntakeStates.SHOOTING -> {
                intake.power = INTAKE_POWER
                transfer.power = INTAKE_POWER
            }
            IntakeStates.IDLE -> {
                intake.power = 0.0
                transfer.power = 0.0
            }
        }
    }
}