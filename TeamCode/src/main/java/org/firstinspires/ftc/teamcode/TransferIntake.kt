package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE

enum class IntakeStates {INTAKE_EMPTY_TOP, INTAKE_FULL_TOP, SHOOTING, IDLE}

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
        position = KICKARM_DOWN
    }
    private val distance = hardwareMap.get(DistanceSensor::class.java, "distance")
    private var hasDetected = false
    private var detectionTime = ElapsedTime()
    private var intakeState = IntakeStates.IDLE

    fun shoot(s : Boolean) {
        if (s) {
            stop.position = KICKARM_RELEASE
            intakeState = IntakeStates.SHOOTING
        } else {
            stop.position = KICKARM_DOWN
            intakeState = IntakeStates.IDLE
        }
    }
    fun intake(i: Boolean) {
        if (i) {
            stop.position = KICKARM_DOWN
            intakeState = if (hasDetected && detectionTime.milliseconds() > 200) IntakeStates.INTAKE_FULL_TOP else IntakeStates.INTAKE_EMPTY_TOP
        } else {
            stop.position = KICKARM_DOWN
            intakeState = IntakeStates.IDLE
        }
    }
    fun update() {
        detect()
        when (intakeState) {
            IntakeStates.INTAKE_EMPTY_TOP -> {
                if (hasDetected && detectionTime.milliseconds() > 200) {
                    intakeState = IntakeStates.INTAKE_FULL_TOP
                } else {
                    intake.power = INTAKE_POWER
                    transfer.power = INTAKE_POWER
                }
            } IntakeStates.INTAKE_FULL_TOP -> {
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
    private fun detect() {
        val d = distance.getDistance(DistanceUnit.MM)
        if (hasDetected && d > 120) { hasDetected = false }
        else if (d < 120) {
            hasDetected = true
            detectionTime.reset()
        }
    }
}