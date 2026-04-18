package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.max
import kotlin.math.min

class Wheel(name: String, hardwareMap: HardwareMap, direction: DcMotorSimple.Direction) {
    private val drive: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name).apply {
        setDirection(direction)
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    fun setPower(power: Double) { drive.power = max(min(power,1.0),-1.0) }
    fun getVelocity(): Double { return drive.velocity }
}