package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Constants.Companion.ENCODER_ERROR
import org.firstinspires.ftc.teamcode.Constants.Companion.KD_TRANSLATION
import org.firstinspires.ftc.teamcode.Constants.Companion.KP_TRANSLATION
import kotlin.math.max
import kotlin.math.min

class Wheel(private val name: String, hardwareMap: HardwareMap, direction: DcMotorSimple.Direction, private val telemetry: Telemetry) {
    val drive: DcMotor = hardwareMap.get(DcMotor::class.java, name).apply {
        setDirection(direction)
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    var target = 0.0
    private var lastError = 0.0
    private var lastTime = 0L
    fun setPower(power: Double) { drive.power = max(min(power,1.0),-1.0) }
    fun autoMove(headingError: Double, currentTime: Long, toTarget: Boolean): Boolean {
        val error = target - drive.currentPosition.toDouble()
        val deltaTime = (currentTime - lastTime) / 1e9
        var derivative = 0.0
        if (deltaTime > 0) { derivative = (error -  lastError) / deltaTime }
        lastError = error
        lastTime = currentTime
        telemetry.addData("here", toTarget)
        telemetry.addData("error", error)
        telemetry.addData("derivative", derivative)
        if (toTarget) { setPower((KP_TRANSLATION * error) + (KD_TRANSLATION * derivative) + headingError )}
        else { setPower(headingError) }
        telemetry.update()
        return ((toTarget && error < ENCODER_ERROR) || headingError < 5)
    }
}