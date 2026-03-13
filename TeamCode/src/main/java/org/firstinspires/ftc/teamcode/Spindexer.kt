package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Spindexer (hardwareMap: HardwareMap) {
    private val positions = arrayOf(false, false, false)
    private val spindex = hardwareMap.get(Servo::class.java, "spindex")
    val kickarm: Servo = hardwareMap.get(Servo::class.java, "kickarm")
    private var currentPos = Constants.SpinPosition.ZERO_IN
    private var hasDetected = false
    private val detectionTime = ElapsedTime()
    private val distanceSensor = hardwareMap.get(DistanceSensor::class.java, "distance_sensor")
    fun detect(): Boolean {
        val distance = distanceSensor.getDistance(DistanceUnit.MM)
        if (hasDetected) {
            if (distance > 10) { hasDetected = false }
            else if (detectionTime.milliseconds() > 50) {
                positions[currentPos.value()] = true
                return true
            }
        } else {
            if (distance < 10) {
                hasDetected = true
                detectionTime.reset()
            }
        }
        return false
    }
    /** kickarm controlled in OpMode as cannot block shooter spinning **/
    fun release(position: Int = -1): Boolean {
        val gotoPos = if (position == -1) {
            if (true in positions) { positions.indexOf(true) } else { return false }
        } else position
        currentPos = when (gotoPos) {
            0 -> Constants.SpinPosition.ZERO_OUT
            1 -> Constants.SpinPosition.ONE_OUT
            else -> Constants.SpinPosition.TWO_OUT
        }
        spindex.position = currentPos.pos()
        return true
    }
    fun removeItem() { positions[currentPos.value()] = false }
    /** testing purposes only **/
    fun manualRotate(delta: Float) { spindex.position += delta }
    /** get ready to intake **/
    fun emptyIntake(): Boolean {
        val gotoPos = when (false) {
            in positions -> { positions.indexOf(false) }
            else -> { return false }
        }
        currentPos = when (gotoPos) {
            0 -> Constants.SpinPosition.ZERO_IN
            1 -> Constants.SpinPosition.ONE_IN
            else -> Constants.SpinPosition.TWO_IN
        }
        spindex.position = currentPos.pos()
        return true
    }
}