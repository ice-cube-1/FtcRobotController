package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Spindexer (hardwareMap: HardwareMap) {
    private val positions = arrayOf(false, false, false)
    private val spindex = hardwareMap.get(ServoImplEx::class.java, "spindex").apply {
        pwmRange = PwmControl.PwmRange(500.0, 2500.0)
    }
    val kickarm: Servo = hardwareMap.get(Servo::class.java, "kickarm")
    private var currentPos = Constants.SpinPosition.ZERO_IN
    private var hasDetected = false
    private val detectionTime = ElapsedTime()
    private val distanceSensor = hardwareMap.get(DistanceSensor::class.java, "distance_sensor")
    fun detect(): Boolean {
        val distance = distanceSensor.getDistance(DistanceUnit.MM)
        if (hasDetected) {
            if (distance > 120) { hasDetected = false }
            else if (detectionTime.milliseconds() > 200) {
                positions[currentPos.value()] = true
                hasDetected = false
                return true
            }
        } else {
            if (distance < 120) {
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
    fun setALl(to: Boolean) { positions[0] = to; positions[1] = to; positions[2] = to }
    fun moveKickarm(position: Double) { kickarm.position = position }
    fun rotateSpindexer(distance: Double) {
        var newPos = spindex.position + distance
        if (newPos < 0) newPos += 1.2
        if (newPos > 1) newPos -= 1.2
        spindex.position = newPos
        currentPos = Constants.SpinPosition.fromPos(spindex.position)
    }
    fun getData(): String {
        return "Current position: ${currentPos}\n ${positions.count{it}} artefacts\n Positions $positions\n${spindex.position}"
    }
}