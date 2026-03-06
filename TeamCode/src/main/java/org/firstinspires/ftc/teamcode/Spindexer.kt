package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Constants.Companion.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.Companion.KICKARM_RELEASE
import java.lang.Thread.sleep

class Spindexer (hardwareMap: HardwareMap, sensing: Boolean = false) {
    private lateinit var distanceSensor: DistanceSensor
    private val positions = arrayOf(false, false, false)
    private val spindex = hardwareMap.get(Servo::class.java, "spindex")
    private val kickarm = hardwareMap.get(Servo::class.java, "kickarm")
    private var currentPos = Constants.Companion.SpinPosition.ZERO_IN
    private var hasDetected = false
    private val detectionTime = ElapsedTime()
    init {
        if (sensing) { distanceSensor = hardwareMap.get(DistanceSensor::class.java, "distance_sensor"); }
    }
    fun detect(): Boolean {
        val distance = distanceSensor.getDistance(DistanceUnit.MM)
        if (hasDetected) {
            if (distance > 10) { hasDetected = false }
            else if (detectionTime.milliseconds() > 10) {
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

    fun release(): Boolean {
        val gotoPos = if (true in positions) {
            positions.indexOf(true)
        } else { return false }
        currentPos = when (gotoPos) {
            0 -> Constants.Companion.SpinPosition.ZERO_OUT
            1 -> Constants.Companion.SpinPosition.ONE_OUT
            else -> Constants.Companion.SpinPosition.TWO_OUT
        }
        spindex.position = currentPos.pos()
        sleep(200)
        kick()
        return true
    }
    fun kick() {
        kickarm.position = KICKARM_RELEASE
        sleep(200)
        kickarm.position = KICKARM_DOWN
        sleep(200)
    }
    fun manualRotate(delta: Float) {
        spindex.position += delta
    }
    fun emptyIntake(): Boolean {
        val gotoPos = when (false) {
            in positions -> { positions.indexOf(false) }
            else -> { return false }
        }
        currentPos = when (gotoPos) {
            0 -> Constants.Companion.SpinPosition.ZERO_IN
            1 -> Constants.Companion.SpinPosition.ONE_IN
            else -> Constants.Companion.SpinPosition.TWO_IN
        }
        spindex.position = currentPos.pos()
        sleep(200)
        return true
    }
}