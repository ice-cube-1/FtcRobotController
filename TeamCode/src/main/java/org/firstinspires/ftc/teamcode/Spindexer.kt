package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Constants.Companion.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.Companion.KICKARM_RELEASE
import java.lang.Thread.sleep


enum class Detected {
    NONE { override fun opposite() = NONE },
    GREEN { override fun opposite() = PURPLE },
    PURPLE { override fun opposite() = GREEN };
    abstract fun opposite(): Detected
}

class Spindexer (hardwareMap: HardwareMap) {
    private val colorSensor: RevColorSensorV3 = hardwareMap.get(RevColorSensorV3::class.java, "color_sensor");
    private val positions = arrayOf(Detected.NONE, Detected.NONE, Detected.NONE)
    private val spindex = hardwareMap.get(Servo::class.java, "spindex")
    private val kickarm = hardwareMap.get(Servo::class.java, "kickarm")
    private var currentPos = Constants.Companion.SpinPosition.ZERO_IN
    fun detect() {
        val colors = colorSensor.normalizedColors
        if (colors.red > 200.0 && colors.blue > 200.0 && colors.green < 200.0) { positions[currentPos.value()] = Detected.PURPLE }
        if (colors.green > 200.0 && colors.blue < 200.0 && colors.red < 200.0) { positions[currentPos.value()] = Detected.GREEN }
    }
    fun release(nextColor: Detected): Boolean {
        val gotoPos = if (nextColor in positions) {
            positions.indexOf(nextColor)
        } else if (nextColor.opposite() in positions)  {
            positions.indexOf(nextColor)
        } else { return false }
        currentPos = when (gotoPos) {
            0 -> Constants.Companion.SpinPosition.ZERO_OUT
            1 -> Constants.Companion.SpinPosition.ONE_OUT
            else -> Constants.Companion.SpinPosition.TWO_OUT
        }
        spindex.position = currentPos.pos()
        sleep(200)
        kickarm.position = KICKARM_RELEASE
        sleep(200)
        kickarm.position = KICKARM_DOWN
        sleep(200)
        return true
    }
    fun emptyIntake(): Boolean {
        val gotoPos = when (Detected.NONE) {
            in positions -> { positions.indexOf(Detected.NONE) }
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