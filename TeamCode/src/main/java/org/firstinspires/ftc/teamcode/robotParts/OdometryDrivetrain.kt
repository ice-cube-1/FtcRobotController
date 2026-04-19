package org.firstinspires.ftc.teamcode.robotParts

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robotParts.Constants.KD_HEADING
import org.firstinspires.ftc.teamcode.robotParts.Constants.KD_TRANSLATION
import org.firstinspires.ftc.teamcode.robotParts.Constants.KP_HEADING
import org.firstinspires.ftc.teamcode.robotParts.Constants.KP_TRANSLATION
import org.firstinspires.ftc.teamcode.robotParts.Constants.KS_MANUAL
import org.firstinspires.ftc.teamcode.robotParts.Constants.K_S
import org.firstinspires.ftc.teamcode.robotParts.Constants.ODOMETRY_TICKS_PER_CM
import org.firstinspires.ftc.teamcode.robotParts.Constants.POWER_DELTA
import org.firstinspires.ftc.teamcode.robotParts.Constants.POWER_MAX
import org.firstinspires.ftc.teamcode.robotParts.Constants.xDisp
import org.firstinspires.ftc.teamcode.robotParts.Constants.yDisp
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt


enum class Wheels {LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK}
enum class Odometry(private val direction: Int, private var prev: Double = 0.0, var delta: Double = 0.0) {
    LEFT(direction = -1) {
        override fun x() = -xDisp
        override fun y() = 0.0
    },
    RIGHT(direction = -1) {
        override fun x() = xDisp
        override fun y() = 0.0
    },
    CENTER(direction = -1) {
        override fun x() = -18.0
        override fun y() = yDisp
    };
    abstract fun x(): Double
    abstract fun y(): Double
    fun getUpdateDelta(newValue: Int) {
        val value = newValue / ODOMETRY_TICKS_PER_CM
        delta = (value - prev) * direction
        prev = value
    }
    fun reset() {
        delta = 0.0
        prev = 0.0
    }
}

class OdometryDrivetrain (private val hardwareMap: HardwareMap,
                          private var x: Double,
                          private var y: Double,
                          private var theta: Double) {
    private var wheels: Array<DcMotorEx>
    private val left = Odometry.LEFT
    private val right = Odometry.RIGHT
    private val center = Odometry.CENTER
    private var deltaX = 0.0
    private var deltaY = 0.0
    private var deltaTheta = 0.0
    private var xNoRotation = 0.0
    private var yNoRotation = 0.0
    private var targetX = 0.0
    private var targetY = 0.0
    private var targetTheta = 0.0
    private val timer = ElapsedTime()
    private var lastTime = 0.0
    private var maxPower = 1.0
    private var deltaTime = 0.0
    init {
        wheels = arrayOf(
            initOdoWheel("lf", Direction.REVERSE),
            initOdoWheel("rf",Direction.FORWARD),
            initOdoWheel("lb", Direction.REVERSE),
            initOdoWheel("rb", Direction.FORWARD)
        )
        left.reset()
        right.reset()
        center.reset()
        timer.reset()
    }
    fun driveManual(moveX: Double, moveY: Double, turn: Double, powerMaximum: Double, manual: Boolean) {
        getDeltaOdometry()
        val xRobot = cos(theta) * moveX - sin(theta) * moveY
        val yRobot = sin(theta) * moveX + cos(theta) * moveY
        maxPower = min(maxPower + deltaTime * POWER_DELTA, powerMaximum)
        val attemptPower = abs(yRobot) + abs(xRobot) + abs(turn)
        val denominator = if (attemptPower > maxPower) attemptPower / maxPower else 1.0
        setWheelPower(Wheels.LEFT_FRONT, (yRobot + xRobot + turn) / denominator, manual)
        setWheelPower(Wheels.RIGHT_FRONT, (yRobot - xRobot - turn) / denominator, manual)
        setWheelPower(Wheels.LEFT_BACK, (yRobot - xRobot + turn) / denominator, manual)
        setWheelPower(Wheels.RIGHT_BACK, (yRobot + xRobot - turn) / denominator, manual)
    }
    fun updateGoto(newX: Double, newY: Double, newTheta: Double) {
        targetX = newX
        targetY = newY
        targetTheta = newTheta
        maxPower = 0.0
    }
    fun continueDriving() : Boolean {
        val currentTime = timer.milliseconds()
        deltaTime = (currentTime - lastTime)
        lastTime = currentTime
        var headingError = targetTheta - theta
        while (headingError > PI) headingError -= 2*PI
        while (headingError < -PI) headingError += 2*PI
        val xError = targetX - x
        val yError = targetY - y
        val totalError = sqrt(xError * xError + yError * yError)
        if (totalError < 2.5 && abs(headingError) < 0.02) {
            driveManual(0.0, 0.0, 0.0, POWER_MAX, false)
            return true
        }
        val xComp = (KP_TRANSLATION * (xError)) + (KD_TRANSLATION * deltaX / deltaTime)
        val yComp = (KP_TRANSLATION * (yError)) + (KD_TRANSLATION * deltaY / deltaTime)
        val turnComp = (KP_HEADING * headingError) + (KD_HEADING * deltaTheta / deltaTime)
        driveManual(xComp, yComp, turnComp, POWER_MAX, false)
        return false
    }

    private fun getDeltaOdometry() {
        center.getUpdateDelta(getOdometryPos(Odometry.CENTER))
        left.getUpdateDelta(getOdometryPos(Odometry.LEFT))
        right.getUpdateDelta(getOdometryPos(Odometry.RIGHT))
        deltaTheta = (left.delta - right.delta)/(right.x() - left.x())
        deltaY = (left.delta + right.delta)/2.0
        deltaX = center.delta - deltaTheta * center.y()
        theta += deltaTheta
        if (theta > PI) { theta -= 2*PI}
        if (theta < -PI) { theta += 2*PI}
        xNoRotation += deltaX
        yNoRotation += deltaY
        x += cos(theta) * deltaX + sin(theta) * deltaY
        y += -sin(theta) * deltaX + cos(theta) * deltaY
    }

    private fun setWheelPower(name: Wheels, power: Double, manual: Boolean) {
        var wheelPower = power
        if (abs(power) > 0.01) {
            if (power > 0) wheelPower += if (manual) KS_MANUAL else K_S
            else wheelPower -= if (manual) KS_MANUAL else K_S
        }
        when (name) {
            Wheels.LEFT_FRONT -> wheels[0].power = wheelPower
            Wheels.RIGHT_FRONT -> wheels[1].power = wheelPower
            Wheels.LEFT_BACK -> wheels[2].power = wheelPower
            Wheels.RIGHT_BACK -> wheels[3].power = wheelPower
        }
    }
    private fun getOdometryPos(name: Odometry): Int {
        return when (name) {
            Odometry.LEFT -> wheels[0].currentPosition
            Odometry.RIGHT -> wheels[1].currentPosition
            Odometry.CENTER -> wheels[2].currentPosition
        }
    }
    fun getData(): String {
        return "X: $x\n Y: $y\n, theta: $theta\n"
    }
    private fun initOdoWheel(name: String, direction: Direction): DcMotorEx {
        return hardwareMap.get(DcMotorEx::class.java, name).apply {
            setDirection(direction)
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
}