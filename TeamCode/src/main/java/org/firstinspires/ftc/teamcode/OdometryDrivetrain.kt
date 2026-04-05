package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KD_HEADING
import org.firstinspires.ftc.teamcode.Constants.KD_TRANSLATION
import org.firstinspires.ftc.teamcode.Constants.KP_HEADING
import org.firstinspires.ftc.teamcode.Constants.KP_TRANSLATION
import org.firstinspires.ftc.teamcode.Constants.MANUAL_MULTIPLIER
import org.firstinspires.ftc.teamcode.Constants.ODOMETRY_TICKS_PER_CM
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin


enum class Wheels {LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK}
enum class Odometry(private val direction: Int, private var prev: Double = 0.0, var delta: Double = 0.0) {
    LEFT(direction = -1) {
        override fun x() = -10.7
        override fun y() = -3.5
    },
    RIGHT(direction = -1) {
        override fun x() = 10.7
        override fun y() = -3.5
    },
    CENTER(direction = 1) {
        override fun x() = -18.0
        override fun y() = -20.0
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

class OdometryDrivetrain (private val hardwareMap: HardwareMap) {
    private lateinit var wheels: Array<DcMotorEx>
    private val left = Odometry.LEFT
    private val right = Odometry.RIGHT
    private val center = Odometry.CENTER
    private var deltaX = 0.0
    private var deltaY = 0.0
    private var deltaTheta = 0.0
    private var x = 0.0
    private var y = 0.0
    private var targetX = 0.0
    private var targetY = 0.0
    private var targetTheta = 0.0
    private var theta = 0.0
    private val timer = ElapsedTime()
    private var lastTime = 0.0
    init {
        wheels = arrayOf(
            initOdoWheel("lf", Direction.REVERSE),
            initOdoWheel("rf",Direction.REVERSE),
            initOdoWheel("lb", Direction.FORWARD),
            initOdoWheel("rb", Direction.FORWARD)
        )
        left.reset()
        right.reset()
        center.reset()
        timer.reset()
    }
    private fun driveManual(moveX: Double, moveY: Double, turn: Double) {
        val xRobot = cos(theta) * moveX - sin(theta) * moveY
        val yRobot = sin(theta) * moveX + cos(theta) * moveY
        val denominator = max(0.1, abs(yRobot) + abs(xRobot) + abs(turn)) / MANUAL_MULTIPLIER
        getWheel(Wheels.LEFT_FRONT).power = (yRobot + xRobot + turn) / denominator
        getWheel(Wheels.RIGHT_FRONT).power = (yRobot - xRobot - turn) / denominator
        getWheel(Wheels.LEFT_BACK).power = (yRobot - xRobot + turn) / denominator
        getWheel(Wheels.RIGHT_BACK).power = (yRobot + xRobot - turn) / denominator
    }
    private fun updateGoto(newX: Double, newY: Double, newTheta: Double) {
        targetX = newX
        targetY = newY
        targetTheta = newTheta
    }
    private fun continueDriving() : Boolean {
        val currentTime = timer.milliseconds()
        val deltaTime = (currentTime - lastTime)
        lastTime = currentTime
        var headingError = targetTheta - theta
        while (headingError > PI) headingError -= 2*PI
        while (headingError < -PI) headingError += 2*PI
        val xComp = (KP_TRANSLATION * (targetX-x)) + (KD_TRANSLATION * deltaX / deltaTime)
        val yComp = (KP_TRANSLATION * (targetY-y)) + (KD_TRANSLATION * deltaY / deltaTime)
        val turnComp = (KP_HEADING * headingError) + (KD_HEADING * deltaTheta / deltaTime)
        driveManual(xComp, yComp, turnComp)
        return abs(targetY-y) < 2 && abs(targetX-x) < 2 && abs(headingError) < .1
    }

    private fun getDeltaOdometry() {
        center.getUpdateDelta(getOdometryPos(Odometry.CENTER))
        left.getUpdateDelta(getOdometryPos(Odometry.LEFT))
        right.getUpdateDelta(getOdometryPos(Odometry.RIGHT))
        deltaTheta = (left.delta - right.delta)/(right.x() - left.x())
        deltaY = (left.delta + right.delta)/2.0 - deltaTheta * (left.y() + right.y())/2.0
        deltaX = center.delta - deltaTheta * center.x()
        theta += deltaTheta
        if (theta > PI) { theta -= 2*PI}
        if (theta < -PI) { theta += 2*PI}
        x += cos(theta) * deltaX - sin(theta) * deltaY
        y += sin(theta) * deltaX + cos(theta) * deltaY
    }

    private fun getWheel(name: Wheels) : DcMotorEx {
        return when (name) {
            Wheels.LEFT_FRONT -> wheels[0]
            Wheels.RIGHT_FRONT -> wheels[1]
            Wheels.LEFT_BACK -> wheels[2]
            Wheels.RIGHT_BACK -> wheels[3]
        }
    }
    private fun getOdometryPos(name: Odometry): Int {
        return when (name) {
            Odometry.LEFT -> wheels[2].currentPosition
            Odometry.RIGHT -> wheels[1].currentPosition
            Odometry.CENTER -> wheels[0].currentPosition
        }
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