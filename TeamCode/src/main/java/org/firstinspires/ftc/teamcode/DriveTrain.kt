package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Constants.KD_HEADING
import org.firstinspires.ftc.teamcode.Constants.KP_HEADING
import org.firstinspires.ftc.teamcode.Constants.X_TICKS_PER_INCH
import org.firstinspires.ftc.teamcode.Constants.Y_TICKS_PER_INCH
import java.lang.Math.toRadians
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin


class DriveTrain (hardwareMap: HardwareMap, private var x: Double, private var y: Double, private val startAngle: Double) {
    private val wheels = arrayOf(
        Wheel("lf", hardwareMap, DcMotorSimple.Direction.REVERSE),
        Wheel("rf", hardwareMap, DcMotorSimple.Direction.FORWARD),
        Wheel("lb", hardwareMap, DcMotorSimple.Direction.REVERSE),
        Wheel("rb", hardwareMap, DcMotorSimple.Direction.FORWARD)
    )
    private val imu = hardwareMap.get(IMU::class.java, "imu").apply {
        initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        )))
        resetYaw()
    }
    private var lastHeadingError = 0.0
    private var lastHeadingTime = 0L
    var targetAngle = startAngle

    fun startDrive(newX: Double, newY: Double) {
        val deltaX = newX - x
        val deltaY = newY - y
        x = newX
        y = newY
        val xTransposed = ((deltaX * cos(toRadians(targetAngle)) - deltaY * sin(toRadians(targetAngle))) * X_TICKS_PER_INCH)
        val yTransposed = ((deltaY * cos(toRadians(targetAngle)) + deltaX * sin(toRadians((targetAngle)))) * Y_TICKS_PER_INCH)
        wheels[0].setTarget(yTransposed + xTransposed)
        wheels[1].setTarget(yTransposed - xTransposed)
        wheels[2].setTarget(yTransposed - xTransposed)
        wheels[3].setTarget(yTransposed + xTransposed)
        lastHeadingTime = System.nanoTime()
    }
    fun updateDrive(): Boolean { return update( true) }
    fun updateRotation(): Boolean { return update(false) }
    fun stop() { for (wheel in wheels) { wheel.setPower(0.0) } }
    fun driveManual(moveX: Float, moveY: Float, turn: Float) {
        val theta = toRadians(getOrientationDeg())
        val xTransposed = moveX * cos(theta) - moveY * sin(theta)
        val yTransposed = moveY * cos(theta) + moveX * sin(theta)
        wheels[0].setPower(yTransposed + xTransposed + turn)
        wheels[1].setPower(yTransposed - xTransposed - turn)
        wheels[2].setPower(yTransposed - xTransposed + turn)
        wheels[3].setPower(yTransposed + xTransposed - turn)
    }

    private fun update(toTarget: Boolean): Boolean {
        var error = targetAngle - getOrientationDeg()
        while (error > 180) error -= 360.0
        while (error <= -180) error += 360.0
        val currentTime = System.nanoTime()
        val deltaTime: Double = (currentTime - lastHeadingTime) / 1e9
        var derivative = 0.0
        if (deltaTime > 0) { derivative = (error - lastHeadingError) / deltaTime }
        val turn = (KP_HEADING * error) + (KD_HEADING * derivative)
        lastHeadingError = error
        lastHeadingTime = currentTime
        val move = booleanArrayOf(
            wheels[0].autoMove(turn, currentTime, toTarget),
            wheels[1].autoMove(-turn, currentTime, toTarget),
            wheels[2].autoMove(turn, currentTime, toTarget),
            wheels[3].autoMove(-turn, currentTime, toTarget)
        )
        val result = abs(error) < 5 && (move.any{ it } || !toTarget)
        return result
    }

    fun getData(): String {
        var out = ""
        for (i in wheels) { out += i.getPosition().toString() + "\n" }
        out += "X:$x - Y:$y"
        return out
    }
    fun getOrientationDeg(): Double {
        var theta = -imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES) + startAngle
        while (theta < -180.0) theta += 360.0
        while (theta > 180.0) theta -= 360.0
        return theta
    }
}
