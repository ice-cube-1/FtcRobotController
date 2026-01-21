package org.firstinspires.ftc.teamcode

import Wheel
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Constants.Companion.KD_HEADING
import org.firstinspires.ftc.teamcode.Constants.Companion.KP_HEADING
import org.firstinspires.ftc.teamcode.Constants.Companion.X_TICKS_PER_INCH
import org.firstinspires.ftc.teamcode.Constants.Companion.Y_TICKS_PER_INCH
import java.lang.Math.toRadians
import kotlin.math.cos
import kotlin.math.sin


class DriveTrain (hardwareMap: HardwareMap) {
    private val wheels = arrayOf(
        Wheel("lf", hardwareMap, DcMotorSimple.Direction.FORWARD),
        Wheel("rf", hardwareMap, DcMotorSimple.Direction.REVERSE),
        Wheel("lb", hardwareMap, DcMotorSimple.Direction.FORWARD),
        Wheel("rb", hardwareMap, DcMotorSimple.Direction.REVERSE))
    private val imu = hardwareMap.get(IMU::class.java, "imu").apply {
        initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )))
    }
    var targetAngle = 0.0
    private var x = 0.0
    private var y = 0.0
    private var lastHeadingError = 0.0
    private var lastHeadingTime = 0L

    fun updateTargets(newX: Double, newY: Double) {
        val deltaX = newX - x
        val deltaY = newY - y
        val xTransposed = (deltaX * cos(toRadians(targetAngle)) - deltaY * sin(toRadians(targetAngle))) * X_TICKS_PER_INCH
        val yTransposed = (deltaY * cos(toRadians(targetAngle)) + deltaX * sin(toRadians((targetAngle)))) * Y_TICKS_PER_INCH
        wheels[0].target += yTransposed + xTransposed
        wheels[1].target += yTransposed - xTransposed
        wheels[2].target += yTransposed - xTransposed
        wheels[3].target += yTransposed + xTransposed
        x = newX
        y = newY
    }

    fun update(): Boolean {
        val turn = turnPower()
        return wheels[0].move(turn) && wheels[1].move(-turn) && wheels[2].move(turn) && wheels[3].move(-turn)
    }

    private fun turnPower(): Double {
        val currentAngle: Double = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
        var error = targetAngle - currentAngle
        while (error > 180) error -= 360.0
        while (error <= -180) error += 360.0
        val currentTime = System.nanoTime()
        val deltaTime: Double = (currentTime - lastHeadingTime) / 1e9
        var derivative = 0.0
        if (deltaTime > 0) { derivative = (error - lastHeadingError) / deltaTime }
        val output = (KP_HEADING * error) + (KD_HEADING * derivative)
        lastHeadingError = error
        lastHeadingTime = currentTime
        return output
    }
}
