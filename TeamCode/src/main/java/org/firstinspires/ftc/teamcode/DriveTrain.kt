package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Constants.KD_HEADING
import org.firstinspires.ftc.teamcode.Constants.KP_HEADING
import org.firstinspires.ftc.teamcode.Constants.X_TICKS_PER_INCH
import org.firstinspires.ftc.teamcode.Constants.Y_TICKS_PER_INCH
import java.lang.Math.toRadians
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin


class DriveTrain (hardwareMap: HardwareMap, private val telemetry: Telemetry,
                  private var x: Double, private var y: Double, private val startAngle: Double) {
    private val wheels = arrayOf(
        Wheel("lf", hardwareMap, DcMotorSimple.Direction.REVERSE, telemetry),
        Wheel("rf", hardwareMap, DcMotorSimple.Direction.FORWARD, telemetry),
        Wheel("lb", hardwareMap, DcMotorSimple.Direction.REVERSE, telemetry),
        Wheel("rb", hardwareMap, DcMotorSimple.Direction.FORWARD, telemetry)
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
        telemetry.addData("y",yTransposed)
        telemetry.addData("x",xTransposed)
        telemetry.update()
        lastHeadingTime = System.nanoTime()
    }
    fun updateDrive(): Boolean { return updateRotation( true) }
    fun stop() { for (wheel in wheels) { wheel.setPower(0.0) } }
    fun driveManual(moveX: Float, moveY: Float, turn: Float) {
        val theta = -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS) + toRadians(startAngle)
        val xTransposed = moveX * cos(theta) - moveY * sin(theta)
        val yTransposed = moveY * cos(theta) + moveX * sin(theta)
        wheels[0].setPower(yTransposed + xTransposed + turn)
        wheels[1].setPower(yTransposed - xTransposed - turn)
        wheels[2].setPower(yTransposed - xTransposed + turn)
        wheels[3].setPower(yTransposed + xTransposed - turn)
    }

    fun updateRotation(toTarget: Boolean = false): Boolean {
        val currentAngle: Double = -imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES) + startAngle
        var error = targetAngle - currentAngle
        while (error > 180) error -= 360.0
        while (error <= -180) error += 360.0
        val currentTime = System.nanoTime()
        val deltaTime: Double = (currentTime - lastHeadingTime) / 1e9
        var derivative = 0.0
        if (deltaTime > 0) { derivative = (error - lastHeadingError) / deltaTime }
        val turn = (KP_HEADING * error) + (KD_HEADING * derivative)
        lastHeadingError = error
        lastHeadingTime = currentTime
        val result = abs(error) < 5 && (
            wheels[0].autoMove(turn, currentTime, toTarget) ||
            wheels[1].autoMove(-turn, currentTime, toTarget) ||
            wheels[2].autoMove(turn, currentTime, toTarget) ||
            wheels[3].autoMove(-turn, currentTime, toTarget) || !toTarget)
        telemetry.update()
        return result
    }

    fun getData(): String {
        var out = ""
        for (i in wheels) {
            out += i.getPosition().toString() + "\n"
        }
        return out
    }
}
