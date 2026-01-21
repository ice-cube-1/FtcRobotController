package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.PIDConstants.Companion.KD_HEADING
import org.firstinspires.ftc.teamcode.PIDConstants.Companion.KD_TRANSLATION
import org.firstinspires.ftc.teamcode.PIDConstants.Companion.KP_HEADING
import org.firstinspires.ftc.teamcode.PIDConstants.Companion.KP_TRANSLATION


class PIDConstants {
    companion object {
        const val KP_HEADING: Double = 1.0
        const val KD_HEADING: Double = 0.001
        const val KP_TRANSLATION: Double = 0.01
        const val KD_TRANSLATION: Double = 0.001
    }
}


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
    private var targetAngle = 0.0
    private var lastHeadingError = 0.0
    private var lastHeadingTime = 0L

    fun update() {
        turnPower()
        wheels[0].move(lastHeadingError)
        wheels[1].move(-lastHeadingError)
        wheels[2].move(lastHeadingError)
        wheels[3].move(-lastHeadingError)
    }

    private fun turnPower() {
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
    }
}

class Wheel(val name: String, hardwareMap: HardwareMap, direction: DcMotorSimple.Direction) {
    val drive: DcMotor = hardwareMap.get(DcMotor::class.java, name).apply {
        setDirection(direction)
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    private val target = 0.0
    private var lastError = 0.0
    private var lastTime = 0L

    fun move(headingError: Double) {
        val currentTime = System.nanoTime()
        val error = target - drive.currentPosition.toDouble()
        val deltaTime = (currentTime - lastTime) / 1e9
        var derivative = 0.0
        if (deltaTime > 0) { derivative = (error -  lastError) / deltaTime }
        lastError = error
        lastTime = currentTime
        drive.power = (KP_TRANSLATION * error) + (KD_TRANSLATION * derivative) + headingError
    }
}
