package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Constants.Companion.KI_SPINNER
import org.firstinspires.ftc.teamcode.Constants.Companion.TURRET_ENCODER_KP
import org.firstinspires.ftc.teamcode.Constants.Companion.TURRET_STEPS
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min


class Shooter (hardwareMap: HardwareMap) {
    private val aprilTag = AprilTagProcessor.easyCreateWithDefaults()
    private val visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "Webcam 1"), aprilTag)
    private val turret = hardwareMap.get(DcMotor::class.java, "turret")
    private val hoodAngle = hardwareMap.get(Servo::class.java, "hood")
    private var lastDist = 0.0
    private var lastAngle = 0.0
    private val motors =  arrayOf(
        Wheel("m1", hardwareMap, DcMotorSimple.Direction.REVERSE, telemetry),
        Wheel("m2", hardwareMap, DcMotorSimple.Direction.FORWARD, telemetry)
    )
    private var power = 0.0
    private var timer = ElapsedTime()
    var shooterOn = false
    init { FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0); }

    private fun lookForTag(): Boolean {
        val currentDetections = aprilTag.detections
        for (detection in currentDetections) {
            if (detection.id == 20) {
                lastDist = detection.ftcPose.range
                lastAngle = detection.ftcPose.bearing
                return true
            }
        }
        return false
    }

    fun canShoot(): Boolean { /** TODO add range check */
        return abs(lastAngle) < 2
    }
    private fun getTargetV(): Double {
        return 0.0 /** TODO once calibrated */
    }
    private fun getTargetAngle(): Double {
        return 0.0 /** TODO once calibrated */
    }
    fun centerOnTag(): Boolean {
        if (lookForTag()) {
            turret.targetPosition += (lastAngle * TURRET_ENCODER_KP).toInt()
            checkWraparound()
            return true
        } else {
            return false
        }
    }
    fun scan() {
        turret.targetPosition += TURRET_STEPS
        checkWraparound()
    }
    fun checkWraparound() {
        if (turret.targetPosition > MAX_TURRET) {
            turret.targetPosition = MIN_TURRET
        } else if (turret.targetPosition < MIN_TURRET) {
            turret.targetPosition = MAX_TURRET
        }
    }
    fun spin() {
        hoodAngle.position = getTargetAngle()
        val error = getTargetV() - getVelocity()
        power = max(0.0, min(1.0, power + KI_SPINNER * timer.seconds() * error))
        timer.reset()
        if (shooterOn) { for (m in motors) { m.setPower(power) } }
        else {for (m in motors) {m.setPower(0.0) } }
    }
    private fun getVelocity(): Double {
        var ans = 0.0
        for (motor in motors) { ans += motor.getVelocity() }
        return ans / 2
    }
}