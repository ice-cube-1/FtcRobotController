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
import org.firstinspires.ftc.teamcode.Constants.Companion.MAX_TURRET
import org.firstinspires.ftc.teamcode.Constants.Companion.MIN_TURRET
import org.firstinspires.ftc.teamcode.Constants.Companion.TURRET_ENCODER_KP
import org.firstinspires.ftc.teamcode.Constants.Companion.TURRET_STEPS
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min


class Shooter (hardwareMap: HardwareMap, vision: Boolean = false) {
    private val aprilTag = AprilTagProcessor.easyCreateWithDefaults()
    private lateinit var visionPortal: VisionPortal
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
    init { if (vision) {
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "Webcam 1"), aprilTag)
    } }
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
    private fun getTargetV_Angle(): Pair<Double, Double> {
        return Pair(0.0, 0.0) /** TODO once calibrated */
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
    private fun checkWraparound() {
        if (turret.targetPosition > MAX_TURRET) {
            turret.targetPosition = MIN_TURRET
        } else if (turret.targetPosition < MIN_TURRET) {
            turret.targetPosition = MAX_TURRET
        }
    }
    private fun spin() {
        val values = getTargetV_Angle()
        hoodAngle.position = values.second
        val error = values.first - getVelocity()
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
    fun manual(turretRotation: Double, back: Float, shooterOn: Boolean) {
        if (shooterOn) { spin() }
        turret.power = turretRotation
        hoodAngle.position += back
    }
}