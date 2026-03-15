package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Constants.CCW_TURRET
import org.firstinspires.ftc.teamcode.Constants.CW_TURRET
import org.firstinspires.ftc.teamcode.Constants.KP_SHOOTER
import org.firstinspires.ftc.teamcode.Constants.TURRET_ENCODER_KP
import org.firstinspires.ftc.teamcode.Constants.TURRET_SPEED
import org.firstinspires.ftc.teamcode.Constants.VELOCITY_DELTA
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

enum class TurretState {SCANNING, MAYBE_FOUND, FOUND_CONFIRMED, MATCH_LOST}

class Shooter (hardwareMap: HardwareMap, vision: Boolean = false, telemetry: Telemetry) {
    private val aprilTag = AprilTagProcessor.easyCreateWithDefaults()
    private lateinit var visionPortal: VisionPortal
    private val turret = Wheel("turret", hardwareMap, DcMotorSimple.Direction.FORWARD, telemetry)
    private val hoodAngle = hardwareMap.get(Servo::class.java, "hood")
    private var lastDist = 0.0
    private var lastAngle = 0.0
    private val motors =  arrayOf(
        Wheel("m1", hardwareMap, DcMotorSimple.Direction.REVERSE, telemetry),
        Wheel("m2", hardwareMap, DcMotorSimple.Direction.FORWARD, telemetry)
    )
    private var power = 0.0
    private var spinnerTimer = ElapsedTime()
    private var cameraStateTimer = ElapsedTime()
    var turretState = TurretState.SCANNING
    private var turretDirection = 1
    private var shooterOn = false
    private var targetV = 0
    init { if (vision) {
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "Webcam 1"), aprilTag)
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0)
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
    fun getMotifID(): Int {
        val currentDetections = aprilTag.detections
        for (detection in currentDetections) {
            if (detection.id in 21..23) {return getMotifID()}
        }
        return -1
    }
    fun setTurretManual(position: Double) {
        turret.setTarget(CW_TURRET + (CCW_TURRET- CW_TURRET)*(position + 180)/360)
        while (abs(turret.getTarget() - turret.getPosition()) > 5) {
            turret.setPower(TURRET_ENCODER_KP * (turret.getTarget() - turret.getPosition()))
        }
    }
    fun reset() {
        turnOffShooter()
        turret.resetEncoder()
    }
    fun canShoot(): Boolean { /** TODO add range check */
        return abs(lastAngle) < 2
    }
    private fun getTargetVelocityAngle(): Pair<Int, Double> {
        return Pair(0, 0.0) /** TODO once calibrated */
    }
    private fun centerOnTag() {
        if (!(turret.getPosition() > CCW_TURRET || turret.getPosition() < CW_TURRET)) {
            turret.setPower(lastAngle * TURRET_ENCODER_KP)
        }
    }
    private fun scan() {
        if (turret.getPosition() > CCW_TURRET || turret.getPosition() < CW_TURRET) { turretDirection *= -1 }
        turret.setPower(TURRET_SPEED * turretDirection)
    }
    fun moveTurret() {
        if (lookForTag()) {
            when (turretState) {
                TurretState.SCANNING -> {
                    turretState = TurretState.MAYBE_FOUND
                    cameraStateTimer.reset()
                }
                TurretState.MAYBE_FOUND -> {
                    if (cameraStateTimer.milliseconds() > 50) turretState = TurretState.FOUND_CONFIRMED
                    else centerOnTag()
                }
                TurretState.FOUND_CONFIRMED -> centerOnTag()
                TurretState.MATCH_LOST -> turretState = TurretState.FOUND_CONFIRMED
            }
        } else {
            when (turretState) {
                TurretState.SCANNING -> scan()
                TurretState.MAYBE_FOUND -> turretState = TurretState.SCANNING
                TurretState.FOUND_CONFIRMED -> {
                    turretState = TurretState.MATCH_LOST
                    cameraStateTimer.reset()
                }
                TurretState.MATCH_LOST -> {
                    if (cameraStateTimer.milliseconds() > 50) turretState = TurretState.MATCH_LOST
                    else scan()
                }
            }
        }
    }
    fun turnOnShooter() {
        val values = getTargetVelocityAngle()
        hoodAngle.position = values.second
        targetV = 0
        shooterOn = true
    }
    fun turnOffShooter() { shooterOn = false }

    fun spin(): Boolean {
        val values = getTargetVelocityAngle()
        hoodAngle.position = values.second
        val error = targetV - motors.map { it.getVelocity() }.average()
        power = max(0.0, min(1.0, power + KP_SHOOTER * spinnerTimer.seconds() * error))
        if (shooterOn) { targetV = min(targetV + VELOCITY_DELTA, values.first) }
        if (shooterOn) { for (m in motors) { m.setPower(power) } }
        else {for (m in motors) {m.setPower(0.0) } }
        spinnerTimer.reset()
        return abs(error) < 100 && abs(targetV - values.first) < 100
    }
}