package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Constants.CCW_TURRET
import org.firstinspires.ftc.teamcode.Constants.CW_TURRET
import org.firstinspires.ftc.teamcode.Constants.ENCODER_ERROR
import org.firstinspires.ftc.teamcode.Constants.KP_SHOOTER
import org.firstinspires.ftc.teamcode.Constants.TURRET_ENCODER_KP
import org.firstinspires.ftc.teamcode.Constants.TURRET_SPEED
import org.firstinspires.ftc.teamcode.Constants.VELOCITY_DELTA
import org.firstinspires.ftc.teamcode.Constants.endVelocity
import org.firstinspires.ftc.teamcode.Constants.shootAngle
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Thread.sleep
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class ShooterUnconfigured(hardwareMap: HardwareMap) {
    private val aprilTag = AprilTagProcessor.easyCreateWithDefaults()
    private var visionPortal: VisionPortal =
        VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "Webcam 1"), aprilTag)
    private val turret = Wheel("turret", hardwareMap, DcMotorSimple.Direction.REVERSE)
    private val hoodAngle = hardwareMap.get(Servo::class.java, "hood")
    private var lastDist = 0.0
    private var lastAngle = 0.0
    private val motors =  arrayOf(
        Wheel("m1", hardwareMap, DcMotorSimple.Direction.REVERSE),
        Wheel("m2", hardwareMap, DcMotorSimple.Direction.FORWARD)
    )
    private var power = 0.0
    private var spinnerTimer = ElapsedTime()
    private var mostRecent = 0.0
    private var timer = ElapsedTime()
    var turretState = TurretState.SCANNING
    private var shooterOn = false
    private var targetV = 0
    private var scanMin = 0.0
    private var scanMax = 0.0
    private var toScanMin = true
    private var goto = CCW_TURRET
    private var boundOffset = 0.0
    init { FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0) }
    fun setStart(boundOffset: Double) { this.boundOffset = boundOffset }
    private fun lookForTag() {
        val currentDetections = aprilTag.detections
        for (detection in currentDetections) {
            if (detection.id == 20 && detection.metadata != null) {
                lastDist = detection.ftcPose.range
                lastAngle = detection.ftcPose.bearing
                mostRecent = timer.milliseconds()
            }
        }
    }
    fun getMotifID(): Int {
        val currentDetections = aprilTag.detections
        for (detection in currentDetections) {
            if (detection.id in 21..23) {return detection.id }
        }
        return -1
    }
    fun setTurretManual(position: Double) {
        turret.setTarget(CCW_TURRET - (CCW_TURRET- CW_TURRET)*(position + 180)/360)
        while (abs(turret.getTarget() - turret.getPosition()) > ENCODER_ERROR) {
            turret.setPower(TURRET_ENCODER_KP * (turret.getTarget() - turret.getPosition())/10.0)
        }
        turret.setPower(0.0)
        sleep(1000)
    }

    fun canShoot(): Boolean { /** TODO add range check */
        return abs(lastAngle) < 3
    }
    private fun getTargetVelocityAngle(): Pair<Int, Double> {
        return Pair(endVelocity, shootAngle)
    }
    private fun centerOnTag() { turret.setPower(-lastAngle * TURRET_ENCODER_KP) }
    private fun scan() {
        if (scanMin < scanMax) {
            if ((toScanMin && scanMin > turret.getPosition()) || (!toScanMin && scanMax < turret.getPosition())) { toScanMin = !toScanMin }
            turret.setPower(if (toScanMin) -TURRET_SPEED else TURRET_SPEED)
        } else {
            turretState = TurretState.WRAPPING
            goto = if (goto == CCW_TURRET) CW_TURRET else CCW_TURRET
        }
    }
    fun moveTurret(orientation: Double) {
        lookForTag()
        setScanBounds(orientation-45.0 + boundOffset, orientation+45.0 + boundOffset)
        checkWraparound()
        when (turretState) {
            TurretState.SCANNING -> {
                if (timer.milliseconds() < mostRecent + 500) turretState = TurretState.DETECTED
                else scan()
            }
            TurretState.DETECTED -> {
                if (timer.milliseconds() > mostRecent + 500) turretState = TurretState.SCANNING
                else centerOnTag()
            }
            TurretState.WRAPPING -> {
                if (timer.milliseconds() < mostRecent + 500) turretState = TurretState.DETECTED
                else wrap()
            }
        }
    }
    private fun setScanBounds(min: Double, max: Double) {
        scanMax = CCW_TURRET - (CCW_TURRET - CW_TURRET)*(min+180)/360
        scanMin= CCW_TURRET - (CCW_TURRET - CW_TURRET)*(max+180)/360
        if (scanMin < CW_TURRET) scanMin += CCW_TURRET- CW_TURRET
        if (scanMax > CCW_TURRET) scanMax -= CCW_TURRET- CW_TURRET
    }
    private fun checkWraparound(): Boolean {
        if (turret.getPosition() > CCW_TURRET/2) {
            turretState = TurretState.WRAPPING
            goto = CW_TURRET
        } else if (turret.getPosition() < CW_TURRET/2) {
            turretState = TurretState.WRAPPING
            goto = CCW_TURRET
        }
        else { return false }
        return true
    }
    private fun wrap() {
        val error = goto - turret.getPosition()
        if (abs(error) < ENCODER_ERROR) {
            turret.setPower(0.0)
            turretState = TurretState.SCANNING
        } else {
            turret.setPower(if (error > 0) TURRET_SPEED else -TURRET_SPEED)
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
    fun getData() : String {
        return "Turret state: $turretState, last tag range = $lastDist, bearing = $lastAngle\n" +
                "Shooter target: $targetV, aiming for ${getTargetVelocityAngle().first}\n" +
                "power ${max(0.0, min(1.0, power + KP_SHOOTER * spinnerTimer.seconds() *
                        targetV - motors.map { it.getVelocity() }.average()))}\n" +
                "can shoot: ${canShoot()}"
    }
}