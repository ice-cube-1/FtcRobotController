package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Constants.ADD_TEST
import org.firstinspires.ftc.teamcode.Constants.CCW_TURRET
import org.firstinspires.ftc.teamcode.Constants.CW_TURRET
import org.firstinspires.ftc.teamcode.Constants.SERVO_ERROR
import org.firstinspires.ftc.teamcode.Constants.HOOD_ANGLE
import org.firstinspires.ftc.teamcode.Constants.KP_SHOOTER
import org.firstinspires.ftc.teamcode.Constants.TURRET_KP
import org.firstinspires.ftc.teamcode.Constants.TURRET_STEP
import org.firstinspires.ftc.teamcode.Constants.VELOCITY_DELTA
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min


enum class TurretState {DETECTED, WRAPPING}

class ShooterNew(hardwareMap: HardwareMap) {
    private val aprilTag = AprilTagProcessor.easyCreateWithDefaults()
    private var visionPortal: VisionPortal =
        VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "Webcam 1"), aprilTag)
    private val turret = arrayOf(
        hardwareMap.get(Servo::class.java, "t1"),
        hardwareMap.get(Servo::class.java, "t2")
    )
    private val turretRealPositions = arrayOf(
        hardwareMap.get(AnalogInput::class.java, "t1"),
        hardwareMap.get(AnalogInput::class.java, "t2")
    )
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
    private var scanTimer = ElapsedTime()
    var turretState = TurretState.WRAPPING
    private var shooterOn = false
    private var targetV = 0
    private var goto = CCW_TURRET
    private var tagID = 20
    private var manual = false
    init {
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0)
        hoodAngle.position = HOOD_ANGLE
    }
    fun setStart(tagID: Int, manual: Boolean) { this.tagID = tagID; this.manual = manual }
    private fun lookForTag() {
        val currentDetections = aprilTag.detections
        for (detection in currentDetections) {
            if (detection.id == tagID && detection.metadata != null) {
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
    fun setTurretPos(position: Double) {
        turret[0].position = position
        turret[1].position = 360 - position
    }
    private fun getTurretPosition() : Double {
        return turretRealPositions[0].voltage + turretRealPositions[1].voltage / 6.6 * 360
    }

    fun canShoot(): Boolean { return abs(lastAngle) < 3 && lastDist < 120.0 && (targetV - motors[1].getVelocity() < 30) }
    private fun getTargetVelocity(): Int {
        return (lastDist * 5.9976 + 1022.8 + ADD_TEST).toInt()
    }
    private fun centerOnTag() {
        if (abs(lastAngle - TURRET_KP) > 0.02) {
            turret[0].position = lastAngle + TURRET_KP
            turret[1].position = 360.0 - lastAngle + TURRET_KP
        }
    }
    private fun wrap() {
        if (scanTimer.milliseconds() < 500) {
            val error = goto - getTurretPosition()
            if (abs(error) < SERVO_ERROR) {
                goto = if (goto == CCW_TURRET) CW_TURRET else CCW_TURRET
            } else {
                turret[0].position += if (error > 0) TURRET_STEP else -TURRET_STEP
                turret[1].position -= if (error > 0) TURRET_STEP else -TURRET_STEP

            }
        } else if (scanTimer.milliseconds() > 800) { scanTimer.reset() }
    }
    fun moveTurret() {
        lookForTag()
        checkWraparound()
        when (turretState) {
            TurretState.DETECTED -> {
                if (timer.milliseconds() > mostRecent + 500) turretState = TurretState.WRAPPING
                else centerOnTag()
            }
            TurretState.WRAPPING -> {
                if (timer.milliseconds() < mostRecent + 500) turretState = TurretState.DETECTED
                else wrap()
            }
        }
    }
    private fun checkWraparound(): Boolean {
        if (abs(getTurretPosition() - (turret[1].position - 360.0 - turret[0].position)) > SERVO_ERROR) {
            goto = if (goto == CW_TURRET) CCW_TURRET else CW_TURRET
        }
        if (getTurretPosition() > CCW_TURRET/3) {
            turretState = TurretState.WRAPPING
            goto = CW_TURRET
        } else if (getTurretPosition() < CW_TURRET/3) {
            turretState = TurretState.WRAPPING
            goto = CCW_TURRET
        }
        else { return false }
        return true
    }
    fun turnOnShooter() {
        targetV = 0
        shooterOn = true
    }
    fun turnOffShooter() {
        shooterOn = false
        targetV = 0
        motors[0].setPower(0.0)
        motors[1].setPower(0.0)
    }
    fun spin(): Boolean {
        val velocity = getTargetVelocity()
        val error = targetV - motors.map { it.getVelocity() }.average()
        power = max(0.0, min(1.0, power + KP_SHOOTER * spinnerTimer.seconds() * error))
        if (shooterOn) { targetV = min(targetV + VELOCITY_DELTA, velocity) }
        if (shooterOn) { for (m in motors) { m.setPower(power) } }
        else {for (m in motors) {m.setPower(0.0) } }
        spinnerTimer.reset()
        return abs(error) < 100 && abs(targetV - velocity) < 100
    }
    fun getData() : String {
        return "Turret state: $turretState, last tag range = $lastDist, bearing = $lastAngle\n" +
                "Shooter target: $targetV, aiming for ${getTargetVelocity()}\n" +
                "power ${max(0.0, min(1.0, power + KP_SHOOTER * spinnerTimer.seconds() *
                        targetV - motors.map { it.getVelocity() }.average()))}\n" +
                "actually going at ${motors[0].getVelocity()}, ${motors[1].getVelocity()}"
    }
}