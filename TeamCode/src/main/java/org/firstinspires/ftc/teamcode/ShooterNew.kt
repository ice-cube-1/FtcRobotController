package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.CCW_TURRET
import org.firstinspires.ftc.teamcode.Constants.CW_TURRET
import org.firstinspires.ftc.teamcode.Constants.HOOD_ANGLE
import org.firstinspires.ftc.teamcode.Constants.KP_SHOOTER
import org.firstinspires.ftc.teamcode.Constants.TURRET_MAX_DEGREES
import org.firstinspires.ftc.teamcode.Constants.TURRET_STEP
import org.firstinspires.ftc.teamcode.Constants.TURRET_ZERO_DEG
import org.firstinspires.ftc.teamcode.Constants.VELOCITY_DELTA
import java.lang.Math.toDegrees
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt


enum class TurretState {DETECTED, WRAPPING}

class ShooterNew(hardwareMap: HardwareMap, private var tagID: Int) {
    private var limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
    private val turret = arrayOf(
        hardwareMap.get(ServoImplEx::class.java, "t1").apply {
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        },
        hardwareMap.get(ServoImplEx::class.java, "t2").apply {
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        }
    )
    private val hoodAngle = hardwareMap.get(Servo::class.java, "hood").apply { position = HOOD_ANGLE }
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
    private var nextPos = 0.0
    fun getMotifID(): Int {
        val currentDetections = limelight.latestResult
        for (tag in currentDetections.fiducialResults) {
            if (tag.fiducialId in 21..23) {return tag.fiducialId }
        }
        return -1
    }

    fun canShoot(): Boolean { return abs(lastAngle) < 3 && lastDist < 120.0 && (targetV - motors[1].getVelocity() < 30) }
    private fun getTargetVelocity(): Int {
        return (lastDist * 5.9976 + 1022.8).toInt()
    }
    private fun lookForTag() {
        val result = limelight.latestResult
        for (tag in result.fiducialResults) {
            if (tag.fiducialId == tagID) {
                mostRecent = timer.milliseconds() - result.staleness
                val pos = tag.targetPoseCameraSpace.position
                lastDist = sqrt(pos.x * pos.x + pos.z * pos.z)
                lastAngle = toDegrees(atan2(pos.x , pos.z))
            }
        }
    }
    fun moveTurret() {
        lookForTag()
        when (turretState) {
            TurretState.DETECTED -> {
                if (timer.milliseconds() > mostRecent + 500) turretState = TurretState.WRAPPING
                else if (abs(lastAngle) > 0.02) {
                    nextPos = lastAngle + getTurretAngle()
                }
            }
            TurretState.WRAPPING -> {
                if (timer.milliseconds() < mostRecent + 500) turretState = TurretState.DETECTED
                else {
                    if (scanTimer.milliseconds() > 500) {
                        val error = goto - getTurretAngle()
                        nextPos = (if (error > 0) TURRET_STEP else -TURRET_STEP) + getTurretAngle()
                        scanTimer.reset()
                    } else {
                        nextPos = getTurretAngle()
                    }
                }
            }
        }
        if (nextPos > CW_TURRET) {
            goto = CCW_TURRET
        } else if (nextPos < CCW_TURRET) {
            goto = CW_TURRET
        } else {
            setTurretPos(nextPos)
        }
    }
    private fun getTurretAngle() : Double {
        return ((turret[0].position + turret[1].position) / 2) * TURRET_MAX_DEGREES - TURRET_ZERO_DEG
    }
    private fun setTurretPos(pos: Double) {
        turret[0].position = (pos + TURRET_ZERO_DEG) / TURRET_MAX_DEGREES
        turret[1].position = (pos + TURRET_ZERO_DEG) / TURRET_MAX_DEGREES
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