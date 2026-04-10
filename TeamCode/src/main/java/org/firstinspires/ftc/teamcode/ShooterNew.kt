package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.HOOD_ANGLE
import org.firstinspires.ftc.teamcode.Constants.KP_SHOOTER
import org.firstinspires.ftc.teamcode.Constants.TURRET_KP
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
    private var limelight = hardwareMap.get(Limelight3A::class.java, "limelight").apply {
        setPollRateHz(100)
        start()
        pipelineSwitch(0)
    }
    private val turret = arrayOf(
        hardwareMap.get(ServoImplEx::class.java, "t1").apply {
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
            position = 0.5
        },
        hardwareMap.get(ServoImplEx::class.java, "t2").apply {
            pwmRange = PwmControl.PwmRange(500.0, 2500.0)
            position = 0.5
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
    private var mostRecent = -1000.0
    private var timer = ElapsedTime()
    private var scanTimer = ElapsedTime()
    private var turretState = TurretState.WRAPPING
    private var shooterOn = false
    private var targetV = 0
    private var data = ""
    private var goto = -TURRET_ZERO_DEG
    private var nextPos = 0.0
    fun getMotifID(): Int {
        val currentDetections = limelight.latestResult
        for (tag in currentDetections.fiducialResults) {
            if (tag.fiducialId in 21..23) { return tag.fiducialId }
        }
        return -1
    }
    var atSpeed = false

    fun canShoot(): Boolean { return abs(lastAngle) < 3 && lastDist < 120.0 && (targetV - motors[1].getVelocity() < 30) }
    private fun getTargetVelocity(): Int {
        return (lastDist * 5.9976 + 1022.8).toInt()
    }
    private fun lookForTag() {
        val result = limelight.latestResult
        for (tag in result.fiducialResults) {
            data = " " + tag.fiducialId + " " + tag.targetPoseCameraSpace.position
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
                else if (abs(lastAngle) > 1.0) nextPos = lastAngle * TURRET_KP + getTurretAngle()
            }
            TurretState.WRAPPING -> {
                if (timer.milliseconds() < mostRecent + 500) turretState = TurretState.DETECTED
                else {
                    nextPos = (if (goto - getTurretAngle() > 0) TURRET_STEP else -TURRET_STEP) + getTurretAngle()
                    scanTimer.reset()
                }
            }
        }
        if (nextPos >= TURRET_MAX_DEGREES - TURRET_ZERO_DEG) {
            goto = -TURRET_ZERO_DEG
        } else if (nextPos <= -TURRET_ZERO_DEG) {
            goto = TURRET_MAX_DEGREES - TURRET_ZERO_DEG
        } else {
            setTurretPos(nextPos)
        }
    }
    private fun getTurretAngle() : Double {
        return ((turret[0].position + turret[1].position) / 2.0) * TURRET_MAX_DEGREES - TURRET_ZERO_DEG
    }
    fun manualRotate(goto: Double) : Boolean {
        setTurretPos((if (goto - getTurretAngle() > 0) TURRET_STEP else -TURRET_STEP) + getTurretAngle())
        return abs(goto - getTurretAngle()) < 1.0
    }
    private fun setTurretPos(pos: Double) {
        val goto = min(1.0, max((pos + TURRET_ZERO_DEG) / TURRET_MAX_DEGREES, 0.0))
        turret[0].position = goto
        turret[1].position = goto
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
    fun spin() {
        val velocity = getTargetVelocity()
        val error = targetV - motors.map { it.getVelocity() }.average()
        power = max(0.0, min(1.0, power + KP_SHOOTER * spinnerTimer.seconds() * error))
        if (shooterOn) { targetV = min(targetV + VELOCITY_DELTA, velocity) }
        if (shooterOn) { for (m in motors) { m.setPower(power) } }
        else {for (m in motors) {m.setPower(0.0) } }
        spinnerTimer.reset()
        atSpeed = abs(targetV - velocity) < 10 && abs(error) < 100
    }
    fun getData() : String {
        return "Turret state: $turretState, last tag range = $lastDist, bearing = $lastAngle\n" +
                "Turret current position ${getTurretAngle()}, going to $goto + $data\n"+
                "Shooter target: $targetV, aiming for ${getTargetVelocity()}\n" +
                "power ${max(0.0, min(1.0, power + KP_SHOOTER * spinnerTimer.seconds() *
                        (targetV - motors.map { it.getVelocity() }.average())))}\n" +
                "actually going at ${motors[0].getVelocity()}, ${motors[1].getVelocity()}"
    }
}