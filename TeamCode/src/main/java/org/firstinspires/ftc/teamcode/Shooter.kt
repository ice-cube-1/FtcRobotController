package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Constants.Companion.KI_SPINNER
import org.firstinspires.ftc.teamcode.Constants.Companion.KP_SHOOTER
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min


class Shooter (hardwareMap: HardwareMap) {
    private val aprilTag = AprilTagProcessor.easyCreateWithDefaults();
    private val visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName::class.java, "Webcam 1"), aprilTag);
    private val turret = hardwareMap.get(DcMotor::class.java, "turret")
    private val hoodAngle = hardwareMap.get(Servo::class.java, "hood")
    private var lastDist = 0.0
    private var lastAngle = 0.0
    private val motors =  arrayOf(initMotor("m1", DcMotorSimple.Direction.REVERSE), initMotor("m2", DcMotorSimple.Direction.FORWARD));
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
    fun centerOnTag() { /** TODO add scanning & make sure it doesn't fully wrap around */
        if (lookForTag()) {
            turret.power = lastAngle * KP_SHOOTER
        } else {
            turret.power = 0.0
        }
    }
    fun spin() {
        hoodAngle.position = getTargetAngle()
        val error = getTargetV() - getVelocity();
        power = max(0.0, min(1.0, power + KI_SPINNER * timer.seconds() * error));
        timer.reset();
        if (shooterOn) { for (m in motors) { m.power = power; } }
        else {for (m in motors) {m.power = 0.0; } }
    }
    private fun getVelocity(): Double {
        var ans = 0.0
        for (motor in motors) {
            ans += motor.velocity
        }
        return ans / 2
    }
    private fun initMotor(name: String, motorDirection: DcMotorSimple.Direction): DcMotorEx {
        return hardwareMap.get(DcMotorEx::class.java, name).apply {
            direction = motorDirection
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_USING_ENCODER
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
}