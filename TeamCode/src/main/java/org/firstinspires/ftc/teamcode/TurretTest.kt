package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Constants.CCW_TURRET
import org.firstinspires.ftc.teamcode.Constants.CW_TURRET
import org.firstinspires.ftc.teamcode.Constants.ENCODER_ERROR
import org.firstinspires.ftc.teamcode.Constants.TURRET_ENCODER_KP
import org.firstinspires.ftc.teamcode.Constants.TURRET_SPEED
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

private enum class Turret {SCANNING, DETECTED, WRAPPING}

@TeleOp(name = "Turret test", group = "Test")
class TurretTest : LinearOpMode() {

    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal
    private lateinit var turret: Wheel
    private var lastDist = 0.0
    private var lastAngle = 0.0
    private var mostRecent = 0.0
    private var timer = ElapsedTime()
    private var turretState = Turret.SCANNING
    private var goto = CCW_TURRET
    private var scanMin = 0.0
    private var scanMax = 0.0
    private var toScanMin = true
    override fun runOpMode() {
        turret = Wheel("m", hardwareMap, DcMotorSimple.Direction.REVERSE, telemetry)
        aprilTag = AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary()).build()
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(aprilTag)
            .build()
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0)
        setScanBounds(135.0,-135.0)
        waitForStart()
        turret.resetEncoder()
        while (opModeIsActive()) {
            checkWraparound()
            when (turretState) {
                Turret.SCANNING -> {
                    if (timer.milliseconds() < mostRecent + 500) turretState = Turret.DETECTED
                    else scan()
                }
                Turret.DETECTED -> {
                    if (timer.milliseconds() > mostRecent + 500) turretState = Turret.SCANNING
                    else centerOnTag()
                }
                Turret.WRAPPING -> {
                    if (timer.milliseconds() < mostRecent + 500) {
                        turretState = Turret.DETECTED
                    } else wrap()
                }
            }
            telemetry.addData("turret state", turretState)
            telemetry.addData("goto (wrap)", goto)
            telemetry.addData("pos", turret.getPosition())
            lookForTag()
            telemetry.update()
        }
    }
    private fun setScanBounds(min: Double, max: Double) {
        scanMin = CW_TURRET + (CCW_TURRET - CW_TURRET)*(min+180)/360
        scanMax= CW_TURRET + (CCW_TURRET - CW_TURRET)*(max+180)/360
    }
    private fun checkWraparound(): Boolean {
        if (turret.getPosition() > CCW_TURRET) {
            turretState = Turret.WRAPPING
            goto = CW_TURRET
        } else if (turret.getPosition() < CW_TURRET) {
            turretState = Turret.WRAPPING
            goto = CCW_TURRET
        }
        else { return false }
        return true
    }
    private fun wrap() {
        val error = goto - turret.getPosition()
        if (abs(error) < ENCODER_ERROR) {
            turret.setPower(0.0)
            turretState = Turret.SCANNING
        } else {
            turret.setPower(if (error > 0) TURRET_SPEED else -TURRET_SPEED)
        }
    }
    private fun centerOnTag() { turret.setPower(-lastAngle * TURRET_ENCODER_KP) }
    private fun scan() {
        if (scanMin < scanMax) {
            if ((toScanMin && scanMin > turret.getPosition()) || (!toScanMin && scanMax < turret.getPosition())) { toScanMin = !toScanMin }
            turret.setPower(if (toScanMin) -TURRET_SPEED else TURRET_SPEED)
        } else {
            turretState = Turret.WRAPPING
            goto = if (goto == CCW_TURRET) CW_TURRET else CCW_TURRET
        }
    }
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
}
