package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.opencv.core.Point
import kotlin.math.atan2
import kotlin.math.hypot


@TeleOp(name = "Color Sensor Test", group = "Test")
class ColorSensorTest : OpMode() {

    private lateinit var colorSensor: RevColorSensorV3
    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    override fun init() {
        aprilTag = AprilTagProcessor.Builder().setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .build()
        visionPortal = VisionPortal.Builder()
           .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(aprilTag)
            .build()
        colorSensor = hardwareMap.get(RevColorSensorV3::class.java, "color_sensor")
        colorSensor.enableLed(false)
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0);
    }

    private fun lookForTag(): Boolean {
        val currentDetections = aprilTag.detections
        for (detection in currentDetections) {
            telemetry.addData("apriltag", detection.id)
            val (range, bearing) = aprilTagPoseFromCorners(detection.corners, 822.317, 319.495)
            telemetry.addData("range_inch", range)
            telemetry.addData("bearing",bearing)
        }
        return false
    }


    override fun loop() {
        lookForTag()
        telemetry.addData("Red", colorSensor.red().toFloat())
        telemetry.addData("Green",colorSensor.green().toFloat())
        telemetry.addData("Blue", "%.5f", colorSensor.blue().toFloat())
        telemetry.addData("Alpha", "%.5f", colorSensor.alpha().toFloat())

        telemetry.addLine("Move colored objects in front of the sensor!")

        telemetry.update()
    }
}

fun aprilTagPoseFromCorners(corners: Array<Point>, fx: Double, cx: Double): Pair<Double, Double> {
    val centerX = corners.sumOf {it.x / 4.0}
    val widthPx = hypot(corners[0].x - corners[1].x, corners[0].y - corners[1].y)
    val heightPx = hypot(corners[0].x - corners[3].x, corners[0].y - corners[3].y)
    val avgPx = (widthPx + heightPx) / 2.0
    val tagSizeInches = 10.0 / 2.54
    val rangeInches = fx * tagSizeInches / avgPx
    val bearingRad = atan2(centerX - cx, fx)
    val bearingDeg = Math.toDegrees(bearingRad)
    return Pair(rangeInches, bearingDeg)
}
