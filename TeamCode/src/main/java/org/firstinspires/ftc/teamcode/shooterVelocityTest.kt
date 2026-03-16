package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE
import org.firstinspires.ftc.teamcode.Constants.KP_SHOOTER
import org.firstinspires.ftc.teamcode.Constants.VELOCITY_DELTA
import org.firstinspires.ftc.teamcode.Constants.endVelocity
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "shooter velocity test", group = "Linear OpMode")
class ShooterVelocityTest : LinearOpMode() {
    private lateinit var motors: Array<Wheel>
    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    private fun lookForTag(): Boolean {
        val currentDetections = aprilTag.detections
        for (detection in currentDetections) {
            if (detection.id == 20 && detection.metadata != null) {
                telemetry.addData("apriltag", detection.id)
                telemetry.addData("name", detection.metadata.name)
                telemetry.addData("range", detection.ftcPose.range)
                telemetry.addData("bearing", detection.ftcPose.bearing)
                return true
            }
        }
        return false
    }
    override fun runOpMode() {
        aprilTag = AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary()).build()
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(aprilTag)
            .build()
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0.0)
        val hoodAngle = hardwareMap.get(Servo::class.java, "s")
        motors =  arrayOf(
            Wheel("m1", hardwareMap, DcMotorSimple.Direction.FORWARD),
            Wheel("m2", hardwareMap, DcMotorSimple.Direction.REVERSE)
        )
        val kickarm = hardwareMap.get(Servo::class.java, "kickArm")
        var power = 0.0
        val timer = ElapsedTime()
        timer.reset()
        var shooterOn = false
        var targetV = 0
        while (opModeIsActive()) {
            if (gamepad1.x) {
                shooterOn = !shooterOn
                sleep(200)
                targetV = 0
            }
            if (shooterOn) { targetV = min(targetV + VELOCITY_DELTA, endVelocity) }
            if (gamepad1.dpad_up) { hoodAngle.position += 0.0005 }
            else if (gamepad1.dpad_down) { hoodAngle.position-=0.0005 }
            if (gamepad1.a) { kickarm.position = KICKARM_DOWN }
            if (gamepad1.b) { kickarm.position = KICKARM_RELEASE }
            val error = targetV - motors.map { it.getVelocity() }.average()
            power = max(0.0, min(1.0, power + KP_SHOOTER * timer.seconds() * error))
            timer.reset()
            if (shooterOn) { for (m in motors) { m.setPower(power) } }
            else { for (m in motors) { m.setPower(0.0) } }
            lookForTag()
            telemetry.addData("power", power)
            telemetry.addData("gotoV", targetV)
            telemetry.addData("shooter on", shooterOn)
            telemetry.addData("servo pos", hoodAngle.position)
            telemetry.update()
        }
    }
}