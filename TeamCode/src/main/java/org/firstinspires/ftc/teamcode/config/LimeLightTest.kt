package org.firstinspires.ftc.teamcode.config

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.lang.Math.toDegrees
import kotlin.math.atan
import kotlin.math.sqrt


@TeleOp
class LimeLightTest: LinearOpMode() {
    private lateinit var limelight: Limelight3A

    override fun runOpMode() {
        waitForStart()
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.setPollRateHz(100)
        limelight.start()
        limelight.pipelineSwitch(0)
        while (opModeIsActive()) {
            val result = limelight.latestResult
            if (result != null && result.isValid) {
                val fiducials = result.fiducialResults
                for (fiducial in fiducials) {
                    val pose = fiducial.targetPoseCameraSpace

                    telemetry.addData("Tag ID", fiducial.fiducialId)
                    telemetry.addData("Range (m)", sqrt(pose.position.x * pose.position.x + pose.position.z * pose.position.z))
                    telemetry.addData("bearing", toDegrees(atan(pose.position.x / pose.position.z)))
                }
                telemetry.update()
            }
        }
    }
}