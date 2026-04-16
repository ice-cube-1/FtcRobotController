package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Constants.ODOMETRY_TEST_CM

@TeleOp(name = "Square Test (Odometry)")
class SquareTest : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    override fun runOpMode() {
        drivetrain = OdometryDrivetrain(hardwareMap)
        waitForStart()
        val points = arrayOf(arrayOf(0.0,ODOMETRY_TEST_CM,ODOMETRY_TEST_CM,0.0), arrayOf(ODOMETRY_TEST_CM,ODOMETRY_TEST_CM,0.0,0.0))
        for (i in 0..<points[0].size) {
            drivetrain.updateGoto(points[0][i], points[1][i], 0.0)
            while (!drivetrain.continueDriving()) {
                telemetry.addLine(drivetrain.getData())
                telemetry.update()
            }
        }
        drivetrain.driveManual(0.0, 0.0, 0.0)
    }
}