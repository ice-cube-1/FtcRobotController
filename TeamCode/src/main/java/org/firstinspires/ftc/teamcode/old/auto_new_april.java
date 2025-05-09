package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Auto AprilTag Center", group = "Robot")
@Disabled
public class auto_new_april extends LinearOpMode {

    private DcMotor lf, rf, lb, rb;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private final double targetRange = 30.0; // inches
    private final double rangeTolerance = 1.0; // inches
    private final double bearingTolerance = 2.0; // degrees
    private final double yawTolerance = 2.0; // degrees

    @Override
    public void runOpMode() {
        // Hardware setup
        lf = hardwareMap.get(DcMotor.class, "left_front_drive");
        rf = hardwareMap.get(DcMotor.class, "right_front_drive");
        lb = hardwareMap.get(DcMotor.class, "left_back_drive");
        rb = hardwareMap.get(DcMotor.class, "right_back_drive");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = {lf, rf, lb, rb};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Vision setup
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.size() == 1) {
                AprilTagDetection tag = detections.get(0);

                double rangeError = tag.ftcPose.range - targetRange;
                double bearing = tag.ftcPose.bearing;
                double yaw = tag.ftcPose.yaw;

                telemetry.addLine(String.format("Range: %.1f in, Bearing: %.1f°, Yaw: %.1f°", tag.ftcPose.range, bearing, yaw));

                boolean centered = Math.abs(rangeError) < rangeTolerance &&
                        Math.abs(bearing) < bearingTolerance &&
                        Math.abs(yaw) < yawTolerance;

                if (centered) {
                    stopAllMotors();
                    telemetry.addLine("Centered on tag!");
                } else {
                    double forward = clip(rangeError / 12.0);   // normalize range
                    double strafe = clip(-bearing / 15.0);      // strafe left/right (invert because bearing is +right)
                    double turn = clip(yaw / 30.0);            // rotate (invert to correct yaw direction)

                    // Mecanum drive mixing
                    lf.setPower(0.1 * (forward + strafe + turn));
                    rf.setPower(0.1 * (forward - strafe - turn));
                    lb.setPower(0.1 * (forward - strafe + turn));
                    rb.setPower(0.1 * (forward + strafe - turn));
                }
            } else {
                stopAllMotors();
                telemetry.addLine("No tag detected.");
            }

            telemetry.update();
        }

        stopAllMotors();
    }

    private double clip(double val) {
        return Math.max(-1.0, Math.min(1.0, val));
    }

    private void stopAllMotors() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}
