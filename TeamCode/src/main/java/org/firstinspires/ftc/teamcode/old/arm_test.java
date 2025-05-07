package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="arm test", group="Linear OpMode")
public class arm_test extends LinearOpMode {
    private Motor[] motors = null;
    private Servo[] servos = null;
    class Motor {
        DcMotor drive;
        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    @Override
    public void runOpMode() {
        motors = new Motor[]{new Motor("box", DcMotor.Direction.REVERSE),
                new Motor("arm", DcMotor.Direction.FORWARD)};
        servos = new Servo[] {
                hardwareMap.get(Servo.class, "left"),
                hardwareMap.get(Servo.class, "right")
        };
        waitForStart();
        while (opModeIsActive()) {
            double servo1Pos = servos[0].getPosition();
            double servo2Pos = servos[1].getPosition();
            servo1Pos += gamepad1.left_stick_x*0.002;
            servo2Pos -= gamepad1.left_stick_x*0.002;
            servos[0].setPosition(servo1Pos);
            servos[1].setPosition(servo2Pos);
            motors[1].drive.setPower(0.2*gamepad1.right_stick_x);
        }
    }
}
