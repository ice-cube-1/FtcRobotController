package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Motor {
    DcMotor drive;
    double speed = 0;
    int target = 0;
    Motor(String name, DcMotorSimple.Direction direction) {
        this.drive = hardwareMap.get(DcMotor.class, name);
        this.drive.setDirection(direction);
        this.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
