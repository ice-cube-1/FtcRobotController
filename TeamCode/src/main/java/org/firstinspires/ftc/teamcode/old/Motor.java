package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    DcMotor drive;
    double speed = 0;
    int target = 0;

    public Motor(HardwareMap hwMap, String name, DcMotorSimple.Direction direction) {
        this.drive = hwMap.get(DcMotor.class, name);
        this.drive.setDirection(direction);
        this.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
