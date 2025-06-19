package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.speed;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class Drivetrain {
    Motor[] motors;
    double x, y, rotation_deg;

    Drivetrain(HardwareMap hardwareMap, double start_x, double start_y, double initial_rotation) {
        motors = new Motor[] {new Motor(hardwareMap, "left_front_drive", DcMotor.Direction.REVERSE),
                new Motor(hardwareMap, "left_back_drive", DcMotor.Direction.REVERSE),
                new Motor(hardwareMap, "right_front_drive", DcMotor.Direction.FORWARD),
                new Motor(hardwareMap, "right_back_drive", DcMotor.Direction.FORWARD)};
    }

    void move(double drive_x, double drive_y, double turn) {
        motors[0].speed = drive_y - drive_x - turn;
        motors[1].speed = drive_y + drive_x + turn;
        motors[2].speed = drive_y + drive_x - turn;
        motors[3].speed = drive_y - drive_x + turn;
        double max = Arrays.stream(motors).mapToDouble(motor -> motor.speed).max().getAsDouble();
        for (Motor motor: motors) {
            motor.speed = motor.speed * speed / max;
            motor.drive.setPower(motor.speed);
        }
    }


}
