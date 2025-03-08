package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robot_constants.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* start xy are coordinates from far observation zone corner
 *  rotation is anticlockwise, with zero pointing from observation zone -> net zone
 *  general measurements are in inches, tolerance to walls is 2 inches */

@TeleOp(name="auto_c", group="Robot")
public class auto_c extends auto {
    @Override
    public void runOpMode() {
        start_x = robot_length_inches/2;
        start_y = 50; // CHANGE THIS
        init_stuff();
        driveToPoint(robot_length_inches/2 + 2, robot_length_inches/2 + 2);
    }
}