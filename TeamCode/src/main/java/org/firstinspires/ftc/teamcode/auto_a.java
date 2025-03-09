package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robot_constants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* start xy are coordinates from far observation zone corner
 *  rotation is anticlockwise, with zero pointing from observation zone -> net zone
 *  general measurements are in inches, tolerance to walls / other robots is 2 inches */

@Autonomous(name="auto_a", group="Robot")
public class auto_a extends auto {
    @Override
    public void runOpMode() {
        start_x = robot_length_inches/2;
        start_y = 20; // CHANGE THIS
        initial_rotation = -90; // CHANGE THIS
        init_stuff();
        driveToPoint(robot_length_inches/2 + 2,start_y);
        driveToPoint(robot_length_inches/2 + 2, robot_length_inches/2 + 2);
    }
}