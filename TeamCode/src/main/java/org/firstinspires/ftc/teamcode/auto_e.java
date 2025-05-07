package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robot_constants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* start xy are coordinates from far observation zone corner
 *  rotation is anticlockwise, with zero pointing from observation zone -> net zone
 *  general measurements are in inches, tolerance to walls / other robots is 2 inches */

@Autonomous(name="auto_e", group="Robot")
public class auto_e extends auto_new {
    @Override
    public void runOpMode() {
        start_x = 0;
        start_y = 0; // CHANGE THIS
        initial_rotation = 0; // CHANGE THIS
        init_stuff();
        driveToPoint(-10,-10);
        driveToPoint(0,0);
        centerOnAprilTag(30,20);
        //rotate(45, P_TURN_GAIN, true, speed);
    }
}