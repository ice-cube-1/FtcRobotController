package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* start xy are coordinates from far observation zone corner
 *  rotation is anticlockwise, with zero pointing from observation zone -> net zone
 *  general measurements are in inches, tolerance to walls / other robots is 2 inches
 *  april tag has target range inches + rotation (ANTICLOCKWISE) */

@Autonomous(name="Auto E", group="Robot")
public class Auto_E extends Auto {
    @Override
    public void runOpMode() {
        init_stuff(0,0,0);
        driveToPoint(-10,-10);
        driveToPoint(0,0);
        centerOnAprilTag(30,20);
        //rotate(45, P_TURN_GAIN, true, speed);
    }
}