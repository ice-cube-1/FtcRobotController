package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*  general measurements are in inches, tolerance to walls / other robots is 2 inches
 *  april tag has target range inches + rotation (ANTICLOCKWISE) */

@Config
@Autonomous(name="Auto park", group="Robot")
public class Auto_Park extends Auto {
    @Override
    public void runOpMode() {
        init_stuff(0,0,0);
        driveToPoint(0,5);
        driveToPoint(43, 5);
    }
}