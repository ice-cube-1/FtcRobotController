package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="auto_test", group="auto")
public class auto_test extends auto {
    @Override
    public void runOpMode() {
        start_x = 0;
        start_y = 0;
        initial_rotation = 0;

        init_stuff();

        driveToPoint(0,-120);
    }
}