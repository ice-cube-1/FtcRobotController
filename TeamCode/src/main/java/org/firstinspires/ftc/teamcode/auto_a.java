package org.firstinspires.ftc.teamcode;

public class auto_a  extends auto {
    @Override
    public void runOpMode() {
        start_x = robot_length_inches/2;
        start_y = 50; // CHANGE THIS
        init_stuff();
        driveToPoint(robot_length_inches/2 + 2, robot_width_inches/2 + 2);
    }
}
