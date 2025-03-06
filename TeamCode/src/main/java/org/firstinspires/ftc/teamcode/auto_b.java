package org.firstinspires.ftc.teamcode;

public class auto_b  extends auto {
    @Override
    public void runOpMode() {
        start_x = Math.sqrt(2*Math.pow(robot_length_inches, 2))/2;
        start_y = 50; // CHANGE THIS
        initial_rotation = -45;
        int basket_distance_from_corner = 10;
        init_stuff();
        sleep(5000); // CHANGE THIS - DELAY FOR OTHER ROBOT
        driveToPoint(basket_distance_from_corner, start_y);
        driveToPoint(basket_distance_from_corner, 140 - basket_distance_from_corner);
    }
}
