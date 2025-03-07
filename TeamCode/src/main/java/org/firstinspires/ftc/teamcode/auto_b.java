package org.firstinspires.ftc.teamcode;

public class auto_b  extends auto {
    @Override
    public void runOpMode() {
        start_x = Math.sqrt(2*Math.pow(robot_length_inches, 2))/2;
        start_y = 100; // CHANGE THIS
        sleep(1000); // CHANGE THIS - DELAY FOR OTHER ROBOT
        initial_rotation = -45;
        int basket_distance_from_corner = 15;
        int field_length = 140;
        init_stuff();
        elevator_state = ElevatorState.ELEVATOR_UP;
        driveToPoint(basket_distance_from_corner, start_y);
        driveToPoint(basket_distance_from_corner, field_length - basket_distance_from_corner);
        while (elevator_state == ElevatorState.ELEVATOR_UP) {check_elevator();}
        dropSample();
        elevator_state = ElevatorState.ELEVATOR_DOWN;
        driveToPoint(robot_length_inches/2 + 2, robot_width_inches*3/2 + 2);
        while (elevator_state == ElevatorState.ELEVATOR_DOWN) {check_elevator();}

    }
}


