package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robot_constants.*;
import static org.firstinspires.ftc.teamcode.field_constants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* start xy are coordinates from far observation zone corner
 *  rotation is anticlockwise, with zero pointing from observation zone -> net zone
 *  general measurements are in inches, tolerance to walls is 2 inches */

@Autonomous(name="auto_b/c/d", group="auto")
public class auto_b_c_d extends auto {
    @Override
    public void runOpMode() {
        start_x = robot_length_inches/2; // presuming exactly 45 degree angle
        start_y = 80; // CHANGE THIS
        initial_rotation = -90; // PREFERABLY CHANGE THIS? Depends how easy it is to lay the robot out
        int sleep_delay = 1000; // CHANGE THIS - DELAY FOR OTHER ROBOT

        init_stuff();
        sleep(sleep_delay);
        elevator_state = ElevatorState.TO_UP;
        driveToPoint(robot_length_inches, start_y);
        driveToPoint(basket_distance_from_corner, start_y);

        // only required if initial rotation != 45
        rotate(45,P_TURN_GAIN, true, speed/2);

        driveToPoint(basket_distance_from_corner, field_length - basket_distance_from_corner);
        while (elevator_state != ElevatorState.UP /*|| arm_state != ArmState.UP */) {
            check_elevator_arm();
        }
        /*
        arm_state = ArmState.TO_BASKET;
        while (arm_state != ArmState.BASKET) {
            check_elevator_arm();
        }
        dropSample();
         */
        rotate(90, P_TURN_GAIN, true, speed/2);
        elevator_state = ElevatorState.TO_DOWN;

        // This is if the other robot does not move, so avoids them (CALIBRATE)
        driveToPoint(robot_length_inches*3/2 + 5, field_length-basket_distance_from_corner);
        driveToPoint(robot_length_inches*3/2 + 5, robot_length_inches + 5);

        // extra tolerance to not hit other robot (change dependent on other robot's parking mechanism
        // and if we park first)
        driveToPoint(robot_length_inches/2 + 5, robot_length_inches + 5);
        while (elevator_state != ElevatorState.DOWN /* || arm_state != ArmState.REST*/ ) {
            check_elevator_arm();
        }
    }
}


