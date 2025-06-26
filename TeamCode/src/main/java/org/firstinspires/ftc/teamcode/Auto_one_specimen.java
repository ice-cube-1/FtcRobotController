package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.pincer_rotation_time;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "auto one specimen", group = "robot")
public class Auto_one_specimen extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        // starting 2 tiles away, centered on the line
        init_stuff(0,0,0);
        otherMotors.auto = true;
        otherMotors.pincer.state = OtherMotors.ServoState.CLOSED;
        //otherMotors.elevator.state = OtherMotors.MotorState.GOING_OUT;
        otherMotors.pincer_rotation.state = OtherMotors.MotorState.GOING_OUT;
        otherMotors.pincer_rotation.target_time = otherMotors.timer.milliseconds() + pincer_rotation_time;
        driveToPoint(-18,9);
        rotate(-45);
        while (otherMotors.elevator.state != OtherMotors.MotorState.OUT
                && otherMotors.pincer_rotation.state != OtherMotors.MotorState.OUT) {
            otherMotors.check_FSMs();
        }
        otherMotors.pincer.state = OtherMotors.ServoState.OPEN;
        otherMotors.check_FSMs();
        sleep(1000);
        rotate(0);
        driveToPoint(0,0); // delete
        //driveToPoint(96, 5);
    }
}
