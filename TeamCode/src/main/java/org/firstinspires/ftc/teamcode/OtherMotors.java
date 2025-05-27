package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OtherMotors.Arm.IN;
import static org.firstinspires.ftc.teamcode.OtherMotors.Elevator.DOWN;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OtherMotors {
    private FSM_motor<Elevator> elevator;
    private FSM_motor<Arm> arm;
    
    OtherMotors() {
        elevator = new FSM_motor<>(new Motor("elevator", DcMotorSimple.Direction.FORWARD), DOWN);
        arm = new FSM_motor<>(new Motor("arm", DcMotorSimple.Direction.FORWARD), IN);
    }

    static class FSM_motor<T extends Enum<T>> {
        Motor motor;
        T state;
        FSM_motor(Motor motor, T state) {
            this.motor = motor;
            this.state = state;
        }
    }

    public enum Elevator { GOING_UP, UP, DOWN }
    public enum Arm { GOING_OUT, OUT, GOING_IN, IN }

    void check_FSMs() {
        switch (elevator.state) {
            case GOING_UP -> elevator.motor.drive.setPower(speed);
            case UP -> elevator.motor.drive.setPower(0.001);
            case DOWN -> elevator.motor.drive.setPower(0);
        }
        switch(arm.state) {
            case IN, OUT -> arm.motor.drive.setPower(0);
            case GOING_OUT -> arm.motor.drive.setPower(speed);
            case GOING_IN -> arm.motor.drive.setPower(-speed);
        }
    }
}
