package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OtherMotors {
    private final FSM_motor<Elevator> elevator;
    private final FSM_motor<Arm> arm;
    
    OtherMotors(HardwareMap hardwareMap) {
        elevator = new FSM_motor<>(new Motor(hardwareMap, "elevator", DcMotorSimple.Direction.FORWARD), Elevator.STOP);
        arm = new FSM_motor<>(new Motor(hardwareMap, "arm", DcMotorSimple.Direction.FORWARD), Arm.IN);
    }

    static class FSM_motor<T extends Enum<T>> {
        Motor motor;
        T state;
        FSM_motor(Motor motor, T state) {
            this.motor = motor;
            this.state = state;
        }
    }

    public enum Elevator { HOLD, UP, DOWN, STOP }
    public enum Arm { GOING_OUT, OUT, GOING_IN, IN }

    void check_FSMs() {
        switch (elevator.state) {
            case UP -> {
                if (Math.abs(elevator.motor.drive.getCurrentPosition() - elevator.motor.target) <= encoder_tolerance) {
                    elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    elevator.state = Elevator.HOLD;
                }
                else elevator.motor.drive.setPower(speed);
            }
            case DOWN -> {
                if (Math.abs(elevator.motor.drive.getCurrentPosition() - elevator.motor.target) <= encoder_tolerance) {
                    elevator.state = Elevator.STOP;
                    elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                else elevator.motor.drive.setPower(-speed);
            }
            case STOP -> elevator.motor.drive.setPower(0);
        }
        switch(arm.state) {
            case IN, OUT -> arm.motor.drive.setPower(0);
            case GOING_OUT -> {
                if (Math.abs(arm.motor.drive.getCurrentPosition() - arm.motor.target) <= encoder_tolerance) {
                    arm.state = Arm.OUT;
                }
                else arm.motor.drive.setPower(speed);
            }
            case GOING_IN -> {
                if (Math.abs(arm.motor.drive.getCurrentPosition() - arm.motor.target) <= encoder_tolerance) {
                    arm.state = Arm.IN;
                }
                else arm.motor.drive.setPower(-speed);
            }
        }
    }

}
