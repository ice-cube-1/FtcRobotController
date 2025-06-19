package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OtherMotors {
    final FSM_motor<MotorState> elevator;
    //final FSM_motor<MotorState> arm;
    //final FSM_servo<ServoState> pincer_rotation;
    //final FSM_CR_servo<MotorState> claw_rotation;
    //final FSM_CR_servo<StarState> spinning_star_a;
    //final CRServo spinning_star_b;
    //ElapsedTime timer = new ElapsedTime();

    OtherMotors(HardwareMap hardwareMap) {
        elevator = new FSM_motor<>(new Motor(hardwareMap, "elevator", DcMotorSimple.Direction.FORWARD), MotorState.IN);
        //arm = new FSM_motor<>(new Motor(hardwareMap, "arm", DcMotorSimple.Direction.FORWARD), MotorState.IN);
        //pincer_rotation = new FSM_servo<>(hardwareMap.get(Servo.class, "pincer_rotation"), ServoState.DOWN);
        //claw_rotation = new FSM_CR_servo<>(hardwareMap.get(CRServo.class, "claw_rotation"), MotorState.IN);
        //spinning_star_a = new FSM_CR_servo<>(hardwareMap.get(CRServo.class, "spinning_star_a"), StarState.STOP);
        //spinning_star_b = hardwareMap.get(CRServo.class, "spinning_star_b");
    }

    static class FSM_servo<T extends Enum<T>> {
        Servo servo;
        T state;
        FSM_servo(Servo servo, T state) {
            this.servo = servo;
            this.state = state;
        }
    }

    static class FSM_CR_servo<T extends Enum<T>> {
        CRServo servo;
        T state;
        Double target_time;
        FSM_CR_servo(CRServo servo, T state) {
            this.servo = servo;
            this.state = state;
            this.target_time = 0.0;
        }
    }

    static class FSM_motor<T extends Enum<T>> {
        Motor motor;
        T state;
        FSM_motor(Motor motor, T state) {
            this.motor = motor;
            this.state = state;
        }
    }

    public enum MotorState { GOING_OUT, OUT, GOING_IN, IN }
    public enum StarState { IN, OUT, STOP }
    public enum ServoState { UP, DOWN }

    void check_FSMs() {
        switch (elevator.state) {
            case GOING_OUT -> {
                if (Math.abs(elevator.motor.drive.getCurrentPosition() - elevator.motor.target) <= encoder_tolerance) {
                    elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    elevator.state = MotorState.OUT;
                }
                else elevator.motor.drive.setPower(speed);
            }
            case GOING_IN -> {
                if (Math.abs(elevator.motor.drive.getCurrentPosition() - elevator.motor.target) <= encoder_tolerance) {
                    elevator.state = MotorState.IN;
                    elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                else elevator.motor.drive.setPower(-speed);
            }
            case IN -> elevator.motor.drive.setPower(0);

            case OUT -> {
                if (Math.abs(elevator.motor.drive.getCurrentPosition() - elevator.motor.target) <= encoder_tolerance) {
                    elevator.motor.drive.setPower(0);
                } if (Math.abs(elevator.motor.drive.getCurrentPosition()) < elevator.motor.target) {
                    elevator.motor.drive.setPower(0.01);
                }
            }
        }

        /*
        switch(arm.state) {
            case IN, OUT -> arm.motor.drive.setPower(0);
            case GOING_OUT -> {
                if (Math.abs(arm.motor.drive.getCurrentPosition() - arm.motor.target) <= encoder_tolerance) {
                    arm.state = MotorState.OUT;
                }
                else arm.motor.drive.setPower(speed);
            }
            case GOING_IN -> {
                if (Math.abs(arm.motor.drive.getCurrentPosition() - arm.motor.target) <= encoder_tolerance) {
                    arm.state = MotorState.IN;
                }
                else arm.motor.drive.setPower(-speed);
            }
        }
        switch (pincer_rotation.state) {
            case UP -> pincer_rotation.servo.setPosition(pincer_in_pos/180.0);
            case DOWN -> pincer_rotation.servo.setPosition(pincer_out_pos/180.0);
        }
        switch (claw_rotation.state) {
            case IN, OUT -> claw_rotation.servo.setPower(0.0);
            case GOING_IN -> {
                if (timer.milliseconds() > claw_rotation.target_time) {
                    claw_rotation.state = MotorState.IN;
                }
                else claw_rotation.servo.setPower(speed);
            }
            case GOING_OUT -> {
                if (timer.milliseconds() > claw_rotation.target_time) {
                    claw_rotation.state = MotorState.OUT;
                }
                else claw_rotation.servo.setPower(-speed);
            }
        }
        switch (spinning_star_a.state) {
            case STOP -> {
                spinning_star_a.servo.setPower(0.0);
                spinning_star_b.setPower(0.0);
            }
            case IN -> {
                if (timer.milliseconds() > spinning_star_a.target_time) {
                    spinning_star_a.state = StarState.STOP;
                }
                spinning_star_a.servo.setPower(speed);
                spinning_star_b.setPower(-speed);
            }
            case OUT -> {
                spinning_star_a.servo.setPower(-speed);
                spinning_star_b.setPower(speed);
                if (timer.milliseconds() > spinning_star_a.target_time) {
                    spinning_star_a.state = StarState.STOP;
                }
            }
        }
        */
    }

}
