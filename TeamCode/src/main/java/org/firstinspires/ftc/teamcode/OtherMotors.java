package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.RobotConstants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OtherMotors {
    final FSM_motor<MotorState> elevator;
    final FSM_motor<MotorState> arm;
    final FSM_CR_servo<MotorState> pincer_rotation;
    final FSM_servo<ServoState> pincer;
    final FSM_CR_servo<MotorState> claw_rotation;
    final FSM_CR_servo<StarState> spinning_star_a;
    final CRServo spinning_star_b;
    ElapsedTime timer = new ElapsedTime();
    boolean auto = false;

    OtherMotors(HardwareMap hardwareMap) {
        elevator = new FSM_motor<>(new Motor(hardwareMap, "elevator", DcMotorSimple.Direction.FORWARD), MotorState.IN);
        arm = new FSM_motor<>(new Motor(hardwareMap, "arm", DcMotorSimple.Direction.FORWARD), MotorState.IN);
        pincer_rotation = new FSM_CR_servo<>(hardwareMap.get(CRServo.class, "pincer_rotation"), MotorState.IN);
        claw_rotation = new FSM_CR_servo<>(hardwareMap.get(CRServo.class, "claw_rotation"), MotorState.IN);
        spinning_star_a = new FSM_CR_servo<>(hardwareMap.get(CRServo.class, "spinning_star_a"), StarState.STOP);
        spinning_star_b = hardwareMap.get(CRServo.class, "spinning_star_b");
        pincer = new FSM_servo<>(hardwareMap.get(Servo.class, "pincer"), ServoState.OPEN);
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
    public enum ServoState { OPEN, CLOSED }

    void check_FSMs() {
        switch (elevator.state) {
            case GOING_OUT -> {
                if (auto && Math.abs(elevator.motor.drive.getCurrentPosition() - elevator.motor.target) <= encoder_tolerance) {
                    elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    elevator.state = MotorState.OUT;
                    break;
                }
                elevator.motor.drive.setPower(speed);
            }
            case GOING_IN -> {
                if (auto && Math.abs(elevator.motor.drive.getCurrentPosition() - elevator.motor.target) <= encoder_tolerance) {
                    elevator.state = MotorState.IN;
                    elevator.motor.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    break;
                }
                elevator.motor.drive.setPower(-speed);
            }
            case IN -> elevator.motor.drive.setPower(0);

            case OUT -> {
                if (Math.abs(elevator.motor.drive.getCurrentPosition() - elevator.motor.target) <= encoder_tolerance) {
                    elevator.motor.drive.setPower(0);
                } else if (Math.abs(elevator.motor.drive.getCurrentPosition()) < elevator.motor.target) {
                    elevator.motor.drive.setPower(0.01);
                }
            }
        }

        switch(arm.state) {
            case IN, OUT -> arm.motor.drive.setPower(0);
            case GOING_OUT -> {
                if (auto && Math.abs(arm.motor.drive.getCurrentPosition() - arm.motor.target) <= encoder_tolerance) {
                    arm.state = MotorState.OUT;
                    break;
                }
                arm.motor.drive.setPower(speed);
            }
            case GOING_IN -> {
                if (auto && Math.abs(arm.motor.drive.getCurrentPosition() - arm.motor.target) <= encoder_tolerance) {
                    arm.state = MotorState.IN;
                    break;
                }
                arm.motor.drive.setPower(-speed);
            }
        }
        switch (pincer_rotation.state) {
            case IN, OUT -> claw_rotation.servo.setPower(0.0);
            case GOING_IN -> claw_rotation.servo.setPower(speed);
            case GOING_OUT -> claw_rotation.servo.setPower(-speed);
        }
        switch (claw_rotation.state) {
            case IN, OUT -> claw_rotation.servo.setPower(0.0);
            case GOING_IN -> {
                if (auto && timer.milliseconds() > claw_rotation.target_time) {
                    claw_rotation.state = MotorState.IN;
                    break;
                }
                claw_rotation.servo.setPower(speed);
            }
            case GOING_OUT -> {
                if (auto && timer.milliseconds() > claw_rotation.target_time) {
                    claw_rotation.state = MotorState.OUT;
                    break;
                }
                claw_rotation.servo.setPower(-speed);
            }
        }
        switch (spinning_star_a.state) {
            case STOP -> {
                spinning_star_a.servo.setPower(0.0);
                spinning_star_b.setPower(0.0);
            }
            case IN -> {
                if (auto && timer.milliseconds() > spinning_star_a.target_time) {
                    spinning_star_a.state = StarState.STOP;
                    break;
                }
                spinning_star_a.servo.setPower(speed);
                spinning_star_b.setPower(-speed);
            }
            case OUT -> {
                if (auto && timer.milliseconds() > spinning_star_a.target_time) {
                    spinning_star_a.state = StarState.STOP;
                    break;
                }
                spinning_star_a.servo.setPower(-speed);
                spinning_star_b.setPower(speed);
            }
        }
        switch (pincer.state) {
            case OPEN -> pincer.servo.setPosition(pincer_open_pos);
            case CLOSED -> pincer.servo.setPosition(pincer_closed_pos);
        }
    }
}
