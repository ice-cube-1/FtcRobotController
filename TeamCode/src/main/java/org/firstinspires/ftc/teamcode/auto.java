package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;




@Autonomous(name ="Auto_gyro", group="Robot")
public class auto extends LinearOpMode {
    double robot_length_inches = 18;
    double robot_width_inches = 18;
    private final double speed = 0.5;

    static final double COUNTS_PER_MOTOR_REV = (double) (3249 * 28) / 121 ;   // eg: GoBILDA 223 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 5.51181;   // For figuring circumference

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double P_TURN_GAIN = 0.015;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;


    private Motor[] motors = null;
    private Motor[] elevators = null;
    private Motor arm = null;
    private Motor box = null;
    private IMU imu = null;
    double start_x = 0;
    double start_y = 0;
    private double x = start_x;
    private double y = start_y;
    double initial_rotation = 45;
    private double rotation_deg = initial_rotation;
    private double turnSpeed = 0;
    private double drive_x = 0;
    private double drive_y = 0;
    private double yaw = 0;

    ElevatorState elevator_state = ElevatorState.DOWN;
    private final int elevator_position_up = 1000;
    private final int elevator_position_down = 25;

    ArmState arm_state = ArmState.REST;
    private final int arm_position_basket = 400;
    private final int arm_position_up = 200;
    private final int arm_position_rest = 0;


    enum ElevatorState {
        TO_UP,
        TO_DOWN,
        UP,
        DOWN,
    }

    enum ArmState {
        TO_REST,
        REST,
        TO_UP,
        UP,
        TO_BASKET,
        BASKET,
    }

    @Override
    public void runOpMode() {
        init_stuff();
        /*
        * PUT DRIVE CODE HERE
        * BASE IT ON DRIVE TO POINT, TURN TO POSITION (RELATIVE TO START) CLOCKWISE ETC
        * SHOULD CURRENTLY BE IN INCHES / DEGREES
        */
        elevator_state = ElevatorState.TO_UP;
        driveToPoint(0,12);
        rotate(90, P_TURN_GAIN, true, speed/2);
        while (elevator_state != ElevatorState.UP || arm_state != ArmState.UP) {
            check_elevator_arm();
        }
        arm_state = ArmState.TO_BASKET;
        while (arm_state != ArmState.BASKET) {
            check_elevator_arm();
        }
        dropSample();
        elevator_state = ElevatorState.DOWN;
        driveToPoint(0,0);
        while (arm_state != ArmState.REST || elevator_state != ElevatorState.DOWN) {
            check_elevator_arm();
        }
    }


    void init_stuff() {
        motors = new Motor[]{new Motor("left_front_drive", DcMotor.Direction.REVERSE),
                new Motor("right_front_drive", DcMotor.Direction.FORWARD),
                new Motor("left_back_drive", DcMotor.Direction.REVERSE),
                new Motor("right_back_drive", DcMotor.Direction.FORWARD)};
        elevators = new Motor[]{
                new Motor("left_elevator", DcMotor.Direction.REVERSE),
                new Motor("right_elevator", DcMotor.Direction.FORWARD)
        };
        arm = new Motor("arm", DcMotor.Direction.FORWARD);
        box = new Motor("box", DcMotor.Direction.FORWARD);
        for (Motor elevator: elevators) {
            elevator.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        imu = InitIMU();
        waitForStart();
        for (Motor motor: motors) {
            motor.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        imu.resetYaw();
    }

    void rotate(double degrees, double gain, boolean recursive, double power) {
        if (opModeIsActive()) {
            double delta_deg = degrees - rotation_deg;
            double initialHeading = getHeading();
            double targetHeading = initialHeading + delta_deg;
            while (targetHeading > 180) targetHeading -= 360;
            while (targetHeading <= -180) targetHeading += 360;

            while (opModeIsActive() && Math.abs(getHeading() - targetHeading) > 1) {
                turnSpeed = getSteeringCorrection(targetHeading, gain);
                moveRobot(0, 0, turnSpeed, power);
                check_elevator_arm();
                updateTelemetry();
            }
            moveRobot(0, 0, 0, power);
            rotation_deg = degrees;
            if (recursive) {
                sleep(200);
                rotate(rotation_deg, gain / 2, false, power / 8);
                moveRobot(0, 0, 0, power);
            }
        }

    }

    void dropSample() {
        box.drive.setPower(-0.5);
        sleep(500);
        box.drive.setPower(0);
    }

    void driveToPoint(double new_x, double new_y) {
        double delta_x = new_x - x;
        double delta_y = new_y - y;

        double rotation_radians = Math.toRadians(rotation_deg);

        double transformed_delta_x = delta_x * Math.cos(rotation_radians) + delta_y * Math.sin(rotation_radians);
        double transformed_delta_y = -delta_x * Math.sin(rotation_radians) + delta_y * Math.cos(rotation_radians);

        double crow_flies = Math.sqrt(Math.pow(transformed_delta_x, 2) + Math.pow(transformed_delta_y, 2));
        updateTelemetry();
        driveDistance(transformed_delta_x, transformed_delta_y, crow_flies);
        moveRobot(0,0,0,speed);
        sleep(200);
        rotate(rotation_deg,P_TURN_GAIN/1.5, false, speed/4);
        updateTelemetry();
        x = new_x;
        y = new_y;
        moveRobot(0,0,0,speed);
    }


    void driveDistance(double x, double y, double distance) {
        if (opModeIsActive()) {
            drive_x = x;
            drive_y = y;
            motors[0].target = motors[0].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            motors[1].target = motors[1].drive.getCurrentPosition() + (int)((drive_y - drive_x)  * COUNTS_PER_INCH);
            motors[2].target = motors[2].drive.getCurrentPosition() + (int)((drive_y - drive_x) * COUNTS_PER_INCH);
            motors[3].target = motors[3].drive.getCurrentPosition() + (int)((drive_y + drive_x) * COUNTS_PER_INCH);
            for (Motor motor: motors) {
                motor.drive.setTargetPosition(motor.target);
                motor.drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            moveRobot(drive_x/distance, drive_y/distance, 0, speed);
            while (opModeIsActive() && Arrays.stream(motors).anyMatch(motor -> motor.drive.isBusy())) {
                this.turnSpeed = getSteeringCorrection(rotation_deg, auto.P_DRIVE_GAIN);
                moveRobot(drive_x/distance,drive_y/distance, turnSpeed, speed);
                check_elevator_arm();
                updateTelemetry();
            }
            moveRobot(0,0,0, speed);
            for (Motor motor: motors) {
                motor.drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    void check_elevator_arm() {
        for (Motor elevator : elevators) {
            elevator.position = elevator.drive.getCurrentPosition();
        }
        switch (elevator_state) {
            case TO_UP -> {
                if (elevators[0].position >= elevator_position_up) {
                    elevator_state = ElevatorState.UP;
                } else {
                    for (Motor elevator: elevators) {
                        elevator.drive.setPower(1);
                        if (arm_state == ArmState.REST) {
                            arm_state = ArmState.TO_UP;
                        }
                    }
                }
            } case TO_DOWN -> {
                if (elevators[0].position <= elevator_position_down) {
                    elevator_state = ElevatorState.DOWN;
                } else {
                    for (Motor elevator: elevators) {
                        elevator.drive.setPower(-1);
                        if (arm_state == ArmState.UP || arm_state == ArmState.BASKET) {
                            arm_state = ArmState.TO_REST;
                        }
                    }
                }
            } case DOWN -> {
                for (Motor elevator: elevators) {
                    elevator.drive.setPower(0);
                }
            } case UP -> {
                for (Motor elevator: elevators) {
                    elevator.drive.setPower(0.001);
                }
            }
        }

        double position_difference = elevators[0].position - elevators[1].position;
        double gain = 0.001;
        elevators[0].drive.setPower(elevators[0].drive.getPower() - gain * position_difference);
        elevators[1].drive.setPower(elevators[1].drive.getPower() + gain * position_difference);

        arm.position = arm.drive.getCurrentPosition();
        switch (arm_state) {
            case TO_UP -> {
                if (Math.abs(arm.position - arm_position_up) < 25) {
                    arm_state = ArmState.UP;
                } else {
                    arm.drive.setPower(0.25 * (arm.position - arm_position_up)/Math.abs(arm.position - arm_position_up));
                }
            } case TO_REST -> {
                if (Math.abs(arm.position - arm_position_rest) < 25) {
                    arm_state = ArmState.REST;
                } else {
                    arm.drive.setPower(-0.25);
                }
            } case TO_BASKET -> {
                if (Math.abs(arm.position - arm_position_basket) < 25) {
                    arm_state = ArmState.BASKET;
                } else {
                    arm.drive.setPower(0.25);
                }
            } case BASKET, REST, UP -> arm.drive.setPower(0);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("LF", motors[0].drive.getCurrentPosition());
        telemetry.addData("RF", motors[1].drive.getCurrentPosition());
        telemetry.addData("LB", motors[2].drive.getCurrentPosition());
        telemetry.addData("RB", motors[3].drive.getCurrentPosition());
        telemetry.addData("Turn speed",turnSpeed);
        telemetry.addData("rotation", rotation_deg);
        telemetry.addData("yaw", yaw);
        telemetry.addData("move_x", drive_x);
        telemetry.addData("move_y", drive_y);
        telemetry.addData("LFT", motors[0].target);
        telemetry.addData("RFT", motors[1].target);
        telemetry.addData("LBT", motors[2].target);
        telemetry.addData("RBT", motors[3].target);
        telemetry.update();
    }

    class Motor {
        DcMotor drive;
        double speed = 0;
        int target = 0;
        int position = 0;
        Motor(String name, DcMotorSimple.Direction direction) {
            this.drive = hardwareMap.get(DcMotor.class, name);
            this.drive.setDirection(direction);
            this.drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    void moveRobot(double drive_x, double drive_y, double turn, double power) {
        motors[0].speed = drive_y + drive_x + turn;
        motors[1].speed = drive_y - drive_x - turn;
        motors[2].speed = drive_y - drive_x + turn;
        motors[3].speed = drive_y + drive_x - turn;
        double max = Arrays.stream(motors).mapToDouble(motor -> motor.speed).max().getAsDouble();
        for (Motor motor: motors) {
            motor.speed = motor.speed*power/max;
            motor.drive.setPower(motor.speed);
        }
    }

    double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = getHeading() - desiredHeading;
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    IMU InitIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        return imu;
    }

    double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw(AngleUnit.DEGREES);
        return orientation.getYaw(AngleUnit.DEGREES) - initial_rotation;
    }

}
