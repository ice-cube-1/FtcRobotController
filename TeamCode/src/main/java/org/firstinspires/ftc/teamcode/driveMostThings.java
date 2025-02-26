/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.round;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;


@TeleOp(name="Drive  all things", group="Linear OpMode")
public class driveMostThings extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftElevator = null;
    private DcMotor rightElevator = null;
    private DcMotor box = null;
    private DcMotor arm = null;
    private int triggerPosition = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftElevator = hardwareMap.get(DcMotor.class, "left_elevator");
        rightElevator = hardwareMap.get(DcMotor.class, "right_elevator");
        box = hardwareMap.get(DcMotor.class, "box");
        arm = hardwareMap.get(DcMotor.class, "arm");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftElevator.setDirection(DcMotor.Direction.FORWARD);
        rightElevator.setDirection(DcMotor.Direction.REVERSE);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double leftFrontEncoder = 0;
        double leftFrontPowerMultiple = 0;
        double leftBackEncoder = 0;
        double leftBackPowerMultiple = 0;
        double rightFrontEncoder = 0;
        double rightFrontPowerMultiple = 0;
        double rightBackEncoder = 0;
        double rightBackPowerMultiple = 0;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            if (round(gamepad1.left_trigger) != triggerPosition) {
                triggerPosition = 1-triggerPosition;
                box.setPower(triggerPosition-0.5);
                sleep(1000);
                box.setPower(0);
            }
            while (gamepad1.dpad_up || gamepad1.dpad_down) {
                int direction = 0;
                if (gamepad1.dpad_down) {
                    direction=1;
                }
                leftElevator.setPower(direction-0.5);
                rightElevator.setPower(direction-0.5);
            }

            //leftElevator.setPower(0.0005);
            //rightElevator.setPower(0.0005);

            int[] encoderValues = {leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition()};
            leftFrontPowerMultiple = Math.abs(encoderValues[0]-leftFrontEncoder);
            leftBackPowerMultiple = Math.abs(encoderValues[1]-leftBackEncoder)/2;
            rightFrontPowerMultiple = Math.abs(encoderValues[2]-rightFrontEncoder);
            rightBackPowerMultiple = Math.abs(encoderValues[3]-rightBackEncoder)/2;
            leftFrontEncoder = encoderValues[0];
            leftBackEncoder = encoderValues[1];
            rightFrontEncoder = encoderValues[2];
            rightBackEncoder = encoderValues[3];

            double actual = (leftFrontPowerMultiple+leftBackPowerMultiple+rightFrontPowerMultiple+rightBackPowerMultiple)/4;

            if (actual==0) {
                actual = 1;
                leftFrontEncoder = 1;
                leftBackEncoder = 1;
                rightFrontEncoder = 1;
                rightBackEncoder = 1;
            }


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.right_stick_x;
            double yaw     =  gamepad1.left_stick_x;
            int leftMultiple = 1;
            int rightMultiple = 1;
            if (Math.abs(yaw) > Math.abs(axial)) {
                axial = 0;
                if (yaw > 0) {
                    rightMultiple = 2;
                } else {
                    leftMultiple = 2;
                }
            } else {
                yaw = 0;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial + lateral + yaw)/leftFrontPowerMultiple*actual;
            double rightFrontPower = (axial - lateral - yaw)/rightFrontPowerMultiple*actual;
            double leftBackPower   = (axial - lateral + yaw)/leftBackPowerMultiple*actual;
            double rightBackPower  = (axial + lateral - yaw)/rightBackPowerMultiple*actual;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontDrive.setPower(leftFrontPower*1);
            rightFrontDrive.setPower(rightFrontPower*0.5);
            leftBackDrive.setPower(leftBackPower*0.5);
            rightBackDrive.setPower(rightBackPower*1);
            // arm.setPower(-gamepad1.right_stick_y*0.25);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + actual + Arrays.toString(encoderValues));
            telemetry.addData("LF", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RF", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LB", leftBackDrive.getCurrentPosition());
            telemetry.addData("RB", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }}
