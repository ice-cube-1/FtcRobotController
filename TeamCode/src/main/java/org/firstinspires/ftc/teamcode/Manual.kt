package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime

enum class RobotState {INTAKE, SHOOTER_SPIN_UP, SHOOTER_ON}

/**
 * CONTROLS:
 * LEFT / RIGHT joystick -> movement
 * LEFT BUMPER to go into SHOOTER mode (SPIN UP) - only do this if you want to shoot from there
 * RIGHT BUMPER to enter INTAKE mode
 * Triggers -> intake (forwards / back)
 **/

@TeleOp
class Manual : LinearOpMode() {
    private lateinit var shooter: ShooterNew
    //private lateinit var drivetrain: OdometryDrivetrain
    private lateinit var transferIntake: TransferIntake
    private var robotState = RobotState.INTAKE
    private var timeToEnd = 0.0
    private val timer = ElapsedTime()
    override fun runOpMode() {
        waitForStart()
        shooter = ShooterNew(hardwareMap, 20)
        transferIntake = TransferIntake(hardwareMap)
        while (opModeIsActive()) {
            /** expected field centric control **/
            //drivetrain.driveManual(
            //    gamepad1.left_stick_x * MANUAL_MULTIPLIER,
            //    -gamepad1.left_stick_y * MANUAL_MULTIPLIER,
            //    gamepad1.right_stick_x.toDouble()
            //)
            if (gamepad1.left_bumper && timer.milliseconds() > timeToEnd) {
                robotState = RobotState.SHOOTER_SPIN_UP
                transferIntake.prepShooter()
                shooter.turnOnShooter()
                timeToEnd = timer.milliseconds() + 500
            }
            if (gamepad1.right_bumper && timer.milliseconds() > timeToEnd) {
                robotState = RobotState.INTAKE
                transferIntake.shoot(false)
                shooter.turnOffShooter()
                timeToEnd = timer.milliseconds() + 200
            }
            if (robotState == RobotState.INTAKE) {
                transferIntake.intake(gamepad1.left_trigger - gamepad1.right_trigger)
            }
            if (shooter.atSpeed && robotState == RobotState.SHOOTER_SPIN_UP && shooter.canShoot() && timer.milliseconds() > timeToEnd) {
                robotState = RobotState.SHOOTER_ON
                transferIntake.shoot(true)
            }
            transferIntake.update()
            shooter.moveTurret()
            telemetry.addLine(shooter.getData())
            telemetry.addLine(transferIntake.getData())
            telemetry.update()
            shooter.spin()
        }
    }
}