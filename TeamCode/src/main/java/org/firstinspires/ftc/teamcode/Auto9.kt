package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robotParts.Constants.ROBOT_WIDTH_CM
import org.firstinspires.ftc.teamcode.robotParts.Constants.Robot_LENGTH_CM
import org.firstinspires.ftc.teamcode.robotParts.OdometryDrivetrain
import org.firstinspires.ftc.teamcode.robotParts.Shooter
import org.firstinspires.ftc.teamcode.robotParts.TransferIntake
import kotlin.math.PI

abstract class Auto9(private val offset: Double, private val direction: Double, private val tagID: Int) : LinearOpMode() {
    private lateinit var drivetrain: OdometryDrivetrain
    private lateinit var shooter: Shooter
    private lateinit var transferIntake: TransferIntake
    private val atSpeed = ElapsedTime()
    private val timer = ElapsedTime()
    override fun runOpMode() {
        drivetrain = OdometryDrivetrain(hardwareMap,
            offset-direction*(24.0*2.54+ ROBOT_WIDTH_CM/2.0),
            144.0*2.54 - Robot_LENGTH_CM/2.0,
            direction*PI)
        shooter = Shooter(hardwareMap, tagID)
        shooter.setSubRange(-90.0,0.0)
        transferIntake = TransferIntake(hardwareMap)
        waitForStart()
        drivetrain.updateGoto(offset-direction*(64.0*2.54), 84.0*2.54,direction*PI)
        transferIntake.prepShooter()
        shooter.turnOnShooter()
        if (!driveToPoint()) return
        shoot()
        drivetrain.updateGoto(offset-direction*(64.0*2.54), 84.0*2.54,direction*PI/2)
        if (!driveToPoint()) return
        transferIntake.intake(1.0F)
        drivetrain.updateGoto(offset-direction*(18.0*2.54),84.0*2.54, direction*PI/2)
        if (!driveToPoint()) return
        transferIntake.intake(0.0F)
        drivetrain.updateGoto(offset-direction*(60.0*2.54),84.0*2.54, direction*PI/2)
        if (!driveToPoint()) return
        transferIntake.prepShooter()
        drivetrain.updateGoto(offset-direction*(60.0*2.54),84.0*2.54, direction*PI)
        if (!driveToPoint()) return
        shoot()
        drivetrain.updateGoto(offset-direction*(60.0 * 2.54), 58.0*2.54, direction*PI)
        if (!driveToPoint()) return
        drivetrain.updateGoto(offset-direction*(60.0 * 2.54), 58.0*2.54, direction*PI/2)
        if (!driveToPoint()) return
        transferIntake.intake(1.0F)
        drivetrain.updateGoto(offset-direction*(18.0*2.54),58.0*2.54, direction*PI/2)
        if (!driveToPoint()) return
        transferIntake.intake(0.0F)
        drivetrain.updateGoto(offset-direction*(60.0*2.54),84.0*2.54, direction*PI/2)
        if (!driveToPoint()) return
        transferIntake.prepShooter()
        drivetrain.updateGoto(offset-direction*(60.0*2.54),84.0*2.54, direction*PI)
        if (!driveToPoint()) return
        shoot()
        drivetrain.updateGoto(offset-direction*(60.0*2.54),60.0*2.54,direction*PI)
        if (!driveToPoint()) return
        drivetrain.updateGoto(offset-direction*(24.0*2.54 + ROBOT_WIDTH_CM/2.0),
            144.0*2.54 - Robot_LENGTH_CM/2.0,
            direction*PI)
        if (!driveToPoint()) return
    }
    private fun shoot() {
        timer.reset()
        while (opModeIsActive() && timer.milliseconds() < 3000) {
            updateAllNonDrivetrain()
            if (atSpeed.milliseconds() > 50 && shooter.canShoot()) {
                transferIntake.shoot(true)
                break
            }
        }
        timer.reset()
        while (opModeIsActive() && timer.milliseconds() < 2000) { updateAllNonDrivetrain() }
        transferIntake.shoot(false)
    }
    private fun driveToPoint() : Boolean {
        while (opModeIsActive()  && !drivetrain.continueDriving()) { updateAllNonDrivetrain() }
        timer.reset()
        while (opModeIsActive() && (timer.milliseconds() < 50 || !drivetrain.continueDriving())) { updateAllNonDrivetrain() }
        return opModeIsActive()
    }
    private fun updateAllNonDrivetrain() {
        shooter.moveTurret()
        shooter.spin()
        transferIntake.update()
        telemetry.addLine(shooter.getData())
        telemetry.addLine(transferIntake.getData())
        telemetry.addLine(drivetrain.getData())
        telemetry.update()
        if (!shooter.atSpeed) atSpeed.reset()
    }
}