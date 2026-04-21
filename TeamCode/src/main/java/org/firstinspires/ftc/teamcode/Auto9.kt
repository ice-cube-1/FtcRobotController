package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robotParts.Constants.ROBOT_WIDTH_CM
import org.firstinspires.ftc.teamcode.robotParts.Constants.intakePositions
import org.firstinspires.ftc.teamcode.robotParts.Constants.intake_far_x
import org.firstinspires.ftc.teamcode.robotParts.Constants.numberToCollect
import org.firstinspires.ftc.teamcode.robotParts.Constants.shooter_y
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
        val poses = mutableListOf(
            Triple(offset - direction*(24.0*2.54+ ROBOT_WIDTH_CM/2.0), 84.0*2.54, direction*PI),
            Triple(64.0*2.54, shooter_y*2.54, direction*PI))
        for (i in 0..2) {
            poses.add(Triple(offset-direction*60.0, intakePositions[i], direction*PI))
            poses.add(Triple(offset-direction*60.0, intakePositions[i], direction*PI/2))
            poses.add(Triple(offset-direction*intake_far_x, intakePositions[i], direction*PI/2))
            poses.add(Triple(offset-direction*60.0, shooter_y, direction*PI/2))
            poses.add(Triple(offset-direction*60.0, shooter_y, direction*PI))
        }
        poses.add(Triple(offset-direction*60.0,60.0,direction*PI))
        drivetrain = OdometryDrivetrain(hardwareMap, poses[0].first, poses[0].second, poses[0].third, telemetry)
        shooter = Shooter(hardwareMap, tagID)
        shooter.setSubRange(-45.0+45.0*direction,45.0+45.0*direction)
        transferIntake = TransferIntake(hardwareMap)
        drivetrain.updateGoto(poses[1].first, poses[1].second, poses[1].third)
        waitForStart()
        transferIntake.prepShooter()
        shooter.turnOnShooter()
        if (!driveToPoint()) return
        shoot()
        for (i in 0..<numberToCollect) {
            drivetrain.updateGoto(poses[i*5+2].first,poses[i*5+2].second, poses[i*5+2].third)
            if (!driveToPoint()) return
            drivetrain.updateGoto(poses[i*5+3].first,poses[i*5+3].second, poses[i*5+3].third)
            if (!driveToPoint()) return
            transferIntake.intake(1.0F)
            drivetrain.updateGoto(poses[i*5+4].first,poses[i*5+4].second, poses[i*5+4].third)
            if (!driveToPoint()) return
            transferIntake.intake(0.0F)
            drivetrain.updateGoto(poses[i*5+5].first,poses[i*5+5].second, poses[i*5+5].third)
            if (!driveToPoint()) return
            drivetrain.updateGoto(poses[i*5+6].first,poses[i*5+6].second, poses[i*5+6].third)
            transferIntake.prepShooter()
            if (!driveToPoint()) return
            shoot()
        }
        drivetrain.updateGoto(poses[17].first, poses[17].second, poses[17].third)
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