package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE

@Suppress("ControlFlowWithEmptyBody")
@Autonomous(name="Auto 3 -> Blue", group = "Auto")
class Auto3ArtefactBlue : LinearOpMode() {
    private lateinit var spindexer: Spindexer
    private lateinit var shooter: Shooter
    private lateinit var drivetrain: DriveTrain
    private val timer = ElapsedTime()
    override fun runOpMode() {
        drivetrain = DriveTrain(hardwareMap, 54.0, 135.0,0.0)
        spindexer = Spindexer(hardwareMap)
        shooter = Shooter(hardwareMap)
        waitForStart()
        drivetrain.startDrive(54.0,102.0)
        while (drivetrain.updateDrive() && opModeIsActive()) {}
        drivetrain.stop()
        shooter.setTurretManual(-45.0)
        val motif = shooter.getMotifID()
        shooter.setTurretManual(45.0)
        shooter.turnOnShooter()
        timer.reset()
        while (shooter.turretState != TurretState.DETECTED && timer.seconds() < 5 && opModeIsActive()) {
            shooter.moveTurret(drivetrain.getOrientationDeg())
            shooter.spin()
        }
        /** green is pre loaded into in position zero **/
        if (shooter.turretState == TurretState.DETECTED && shooter.canShoot()) {
            when (motif) {
                21 -> release(arrayOf(0, 1, 2))
                22 -> release(arrayOf(1, 0, 2))
                else -> release(arrayOf(2, 0, 1))
            }
        }
        shooter.turnOffShooter()
        drivetrain.startDrive(54.0,72.0)
        while (drivetrain.updateDrive() && opModeIsActive()) { shooter.moveTurret(drivetrain.getOrientationDeg()) }
        drivetrain.stop()
        drivetrain.startDrive(13.0,72.0)
        while (drivetrain.updateDrive() && opModeIsActive()) { shooter.moveTurret(drivetrain.getOrientationDeg()) }
        drivetrain.stop()
    }
    private fun release(order: Array<Int>) {
        for (i in order) {
            spindexer.release(i)
            timer.reset()
            while (timer.milliseconds() < 400 && opModeIsActive() && !shooter.spin()) {
                shooter.moveTurret(drivetrain.getOrientationDeg())
            }
            spindexer.kickarm.position = KICKARM_RELEASE
            while (timer.milliseconds() < 800 && opModeIsActive()) {
                shooter.spin()
                shooter.moveTurret(drivetrain.getOrientationDeg())
            }
            spindexer.kickarm.position = KICKARM_DOWN
            while (timer.milliseconds() < 1800 && opModeIsActive()) {
                shooter.spin()
                shooter.moveTurret(drivetrain.getOrientationDeg())
            }
            spindexer.removeItem()
        }
    }
}