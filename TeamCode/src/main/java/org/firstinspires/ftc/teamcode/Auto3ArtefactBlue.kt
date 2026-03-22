package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE


@Autonomous(name="Auto 3 - BLUE", group = "Auto")
class Auto3ArtefactBlue : LinearOpMode() {
    private lateinit var spindexer: Spindexer
    private lateinit var shooter: Shooter
    private lateinit var drivetrain: DriveTrain
    private val timer = ElapsedTime()
    override fun runOpMode() {
        shooter = Shooter(hardwareMap)
        shooter.setStart(20, false)
        drivetrain = DriveTrain(hardwareMap)
        drivetrain.setStart(33.0, 135.0,0.0)
        spindexer = Spindexer(hardwareMap)
        spindexer.release(0)
        waitForStart()
        drivetrain.startDrive(33.0,102.0)
        while (!drivetrain.updateDrive()) {}
        drivetrain.stop()
        drivetrain.startDrive(54.0,102.0)
        while (!drivetrain.updateDrive()) {}
        drivetrain.stop()
        var motif = -1
        /** shooter.setTurretManual(-45.0)
        while (motif == -1 && timer.milliseconds() < 500) { motif = shooter.getMotifID() } **/
        shooter.setTurretManual(45.0)
        shooter.turnOnShooter()
        timer.reset()
        while (timer.milliseconds() < 3000) { shooter.spin() }
        if (shooter.turretState == TurretState.DETECTED && shooter.canShoot()) {
            when (motif) {
                21 -> release(arrayOf(0, 1, 2))
                22 -> release(arrayOf(1, 0, 2))
                else -> release(arrayOf(1, 2, 0))
            }
        }
        shooter.turnOffShooter()
        shooter.setTurretManual(0.0)
        drivetrain.startDrive(54.0,72.0)
        while (!drivetrain.updateDrive() && opModeIsActive()) {}
        drivetrain.stop()
        shooter.setTurretManual(0.0)
    }
    fun release(order: Array<Int>) {
        for (i in order) {
            spindexer.release(i)
            timer.reset()
            while (timer.seconds() < 2) { shooter.spin() }
            spindexer.moveKickarm(KICKARM_RELEASE)
            while (timer.seconds() < 3) { shooter.spin() }
            spindexer.moveKickarm(KICKARM_DOWN)
            while (timer.seconds() < 5) { shooter.spin() }
        }
    }
}