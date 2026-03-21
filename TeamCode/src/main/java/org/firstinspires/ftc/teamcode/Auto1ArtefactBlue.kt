package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE


@Autonomous(name="Auto 1 - BLUE", group = "Auto")
class Auto1ArtefactBlue : LinearOpMode() {
    private lateinit var spindexer: Spindexer
    private lateinit var shooter: Shooter
    private lateinit var drivetrain: DriveTrain
    private val timer = ElapsedTime()
    override fun runOpMode() {
        shooter = Shooter(hardwareMap)
        shooter.setStart(20)
        drivetrain = DriveTrain(hardwareMap)
        drivetrain.setStart(54.0, 135.0,0.0)
        spindexer = Spindexer(hardwareMap)
        spindexer.release(0)
        waitForStart()
        drivetrain.startDrive(54.0,102.0)
        while (!drivetrain.updateDrive()) {}
        drivetrain.stop()
        shooter.setTurretManual(45.0)
        timer.reset()
        while (shooter.turretState != TurretState.DETECTED && timer.seconds() < 3) { shooter.moveTurret() }
        shooter.turnOnShooter()
        while (timer.seconds() < 7) { shooter.spin() }
        spindexer.moveKickarm(KICKARM_RELEASE)
        while (timer.seconds() < 9) { shooter.spin() }
        spindexer.moveKickarm(KICKARM_DOWN)
        while (timer.seconds() < 10) {
            shooter.spin()
            shooter.moveTurret()
        }
        shooter.turnOffShooter()
        shooter.setTurretManual(0.0)
        drivetrain.startDrive(54.0,72.0)
        while (!drivetrain.updateDrive() && opModeIsActive()) {}
        drivetrain.stop()
        shooter.setTurretManual(0.0)
    }
}