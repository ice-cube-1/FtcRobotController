package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE

enum class Alliance {RED, BLUE}

@Autonomous(name="Auto 3", group = "Auto")
class Auto3Artefact : LinearOpMode() {
    private lateinit var spindexer: Spindexer
    private lateinit var shooter: Shooter
    private lateinit var drivetrain: DriveTrain
    private val timer = ElapsedTime()
    private lateinit var alliance: Alliance
    override fun runOpMode() {
        var initialised = false
        telemetry.addLine("Dpad LEFT for BLUE alliance, RIGHT for RED")
        telemetry.update()
        while (!initialised && opModeInInit()) {
            if (gamepad1.dpad_left) {
                alliance = Alliance.BLUE
                initialised = true
                telemetry.addLine("BLUE ALLIANCE")
            }
            if (gamepad1.dpad_right) {
                alliance = Alliance.RED
                initialised = true
                telemetry.addLine("RED ALLIANCE")
            }
            telemetry.update()
        }
        shooter = Shooter(hardwareMap)
        shooter.setStart(if (alliance ==  Alliance.BLUE) 20 else 24, false)
        drivetrain = DriveTrain(hardwareMap)
        drivetrain.setStart(if (alliance == Alliance.BLUE) 54.0 else 90.0, 135.0,0.0)
        spindexer = Spindexer(hardwareMap)
        waitForStart()
        drivetrain.startDrive(if (alliance == Alliance.BLUE) 54.0 else 90.0,102.0)
        while (drivetrain.updateDrive() && opModeIsActive()) {}
        drivetrain.stop()
        shooter.setTurretManual(if (alliance == Alliance.BLUE) -45.0 else 45.0)
        timer.reset()
        var motif = -1
        while (motif == -1 && timer.milliseconds() < 500) { motif = shooter.getMotifID() }
        shooter.setTurretManual(if (alliance == Alliance.BLUE) 45.0 else -45.0)
        shooter.turnOnShooter()
        timer.reset()
        while (shooter.turretState != TurretState.DETECTED && timer.seconds() < 5 && opModeIsActive()) {
            shooter.moveTurret()
            shooter.spin()
        }
        /** green is pre loaded into in position 0 **/
        if (shooter.turretState == TurretState.DETECTED && shooter.canShoot()) {
            when (motif) {
                21 -> release(arrayOf(0, 1, 2))
                22 -> release(arrayOf(1, 0, 2))
                else -> release(arrayOf(1, 2, 0))
            }
        }
        shooter.turnOffShooter()
        drivetrain.startDrive(if (alliance == Alliance.BLUE) 54.0 else 90.0,72.0)
        while (drivetrain.updateDrive() && opModeIsActive()) { shooter.moveTurret() }
        drivetrain.stop()
        drivetrain.startDrive(if (alliance == Alliance.BLUE) 13.0 else 131.0,72.0)
        while (drivetrain.updateDrive() && opModeIsActive()) { shooter.moveTurret() }
        drivetrain.stop()
    }
    private fun release(order: Array<Int>) {
        for (i in order) {
            spindexer.release(i)
            timer.reset()
            while (timer.milliseconds() < 400 && opModeIsActive() && !shooter.spin()) {
                shooter.moveTurret()
            }
            spindexer.kickarm.position = KICKARM_RELEASE
            while (timer.milliseconds() < 3400 && opModeIsActive()) {
                shooter.spin()
                shooter.moveTurret()
            }
            spindexer.kickarm.position = KICKARM_DOWN
            while (timer.milliseconds() < 4400 && opModeIsActive()) {
                shooter.spin()
                shooter.moveTurret()
            }
            spindexer.removeItem()
        }
    }
}