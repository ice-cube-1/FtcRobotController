package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.KICKARM_DOWN
import org.firstinspires.ftc.teamcode.Constants.KICKARM_RELEASE


@Autonomous(name="Auto 1", group = "Auto")
class Auto1Artefact : LinearOpMode() {
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
        shooter.setStart(if (alliance == Alliance.BLUE) 45.0 else -45.0, if (alliance ==  Alliance.BLUE) 20 else 24)
        drivetrain = DriveTrain(hardwareMap)
        drivetrain.setStart(if (alliance == Alliance.BLUE) 54.0 else 90.0, 135.0,0.0)
        spindexer = Spindexer(hardwareMap)
        spindexer.release(0)
        waitForStart()
        drivetrain.startDrive(if (alliance == Alliance.BLUE) 54.0 else 90.0,102.0)
        while (!drivetrain.updateDrive()) {}
        drivetrain.stop()
        shooter.setTurretManual(if (alliance == Alliance.BLUE) 45.0 else -45.0)
        timer.reset()
        while (shooter.turretState != TurretState.DETECTED && timer.seconds() < 3) {
            shooter.moveTurret(drivetrain.getOrientationDeg())
        }
        shooter.turnOnShooter()
        while (timer.seconds() < 7) { shooter.spin() }
        spindexer.moveKickarm(KICKARM_RELEASE)
        while (timer.seconds() < 9) { shooter.spin() }
        spindexer.moveKickarm(KICKARM_DOWN)
        while (timer.seconds() < 10) {
            shooter.spin()
            shooter.moveTurret(drivetrain.getOrientationDeg())
        }
        shooter.turnOffShooter()
        drivetrain.startDrive(if (alliance == Alliance.BLUE) 54.0 else 90.0,72.0)
        while (!drivetrain.updateDrive() && opModeIsActive()) { shooter.moveTurret(drivetrain.getOrientationDeg()) }
        drivetrain.stop()
        shooter.setTurretManual(0.0)
    }
}