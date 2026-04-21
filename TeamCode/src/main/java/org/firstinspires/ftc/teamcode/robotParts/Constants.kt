package org.firstinspires.ftc.teamcode.robotParts

object Constants {
    @JvmField var KP_HEADING = 1.0
    @JvmField var KD_HEADING = 0.05
    @JvmField var KP_TRANSLATION = 0.012
    @JvmField var KD_TRANSLATION = 0.0005
    @JvmField var MANUAL_MULTIPLIER = 1.0
    @JvmField var TURRET_KP = 0.1
    @JvmField var TURRET_STEP = 1.8
    @JvmField var STOP_DOWN = 0.32
    @JvmField var STOP_UP = 1.0
    @JvmField var INTAKE_POWER = 0.6
    @JvmField var TRANSFER_POWER = 0.6
    @JvmField var KP_SHOOTER = 0.02
    @JvmField var HOOD_ANGLE = 1.0
    @JvmField var TURRET_MAX_DEGREES = ((35.0/15.0)*(20.0/82.0))*360.0
    @JvmField var TURRET_ZERO_DEG = ((35.0/15.0)*(20.0/82.0))*180.0
    @JvmField var ODOMETRY_TICKS_PER_CM = 132.0
    @JvmField var SHOOTER_IDLE_VELOCITY = 400
    @JvmField var K_S = 0.18
    @JvmField var KS_MANUAL = 0.09
    @JvmField var POWER_MAX = 0.75
    @JvmField var POWER_DELTA = 0.001
    @JvmField var ROBOT_WIDTH_CM = 39.0
    @JvmField var Robot_LENGTH_CM = 46.0
    @JvmField var xDisp = 10.0
    @JvmField var yDisp = -12.0
    @JvmField var K_FF = 0.0004076634
    @JvmField var KFF_INTERCEPT = 0.0652076247
    @JvmField var OFFSET = 0.3
    @JvmField var shooter_y = 90.0
    @JvmField var intakePositions = arrayOf(84.0,60.0,36.0)
    @JvmField var intake_far_x = 34.0
    @JvmField var numberToCollect = 3
}