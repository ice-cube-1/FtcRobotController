package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config

@Config
object Constants {
    @JvmField var KP_HEADING = 1.0
    @JvmField var KD_HEADING = 0.3
    @JvmField var KP_TRANSLATION = 0.04
    @JvmField var KD_TRANSLATION = 0.02
    @JvmField var MANUAL_MULTIPLIER = 0.2
    @JvmField var ENCODER_ERROR = 35
    @JvmField var TURRET_KP = 0.03
    @JvmField var TURRET_STEP = 0.4
    @JvmField var STOP_DOWN = 0.32
    @JvmField var STOP_UP = 0.4
    @JvmField var INTAKE_POWER = 1.0
    @JvmField var KP_SHOOTER = 0.0003
    @JvmField var VELOCITY_DELTA = 8
    @JvmField var HOOD_ANGLE = 0.5
    @JvmField var TURRET_MAX_DEGREES = ((35.0/15.0)*(20.0/82.0))*360.0
    @JvmField var TURRET_ZERO_DEG = ((35.0/15.0)*(20.0/82.0))*180.0
    @JvmField var ODOMETRY_TICKS_PER_CM = 126.3
}