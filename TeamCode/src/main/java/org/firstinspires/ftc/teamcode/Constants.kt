package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import kotlin.math.PI

@Config
class Constants {
    companion object {
        @JvmField var KP_HEADING = -0.04
        @JvmField var KD_HEADING = -0.001
        @JvmField var KP_TRANSLATION = 0.001
        @JvmField var KD_TRANSLATION = 0.0001
        @JvmField var KI_SPINNER = 0.04
        @JvmField var X_TICKS_PER_INCH = ((3249 * 28).toDouble() / 121) / (7.5/2.54 * PI)
        @JvmField var Y_TICKS_PER_INCH = ((3249 * 28).toDouble() / 121) / (7.5/2.54 * PI)
        @JvmField var MANUAL_MULTIPLIER = 0.8
        @JvmField var ENCODER_ERROR = 35
        @JvmField var TURRET_ENCODER_KP = 10
        @JvmField var TURRET_STEPS = 50
        @JvmField var MIN_TURRET = -100
        @JvmField var MAX_TURRET = 100
    }
}
