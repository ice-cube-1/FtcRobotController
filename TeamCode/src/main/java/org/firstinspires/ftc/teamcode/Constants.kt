package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import kotlin.math.PI

@Config
class Constants {
    companion object {
        @JvmField var KP_HEADING: Double = 1.0
        @JvmField var KD_HEADING: Double = 0.001
        @JvmField var KP_TRANSLATION: Double = 0.01
        @JvmField var KD_TRANSLATION: Double = 0.001
        @JvmField var KP_SHOOTER: Double = 0.04
        @JvmField var KI_SPINNER: Double = 0.04
        @JvmField var X_TICKS_PER_INCH: Double = ((3249 * 28).toDouble() / 121) / (7.5/2.54 * PI)
        @JvmField var Y_TICKS_PER_INCH: Double = ((3249 * 28).toDouble() / 121) / (7.5/2.54 * PI)
    }
}
