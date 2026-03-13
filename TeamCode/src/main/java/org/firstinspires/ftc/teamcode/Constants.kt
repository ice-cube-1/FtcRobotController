package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import kotlin.math.PI

@Config
object Constants {
    @JvmField var KP_HEADING = -0.04
    @JvmField var KD_HEADING = -0.001
    @JvmField var KP_TRANSLATION = 0.001
    @JvmField var KD_TRANSLATION = 0.0001
    @JvmField var X_TICKS_PER_INCH = ((3249 * 28).toDouble() / 121) / (7.5/2.54 * PI)
    @JvmField var Y_TICKS_PER_INCH = ((3249 * 28).toDouble() / 121) / (7.5/2.54 * PI)
    @JvmField var MANUAL_MULTIPLIER = 0.8F
    @JvmField var ENCODER_ERROR = 35
    @JvmField var TURRET_ENCODER_KP = 10
    @JvmField var TURRET_SPEED = 0.2
    @JvmField var MIN_TURRET = 100
    @JvmField var MAX_TURRET = 100
    @JvmField var KICKARM_DOWN = 0.3
    @JvmField var KICKARM_RELEASE = 0.6
    @JvmField var INTAKE_POWER = -1
    @JvmField var endVelocity = 0
    @JvmField var KP_SHOOTER = 0.0005
    @JvmField var VELOCITY_DELTA = 2

    enum class SpinPosition {
        ZERO_IN  {
            override fun value() = 0
            override fun pos() = 1.0},
        ZERO_OUT {
            override fun value() = 0
            override fun pos() = 1.0},
        ONE_IN   {
            override fun value() = 1
            override fun pos() = 1.0},
        ONE_OUT  {
            override fun value() = 1
            override fun pos() = 1.0},
        TWO_IN   {
            override fun value() = 2
            override fun pos() = 1.0},
        TWO_OUT  {
            override fun value() = 2
            override fun pos() = 1.0};
        abstract fun value(): Int
        abstract fun pos(): Double
    }
}