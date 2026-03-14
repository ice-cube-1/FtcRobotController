package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config

@Config
object Constants {
    @JvmField var KP_HEADING = 0.07
    @JvmField var KD_HEADING = 0.002
    @JvmField var KP_TRANSLATION = 0.0006
    @JvmField var KD_TRANSLATION = 0.00003
    @JvmField var X_TICKS_PER_INCH = 6453.0/ (223.0/2.54)
    @JvmField var Y_TICKS_PER_INCH = 5793.25 / (223.0/2.54)
    @JvmField var MANUAL_MULTIPLIER = 1.0F
    @JvmField var ENCODER_ERROR = 35
    @JvmField var TURRET_ENCODER_KP = 10
    @JvmField var TURRET_SPEED = 0.2
    @JvmField var MIN_TURRET = 100
    @JvmField var MAX_TURRET = 100
    @JvmField var KICKARM_DOWN = 0.43
    @JvmField var KICKARM_RELEASE = 0.71
    @JvmField var INTAKE_POWER = 1.0
    @JvmField var endVelocity = 0
    @JvmField var KP_SHOOTER = 0.0005
    @JvmField var VELOCITY_DELTA = 2

    enum class SpinPosition {
        ONE_IN  {
            override fun value() = 1
            override fun pos() = 0.6},
        ONE_OUT {
            override fun value() = 1
            override fun pos() = 0.0},
        ZERO_IN   {
            override fun value() = 0
            override fun pos() = 1.0},
        ZERO_OUT  {
            override fun value() = 0
            override fun pos() = 0.4},
        TWO_IN   {
            override fun value() = 2
            override fun pos() = 0.2},
        TWO_OUT  {
            override fun value() = 2
            override fun pos() = 0.8};
        abstract fun value(): Int
        abstract fun pos(): Double
    }
}