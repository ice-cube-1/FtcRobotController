package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Constants.INTAKE_POWER

class Intake (hardwareMap: HardwareMap) {
    private val intake = hardwareMap.get(DcMotor::class.java, "intake").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    var on = false
    var reversed = false
    fun run() { intake.power = if (on) (if (reversed) -INTAKE_POWER else INTAKE_POWER) else 0.0 }
    fun getData(): String { return "Intake on: $on" }
}