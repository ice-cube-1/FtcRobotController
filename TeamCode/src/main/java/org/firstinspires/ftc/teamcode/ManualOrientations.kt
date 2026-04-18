package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.PI

@TeleOp(name = "Manual, Red, intake AWAY")
class Red0: Manual(0.0, 24)
@TeleOp(name = "Manual, Red, intake LEFT")
class Red1: Manual(-PI/2, 24)
@TeleOp(name = "Manual, Red, intake TOWARDS")
class Red2: Manual(PI, 24)
@TeleOp(name = "Manual, Red, intake RIGHT")
class Red3: Manual(PI/2, 24)
@TeleOp(name = "Manual, Blue, intake AWAY")
class Blue0: Manual(0.0, 20)
@TeleOp(name = "Manual, Blue, intake LEFT")
class Blue1: Manual(-PI/2, 20)
@TeleOp(name = "Manual, Blue, intake TOWARDS")
class Blue2: Manual(PI, 20)
@TeleOp(name = "Manual, Blue, intake RIGHT")
class Blue3: Manual(PI/2, 20)