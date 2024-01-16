package org.team2471.off2023.testing

import org.team2471.frc.lib.coroutines.periodic
import org.team2471.off2023.Intake
import org.team2471.off2023.OI

suspend fun joystickTest() {
    periodic {
        val power = OI.operatorController.leftThumbstickX
        println(power)
        Intake.intakeMotors.setPercentOutput(power)
    }
}