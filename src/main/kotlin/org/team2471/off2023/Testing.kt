package org.team2471.off2023

import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.AnalogPotentiometer
import edu.wpi.first.wpilibj.DigitalInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem

object Testing: Subsystem("Testing") {
//    val sensor = AnalogPotentiometer(3, 360.0, 0.0)
    ;
    init {
        GlobalScope.launch {
            periodic {
//                println("HI!!!")
//                println(sensor.get())
            }
        }
    }
}