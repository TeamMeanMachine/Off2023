package org.team2471.off2023

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.AnalogInput
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians

object Turret : Subsystem("Turret") {

    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    val turretCurrentEntry = table.getEntry("Turret Current")
    val turretAngleEntry = table.getEntry("Turret Angle")


    val turningMotor = MotorController(FalconID(Falcons.TURRET))

    val turretGearRatio: Double = 50.0/11.0

    val turretAngle: Angle
        get() = turningMotor.position.degrees

    var turretSetpoint: Angle = 0.0.degrees
        set(value) {
            val angle = value.asDegrees.coerceIn(-175.0, 175.0).degrees
            turningMotor.setPositionSetpoint(angle.asDegrees)
            field = angle
        }

    var spinning = false

    init {
        turningMotor.restoreFactoryDefaults()
        turningMotor.config(20) {
            //                          ticks / gear ratio
            feedbackCoefficient = 360.0 / 2048.0 / turretGearRatio

            brakeMode()
            inverted(false)
            pid {
                p(0.0001)
                d(0.0001)
            }
            currentLimit(0, 20, 0)
//            burnSettings()
        }
        turningMotor.setRawOffset(0.0)

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                turretAngleEntry.setDouble(turretAngle.asDegrees)
                turretCurrentEntry.setDouble(turningMotor.current)
            }
        }
    }

}