package org.team2471.off2023

import com.revrobotics.ColorSensorV3
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.I2C
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem

object Intake : Subsystem("shooterThing") {
    val table = NetworkTableInstance.getDefault().getTable("Intake")
    val shooterPercentEntry = table.getEntry("shooterPercent")
    val conveyorPercentEntry = table.getEntry("conveyorPercent")
    val intakePercentEntry = table.getEntry("intakePercent")

    val intakeCurretEntry = table.getEntry("Intake Current")
    val conveyorCurrentEntry = table.getEntry("Conveyor Current")
    val shooterCurrentEntry = table.getEntry("Shooter Current")


    var shooting = false
    var conveying = false
    var intaking = false

    val intakeMotors = MotorController(FalconID(Falcons.INTAKE_TOP), FalconID(Falcons.INTAKE_BOTTOM))
    val conveyorMotor = MotorController(FalconID(Falcons.CONVEYOR_TOP), FalconID(Falcons.CONVEYOR_BOTTOM))
    val shooterMotor = MotorController(FalconID(Falcons.SHOOTER_TOP), FalconID(Falcons.SHOOTER_BOTTOM))

//    private val i2cPort: I2C.Port = I2C.Port.kMXP
//    private val colorSensor = ColorSensorV3(i2cPort)

    init {
        intakePercentEntry.setDouble(0.1)
        conveyorPercentEntry.setDouble(0.1)
        shooterPercentEntry.setDouble(0.1)
        intakeMotors.config {
            currentLimit(20, 30, 1)
            coastMode()
            inverted(false)
            followersInverted(true)
        }

        conveyorMotor.config {
            currentLimit(20, 30, 1)
            coastMode()
            inverted(false)
            followersInverted(false)
        }

        shooterMotor.config {
            currentLimit(20, 30, 1)
            coastMode()
            inverted(false)
            followersInverted(false)
        }

        GlobalScope.launch{
            periodic {
//                println("distance: ${colorSensor.proximity}    color: ${colorSensor.color}")

                if (intaking) {
                    intakeMotors.setPercentOutput(intakePercentEntry.getDouble(0.1))
                } else {
                    intakeMotors.setPercentOutput(0.0)
                }
                if (shooting) {
                    shooterMotor.setPercentOutput(shooterPercentEntry.getDouble(0.1))
                } else {
                    shooterMotor.setPercentOutput(0.0)
                }
                if (conveying) {
                    conveyorMotor.setPercentOutput(conveyorPercentEntry.getDouble(0.1))
                } else {
                    conveyorMotor.setPercentOutput(0.0)
                }
                shooterCurrentEntry.setDouble(shooterMotor.current)
                intakeCurretEntry.setDouble(intakeMotors.current)
                conveyorCurrentEntry.setDouble(conveyorMotor.current)
            }
        }
    }


}