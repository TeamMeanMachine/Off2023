package org.team2471.off2023

import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.FalconID
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.coroutines.MeanlibDispatcher
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.units.Angle
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.radians
import kotlin.math.atan2

object Turret : Subsystem("Turret") {

    const val turretDeadband = 10.0

    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    val turretCurrentEntry = table.getEntry("Turret Current")
    val turretAngleEntry = table.getEntry("Turret Angle")
    val turretSetpointEntry = table.getEntry("Turret Setpoint")


    val turningMotor = MotorController(FalconID(Falcons.TURRET))

    val turretGearRatio: Double = 50.0/11.0

    val turretAngle: Angle
        get() = turningMotor.position.degrees

//    var rawTurretSetpoint : Angle = 0.0.degrees
    var turretSetpoint: Angle = 0.0.degrees
        set(value) {
//            rawTurretSetpoint = value
            turningMotor.setPositionSetpoint(
                value.wrap().asDegrees.coerceIn(
                    (-180 + turretDeadband),
                    (180 - turretDeadband)
                )
            )
            turretSetpointEntry.setDouble(value.asDegrees)
            field = value
        }

    init {
        turningMotor.restoreFactoryDefaults()
        turningMotor.config(20) {
            //                          ticks / gear ratio
            feedbackCoefficient = 360.0 / 2048.0 / turretGearRatio

            brakeMode()
            inverted(false)
            pid {
                p(0.00002)
                d(0.00005)
            }
            currentLimit(0, 20, 0)
//            burnSettings()
        }
        turningMotor.setRawOffset(0.0)

        GlobalScope.launch(MeanlibDispatcher) {
            periodic {
                turretAngleEntry.setDouble(turretAngle.asDegrees)
                turretCurrentEntry.setDouble(turningMotor.current)

                // sets joystickTarget to the current angle of the right joystick, null if at center
                val joystickAngle : Angle? = if (OI.operatorController.rightThumbstick.length > Limelight.minJoystickDistance) {
                    -OI.operatorController.rightThumbstick.angle + 180.0.degrees
                } else {
                    null
                }


                val currentBuckets : List<BucketTarget> = Limelight.enemyBuckets

                // handle joystick input
                if (joystickAngle != null) {

                    println(joystickAngle)

                    val upperAimingBound : Angle = joystickAngle + 20.0.degrees
                    val lowerAimingBound : Angle = joystickAngle - 20.0.degrees

                    val target : BucketTarget? = Limelight.getBucketInBounds(upperAimingBound, lowerAimingBound)

                    if (target != null) {
                        aimAtBucket(target)
                    } else {
                        turretSetpoint = joystickAngle
                    }

                } else {
                    if (Limelight.enemyBuckets.isNotEmpty()) {
                        aimAtBucket(Limelight.enemyBuckets[0])
                    }
                }

                if (OI.operatorController.a) {
                    println(turretSetpoint)
                }

            }
        }
    }

    fun aimAtBucket(target : BucketTarget){
        turretSetpoint = Limelight.getAngleToBucket(target)
    }

}