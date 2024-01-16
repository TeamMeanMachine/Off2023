package org.team2471.off2023

import edu.wpi.first.wpilibj.DriverStation
import org.jetbrains.kotlin.gradle.utils.`is`
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.input.*
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.cube
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.squareWithSign
import org.team2471.frc.lib.motion.following.demoMode
import org.team2471.frc.lib.motion.following.xPose
import org.team2471.frc.lib.units.degrees

object OI : Subsystem("OI") {
    val driverController = XboxController(0)
    val operatorController = XboxController(1)

    private val deadBandDriver = 0.1
    private val deadBandOperator = 0.1

    private val driveTranslationX: Double
        get() = (if (FieldManager.isRedAlliance) 1.0 else -1.0) * driverController.leftThumbstickX.deadband(deadBandDriver).squareWithSign()

    private val driveTranslationY: Double
        get() = (if (FieldManager.isRedAlliance) -1.0 else 1.0) * driverController.leftThumbstickY.deadband(deadBandDriver).squareWithSign()

    private val forwardTest: Boolean
        get() = driverController.a

    val driveTranslation: Vector2
        get() = Vector2(if (forwardTest) 0.0 else driveTranslationX, -driveTranslationY) //does owen want this cubed?

    val driveRotation: Double
        get() = (driverController.rightThumbstickX.deadband(deadBandDriver)).cube() // * 0.6

    val driveLeftTrigger: Double
        get() = driverController.leftTrigger

    val driveRightTrigger: Double
        get() = driverController.rightTrigger

    val operatorLeftTrigger: Double
        get() = operatorController.leftTrigger

    val operatorLeftY: Double
        get() = operatorController.leftThumbstickY.deadband(0.2)

    val operatorLeftX: Double
        get() = operatorController.leftThumbstickX.deadband(0.2)

    val operatorRightTrigger: Double
        get() = operatorController.rightTrigger

    val operatorRightX: Double
        get() = operatorController.rightThumbstickX.deadband(0.25)

    val operatorRightY: Double
        get() = operatorController.rightThumbstickY.deadband(0.25)

    init {
        driverController::back.whenTrue {
            Drive.zeroGyro();
            Drive.initializeSteeringMotors()
        }
        driverController::start.whenTrue { Drive.calibrateRobotPosition() }
        driverController::x.whenTrue { Drive.xPose() }
        operatorController::a.whenTrue {
            Intake.intaking = !Intake.intaking
            println("intaking ${Intake.intaking}")
        }
        operatorController::b.whenTrue {
            Intake.conveying = !Intake.conveying
            println("conveying ${Intake.conveying}")
        }
        operatorController::y.whenTrue {
            Intake.shooting = !Intake.shooting
            println("shooting ${Intake.shooting}")
        }
    }
}
