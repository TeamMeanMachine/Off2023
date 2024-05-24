package org.team2471.subsystems

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.ControlModeValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.MotorSafety


class Module(
    private val driveMotor: TalonFX,
    private val steerMotor: CANSparkMax,
    private val steerEncoder: DutyCycleEncoder,
    private val isInverted: Boolean,
    var state: SwerveModuleState,
    brakeMode: Boolean
): MotorSafety() {

//    var position: SwerveModulePosition

    private var goal = state

    init {
        driveMotor.setNeutralMode(NeutralModeValue.Coast)

//        driveMotor.setControl(PIDController())

        steerMotor.restoreFactoryDefaults()
    }

    override fun stopMotor() {
        TODO("Not yet implemented")
    }

    override fun getDescription(): String {
        TODO("Not yet implemented")
    }

}