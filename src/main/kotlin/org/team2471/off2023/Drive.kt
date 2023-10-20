package org.team2471.off2023

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.MotorController
import org.team2471.frc.lib.actuators.SparkMaxID
import org.team2471.frc.lib.control.PDConstantFController
import org.team2471.frc.lib.control.PDController
import org.team2471.frc.lib.coroutines.*
import org.team2471.frc.lib.framework.Subsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.input.Controller //Added by Jeremy on 1-30-23 for power testing
import org.team2471.frc.lib.math.Vector2
import org.team2471.frc.lib.math.linearMap
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.motion.following.*
import org.team2471.frc.lib.motion_profiling.MotionCurve
import org.team2471.frc.lib.motion_profiling.following.SwerveParameters
import org.team2471.frc.lib.units.*
import kotlin.math.absoluteValue
import kotlin.math.min
import kotlin.math.roundToInt

@OptIn(DelicateCoroutinesApi::class)
object Drive : Subsystem("Drive"), SwerveDrive {
    val robotHalfWidth = (32.0/2.0).inches //<- THIS VALUE IS WRONG
    val table = NetworkTableInstance.getDefault().getTable(name)
    val navXGyroEntry = table.getEntry("NavX Gyro")

    val odometer0Entry = table.getEntry("Odometer 0")
    val odometer1Entry = table.getEntry("Odometer 1")
    val odometer2Entry = table.getEntry("Odometer 2")
    val odometer3Entry = table.getEntry("Odometer 3")

    val useGyroEntry = table.getEntry("Use Gyro")
    val angleToNodeEntry = table.getEntry("Angle To Node")
    val demoBoundingBoxEntry = table.getEntry("Demo Bounding Box")
    val demoAprilLookingAtEntry = table.getEntry("Demo Look At Tags")

    val rateCurve = MotionCurve()

    /**
     * Coordinates of modules
     * **/
    override val modules: Array<SwerveDrive.Module> = arrayOf(
        Module(
            MotorController(SparkMaxID(Sparks.FRONT_LEFT_DRIVE)),
            MotorController(SparkMaxID(Sparks.FRONT_LEFT_STEER)),
            Vector2(-13.1, 13.1),
            Preferences.getDouble("Angle Offset 0", -263.79).degrees,
            DigitalSensors.FRONT_LEFT,
            odometer0Entry,
            0
        ),
        Module(
            MotorController(SparkMaxID(Sparks.FRONT_RIGHT_DRIVE)),
            MotorController(SparkMaxID(Sparks.FRONT_RIGHT_STEER)),
            Vector2(13.1, 13.1),
            Preferences.getDouble("Angle Offset 1",-352.78).degrees,
            DigitalSensors.FRONT_RIGHT,
            odometer1Entry,
            1
        ),
        Module(
            MotorController(SparkMaxID(Sparks.REAR_RIGHT_DRIVE)),
            MotorController(SparkMaxID(Sparks.REAR_RIGHT_STEER)),
            Vector2(13.1, -13.1),
            Preferences.getDouble("Angle Offset 2",-271.48).degrees,
            DigitalSensors.REAR_RIGHT,
            odometer2Entry,
            2
        ),
        Module(
            MotorController(SparkMaxID(Sparks.REAR_LEFT_DRIVE)),
            MotorController(SparkMaxID(Sparks.REAR_LEFT_STEER)),
            Vector2(-13.1, -13.1),
            Preferences.getDouble("Angle Offset 3",-229.93).degrees,
            DigitalSensors.REAR_LEFT,
            odometer3Entry,
            3
        )
    )

    private var navX: NavxWrapper = NavxWrapper()
    val gyro = navX
    private var gyroOffset = 0.0.degrees

    override var heading: Angle
        get() = (gyroOffset + gyro.angle.degrees).wrap()
        set(value) {
            gyro.reset()
            gyroOffset = -gyro.angle.degrees + value
        }

    override val headingRate: AngularVelocity
        get() = -gyro.rate.degrees.perSecond

    override var velocity = Vector2(0.0, 0.0)
    override var position = Vector2(0.0, 0.0)
    override val combinedPosition: Vector2
        get() = Vector2(0.0, 0.0)
    override var robotPivot = Vector2(0.0, 0.0)
    override var headingSetpoint = 0.0.degrees

    var angleToNode: Angle = 0.0.degrees
    val demoBondingBox: Boolean
        get() = demoBoundingBoxEntry.getBoolean(false)

    override val parameters: SwerveParameters = SwerveParameters(
        gyroRateCorrection = 0.0,
        kpPosition = 0.32,
        kdPosition = 0.6,
        kPositionFeedForward = 0.05,
        kpHeading = 0.008,
        kdHeading = 0.01,
        kHeadingFeedForward = 0.001,
        kMoveWhileSpin = 24.0,
    )

    override val carpetFlow = Vector2(0.0, 1.0)
    override val kCarpet = 0.0234 // how much downstream and upstream carpet directions affect the distance, for no effect, use  0.0 (2.5% more distance downstream)
    override val kTread = 0.0 //.04 // how much of an effect treadWear has on distance (fully worn tread goes 4% less than full tread)  0.0 for no effect
    val plannedPathEntry = table.getEntry("Planned Path")
    val actualRouteEntry = table.getEntry("Actual Route")
    override val plannedPath: NetworkTableEntry = plannedPathEntry
    override val actualRoute: NetworkTableEntry = actualRouteEntry

    val autoPDController = PDConstantFController(0.015, 0.04, 0.05)
    val teleopPDController =  PDConstantFController(0.012, 0.09, 0.05)
    var aimPDController = teleopPDController

    var maxTranslation = 1.0
        get() =  if (demoMode) min(field, demoSpeed) else field

    val isHumanDriving
        get() = OI.driveTranslation.length != 0.0 || OI.driveRotation != 0.0

    init {
        println("drive init")
        initializeSteeringMotors()

        GlobalScope.launch(MeanlibDispatcher) {
            println("in drive global scope")
            val headingEntry = table.getEntry("Heading")
            val xEntry = table.getEntry("X")
            val yEntry = table.getEntry("Y")
            val poseEntry = table.getEntry("advantageScopePose")

            SmartDashboard.setPersistent("Use Gyro")
            SmartDashboard.setPersistent("Gyro Type")

            if (!SmartDashboard.containsKey("DemoSpeed")) {
                println("DemoSpeed does not exist, setting it to 1.0")
                SmartDashboard.getEntry("DemoSpeed").setDouble(1.0)
                SmartDashboard.setPersistent("DemoSpeed")
            }

            navXGyroEntry.setBoolean(true)
            rateCurve.setMarkBeginOrEndKeysToZeroSlope(false)
            rateCurve.storeValue(1.0, 2.0)  // distance, rate
            rateCurve.storeValue(8.0, 6.0)  // distance, rate

            println("in init just before periodic")
            val module0 = (modules[0] as Module)

            periodic {
            //  println("${module0.angle}  ${ round(absoluteAngle.asDegrees, 2) }"
                // round(absoluteAngle.asDegrees, 2) }
               //rintln("${turnMotor.motorID}   ${ round(absoluteAngle.asDegrees, 2) }"

                val (x, y) = position
                xEntry.setDouble(x)
                yEntry.setDouble(y)
                headingEntry.setDouble(heading.asDegrees)
                val poseWPI = FieldManager.convertTMMtoWPI(x.feet, y.feet, heading)
//                println("X: $x, Y: $y")
//                println(poseWPI)
                poseEntry.setDoubleArray(doubleArrayOf(poseWPI.x, poseWPI.y, poseWPI.rotation.degrees))
            }
        }
    }

    override fun preEnable() {
        //initializeSteeringMotors()
        odometer0Entry.setDouble(Preferences.getDouble("odometer 0",0.0))
        odometer1Entry.setDouble(Preferences.getDouble("odometer 1",0.0))
        odometer2Entry.setDouble(Preferences.getDouble("odometer 2",0.0))
        odometer3Entry.setDouble(Preferences.getDouble("odometer 3",0.0))
        println("prefs at enable=${Preferences.getDouble("odometer 0",0.0)}")
    }

    override fun onDisable() {
        if (odometer0Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 0", odometer0Entry.getDouble(0.0))
        if (odometer1Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 1", odometer1Entry.getDouble(0.0))
        if (odometer2Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 2", odometer2Entry.getDouble(0.0))
        if (odometer3Entry.getDouble(0.0) > 0.0) Preferences.setDouble("odometer 3", odometer3Entry.getDouble(0.0))
        actualRoute.setDoubleArray(doubleArrayOf())
        plannedPath.setString("")
    }

    override fun poseUpdate(poseTwist: SwerveDrive.Pose) {
        //MAPoseEstimator.addDriveData(Timer.getFPGATimestamp(), Twist2d(poseTwist.position.y, poseTwist.position.x, -poseTwist.heading.asRadians))
    }

    fun zeroGyro() {
        heading = if (FieldManager.isBlueAlliance) 180.0.degrees else 0.0.degrees
        println("zeroed heading to $heading  alliance blue? ${FieldManager.isBlueAlliance}")
    }

    override suspend fun default() {
        periodic {
            var turn = 0.0
            if (OI.driveRotation.absoluteValue > 0.001) {
                turn = OI.driveRotation
            }
            if (!useGyroEntry.exists()) {
                useGyroEntry.setBoolean(true)
            }
            val useGyro2 = useGyroEntry.getBoolean(true) && !DriverStation.isAutonomous()
            if (FieldManager.homeField) {
                angleToNodeEntry.setDouble(angleToNode.asDegrees)
            }
            drive(
                OI.driveTranslation * maxTranslation,
                turn,
                useGyro2,
                useGyro2
                )
            }
        }
    fun initializeSteeringMotors() {
        for (moduleCount in 0..3) { //changed to modules.indices, untested
            val module = (modules[moduleCount] as Module)
            module.turnMotor.setRawOffset(module.absoluteAngle.asDegrees)
            println("Module: $moduleCount analogAngle: ${module.absoluteAngle}")
        }
    }

    fun resetDriveMotors() {
        for (moduleCount in 0..3) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.restoreFactoryDefaults()
            println("For module $moduleCount, drive motor's factory defaults were restored.")
        }
    }

    fun resetSteeringMotors() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.turnMotor.restoreFactoryDefaults()
            println("For module $moduleCount, turn motor's factory defaults were restored.")
        }
    }

    fun brakeMode() {
        for (moduleCount in modules.indices) {
            val module = (modules[moduleCount] as Module)
            module.driveMotor.brakeMode()
        }
    }

    fun coastMode() {
        for (element in modules) { //switched from element in 0..modules.size-1, untested
            val module = (element as Module)
            module.driveMotor.coastMode()
            module.turnMotor.coastMode()
        }
    }

    class Module(
        val driveMotor: MotorController,
        val turnMotor: MotorController,
        override val modulePosition: Vector2,
        override var angleOffset: Angle,
        digitalInputID: Int,
        private val odometerEntry: NetworkTableEntry,
        val index: Int
    ) : SwerveDrive.Module {
        companion object {
            private const val ANGLE_MAX = 983
            private const val ANGLE_MIN = 47

            private val P = 0.0075 //0.010
            private val D = 0.00075
        }

        override val angle: Angle
            get() = -turnMotor.position.degrees

        val digitalEncoder : DutyCycleEncoder = DutyCycleEncoder(digitalInputID)

        val absoluteAngle: Angle
            get() {
                return (-digitalEncoder.absolutePosition.degrees * 360.0 - angleOffset).wrap()
            }

        override val treadWear: Double
            get() = linearMap(0.0, 10000.0, 1.0, 0.96, odometer).coerceIn(0.96, 1.0)

        val driveCurrent: Double
            get() = driveMotor.current

        private val pdController = PDController(P, D)

        override val speed: Double
            get() = driveMotor.velocity

        val power: Double
            get() {
                return -driveMotor.output
            }

        override val currDistance: Double
            get() = driveMotor.position

        override var prevDistance: Double = 0.0

        override var odometer: Double
            get() = odometerEntry.getDouble(0.0)
            set(value) { odometerEntry.setDouble(value) }

        override fun zeroEncoder() {
            driveMotor.position = 0.0
        }

        override var angleSetpoint: Angle = 0.0.degrees
            set(value) = turnMotor.setPositionSetpoint(-(angle + (value - angle).wrap()).asDegrees)

        override fun setDrivePower(power: Double) {
            driveMotor.setPercentOutput(-power)
        }

        val error: Angle
            get() = turnMotor.closedLoopError.degrees

        init {
            println("Drive.module.init")
            turnMotor.config(20) {
                feedbackCoefficient = (360.0 / 42.0 / 12.0 / 5.08) * (360.5 / 274.04)
                inverted(false)
                setSensorPhase(false)
                coastMode()
                setRawOffsetConfig(absoluteAngle.asDegrees)
                pid {
                    p(0.0002)//0.000002
//                    d(0.0000025)
                }
            }
            driveMotor.config {
                brakeMode()
                //                    wheel diam / 12 in per foot * pi / ticks / gear ratio
                feedbackCoefficient = 3.0 / 12.0 * Math.PI / 42.0 / 4.71
//                inverted(true)
                currentLimit(70, 75, 1)
                openLoopRamp(0.2)
            }
            GlobalScope.launch {
                periodic {

//                    println("${turnMotor.motorID}   ${ round(absoluteAngle.asDegrees, 2) }")
                }
            }
        }

        override fun driveWithDistance(angle: Angle, distance: Length) {
            driveMotor.setPositionSetpoint(distance.asFeet)
            val error = (angle - this.angle).wrap()
            pdController.update(error.asDegrees)
        }

        override fun stop() {
            driveMotor.stop()
        }

        fun setAngleOffset() {
            val digitalAngle = -digitalEncoder.absolutePosition
            angleOffset = digitalAngle.degrees * 360.0
            Preferences.setDouble("Angle Offset $index", angleOffset.asDegrees)
            println("Angle Offset $index = $digitalAngle")
        }
    }
    fun setAngleOffsets() {
        for (element in modules) {
            val module = (element as Module)
            module.setAngleOffset()
        }
        initializeSteeringMotors()
    }


    suspend fun calibrateRobotPosition() = use(Drive) {
        position = Vector2(-11.5, if (FieldManager.isBlueAlliance) 21.25 else -21.25)
        zeroGyro()
    }
}

fun Drive.abortPath(): Boolean {
    return isHumanDriving
}

suspend fun Drive.currentTest() = use(this) {
    var power = 0.0
    var upPressed = false
    var downPressed = false
    periodic {
        if (OI.driverController.dPad == Controller.Direction.UP) {
            upPressed = true
        } else if (OI.driverController.dPad == Controller.Direction.DOWN) {
            downPressed = true
        }
        if (OI.driverController.dPad != Controller.Direction.UP && upPressed) {
            upPressed = false
            power += 0.001
        }
        if (OI.driverController.dPad != Controller.Direction.DOWN && downPressed) {
            downPressed = false
            power -= 0.001
        }
//        for (moduleCount in 0..3) {
//            val module = modules[moduleCount] as Drive.Module
//        }
//        println()
//        println("power: $power")
        var currModule = modules[0] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[1] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[2] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        currModule = modules[3] as Drive.Module
        currModule.driveMotor.setPercentOutput(power)
        currModule.turnMotor.setPositionSetpoint(0.0)
        
        println("current: ${round(currModule.driveCurrent, 2)}  power: $power")
    //    val currModule2 = modules[3] as Drive.Module
      //  currModule2.driveMotor.setPercentOutput(power)
        //currModule2.turnMotor.setPositionSetpoint(0.0)
       // println("current: ${round(currModule.driveCurrent, 2)}  power: $power")

    //        drive(
//            Vector2(0.0, power),
//            0.0,
//            false
//        )
    }
}