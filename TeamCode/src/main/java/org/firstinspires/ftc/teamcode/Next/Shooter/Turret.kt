package org.firstinspires.ftc.teamcode.Next.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.FieldConstants.BLUE_GOAL_X
import org.firstinspires.ftc.teamcode.FieldConstants.GOAL_Y
import org.firstinspires.ftc.teamcode.FieldConstants.RED_GOAL_X
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.TurretConstants.motorGearTeeth
import org.firstinspires.ftc.teamcode.TurretConstants.outputGearTeeth

import kotlin.math.*

const val TURRET_MIN_ANGLE = -135.0
const val TURRET_MAX_ANGLE = 135.0
const val ENCODER_CPR = 4000

@Configurable
object Turret : Subsystem {

    enum class State { IDLE, MANUAL, LOCKED, RESET }

    // ==================== TUNABLE COEFFICIENTS ====================
    @JvmField var kP: Double = 0.03
    @JvmField var kI: Double = 0.0
    @JvmField var kD: Double = 0.005
    @JvmField var ffKV: Double = 0.07
    @JvmField var ffKA: Double = 0.0
    @JvmField var ffKS: Double = 0.01
    @JvmField var minPower: Double = 0.15
    @JvmField var maxPower: Double = 0.75
    @JvmField var alignmentTolerance: Double = 1.0
    @JvmField var rotationCompGain: Double = 2.0

    // ==================== SOTM ====================
    // Tune this — how far ahead in time to predict robot position (seconds)
    // Start at 0.0 and increase until turret leads robot motion correctly
    @JvmField var sotmLookahead: Double = 0.0

    var motor = MotorEx("turret")
    var currentState = State.IDLE
    var manualPower = 0.0
    var targetYaw = 0.0
    var isLocked = false

    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    var robotAngularVelocity = 0.0

    @JvmField var alliance = Drive.Alliance.BLUE

    private val goalX: Double get() = if (alliance == Drive.Alliance.RED) RED_GOAL_X else BLUE_GOAL_X
    private val goalY: Double get() = GOAL_Y

    private var controller = buildController()

    private fun buildController() = controlSystem {
        posPid(kP, kI, kD)
        basicFF(ffKV, ffKA, ffKS)
    }

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
        lastRobotHeading = Drive.currentHeading   // seed with current heading (radians)
        robotAngularVelocity = 0.0
        controller = buildController()
        targetYaw = getYaw()
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        controller = buildController()

        if (currentState == State.LOCKED) {
            updateRobotAngularVelocity()
        }

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-1.0, 1.0)
            State.LOCKED -> runLockedControl()
            State.RESET -> runResetControl()
        }

        ActiveOpMode.telemetry.addData("Turret/State", currentState.name)
        ActiveOpMode.telemetry.addData("Turret/Yaw", "%.2f°".format(Math.toDegrees(getYaw())))
        ActiveOpMode.telemetry.addData("Turret/Target", "%.2f°".format(Math.toDegrees(targetYaw)))
        ActiveOpMode.telemetry.addData("Turret/Locked", if (isLocked) "YES" else "NO")
        ActiveOpMode.telemetry.addData("Turret/RobotVel", "%.2f°/s".format(Math.toDegrees(robotAngularVelocity)))
        ActiveOpMode.telemetry.addData("Turret/SOTM", "lookahead=%.2fs".format(sotmLookahead))
    }

    // ==================== LOCKED CONTROL ====================
    fun runLockedControl() {
        if (!Drive.poseValid) return

        // SOTM: predict where robot will be using pinpoint velocity * lookahead
        // When sotmLookahead = 0.0 this is identical to static aiming
        val predictedX = Drive.currentX + Drive.velocityX * sotmLookahead
        val predictedY = Drive.currentY + Drive.velocityY * sotmLookahead

        val deltaX = goalX - predictedX
        val deltaY = goalY - predictedY
        val fieldAngleToGoal = atan2(deltaY, deltaX)

        // Drive.currentHeading is in radians (Pedro)
        val robotHeadingRad = Drive.currentHeading
        val rawTarget = normalizeAngle(fieldAngleToGoal - robotHeadingRad)

        val minRad = Math.toRadians(TURRET_MIN_ANGLE)
        val maxRad = Math.toRadians(TURRET_MAX_ANGLE)

        // FIX: if target is outside physical range, reset controller to prevent
        // windup snap when target re-enters range, but still drive to the limit
        val isOutsideRange = rawTarget < minRad || rawTarget > maxRad
        targetYaw = rawTarget.coerceIn(minRad, maxRad)

        if (isOutsideRange) {
            controller.reset()
        }

        val currentYaw = getYaw()
        val rotationCompensation = -robotAngularVelocity * rotationCompGain

        controller.goal = KineticState(targetYaw, rotationCompensation)
        var power = controller.calculate(KineticState(currentYaw, robotAngularVelocity))

        // Minimum power to overcome static friction — only apply when outside deadband
        val errorDeg = Math.toDegrees(abs(targetYaw - currentYaw))
        if (errorDeg > alignmentTolerance) {
            power += sign(targetYaw - currentYaw) * minPower
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    // ==================== RESET CONTROL ====================
    private fun runResetControl() {
        val currentYaw = getYaw()
        val error = normalizeAngle(0.0 - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        if (errorDeg < alignmentTolerance) {
            motor.power = 0.0
            targetYaw = 0.0
            currentState = State.IDLE
            return
        }

        controller.goal = KineticState(0.0, 0.0)
        var power = controller.calculate(KineticState(currentYaw, 0.0))

        if (errorDeg > alignmentTolerance) {
            power += sign(error) * minPower
        }

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    // ==================== VELOCITY TRACKING ====================
    private fun updateRobotAngularVelocity() {
        if (!Drive.poseValid) {
            robotAngularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        if (dt !in 0.01..0.2) {
            velTimer.reset()
            return
        }

        val currentHeading = Drive.currentHeading  // radians from Pedro
        if (currentHeading.isNaN() || currentHeading.isInfinite()) {
            velTimer.reset()
            return
        }

        // Both in radians — normalizeAngle handles wrap correctly
        val deltaHeading = normalizeAngle(currentHeading - lastRobotHeading)
        robotAngularVelocity = deltaHeading / dt   // rad/s
        lastRobotHeading = currentHeading
        velTimer.reset()
    }

    // ==================== PUBLIC METHODS ====================
    fun lock() {
        // Seed heading on entry to prevent velocity spike on first tick
        lastRobotHeading = Drive.currentHeading
        velTimer.reset()
        isLocked = true
        currentState = State.LOCKED
    }

    fun stop() {
        currentState = State.IDLE
        isLocked = false
        motor.power = 0.0
    }

    fun setManual(power: Double) {
        currentState = State.MANUAL
        isLocked = false
        manualPower = power
    }

    fun resetToCenter() {
        currentState = State.RESET
        isLocked = false
    }

    fun getYawDegrees(): Double = Math.toDegrees(getYaw())

    fun getYaw(): Double = normalizeAngle(
        motor.currentPosition.toDouble() * ticksToRadians()
    )

    private fun ticksToRadians(): Double {
        // FIX: gear ratio was inverted — output/motor not motor/output
        val gearRatio = outputGearTeeth.toDouble() / motorGearTeeth.toDouble()
        return (2.0 * PI / ENCODER_CPR) * (1.0 / gearRatio)
    }

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    fun setAlliance(red: Boolean) {
        alliance = if (red) Drive.Alliance.RED else Drive.Alliance.BLUE
    }

    fun reZeroEncoder() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}