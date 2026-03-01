package org.firstinspires.ftc.teamcode.Next.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.FieldConstants
import org.firstinspires.ftc.teamcode.FieldConstants.BLUE_GOAL_X
import org.firstinspires.ftc.teamcode.FieldConstants.GOAL_Y
import org.firstinspires.ftc.teamcode.FieldConstants.RED_GOAL_X
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive.currentX
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive.currentY
import org.firstinspires.ftc.teamcode.TurretConstants.motorGearTeeth
import org.firstinspires.ftc.teamcode.TurretConstants.outputGearTeeth

import kotlin.math.*

const val TURRET_MIN_ANGLE = -135.0
const val TURRET_MAX_ANGLE = 90.0
const val ENCODER_CPR = 4000

@Configurable
object Turret : Subsystem {

    enum class State { IDLE, MANUAL, LOCKED, RESET }

    @JvmField var kP: Double = 0.5
    @JvmField var kI: Double = 0.0
    @JvmField var kD: Double = 0.001
    @JvmField var ffKV: Double = 0.11
    @JvmField var ffKA: Double = 0.0
    @JvmField var ffKS: Double = 0.05
    @JvmField var maxPower: Double = 1.0
    @JvmField var alignmentTolerance: Double = 2.0
    @JvmField var sotmLookahead: Double = 0.0

    var motor = MotorEx("turret")
    var currentState = State.IDLE
    var manualPower = 0.0
    var targetYaw = 0.0
    var isLocked = false

    private val velTimer = ElapsedTime()
    private var lastRobotHeading = 0.0
    var robotAngularVelocity = 0.0

    enum class Alliance { RED, BLUE }
    var alliance = Alliance.BLUE

    // ==================== GOAL DATA ====================
    /** Get goal X based on alliance */
    val goalX: Double get() = if (alliance == Alliance.RED) RED_GOAL_X else BLUE_GOAL_X

    /** Goal Y (same for both alliances) */
    val goalY = GOAL_Y



    private val MIN_ANGLE = Math.toRadians(TURRET_MIN_ANGLE)
    private val MAX_ANGLE = Math.toRadians(TURRET_MAX_ANGLE)

    private var controller = buildController()

    private fun buildController() = controlSystem {
        posPid(kP, kI, kD)
        basicFF(ffKV, ffKA, ffKS)
    }

    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        velTimer.reset()
        lastRobotHeading = Drive.currentHeading
        robotAngularVelocity = 0.0
        controller = buildController()
        targetYaw = getYaw()
    }

    override fun periodic() {
        if (currentState == State.LOCKED) {
            updateRobotAngularVelocity()
        }

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-1.0, 1.0)
            State.LOCKED -> runLockedControl()
            State.RESET -> runResetControl()
        }

        telemetry.addData("Turret/State", currentState.name)
        telemetry.addData("Turret/Yaw", "%.2f°".format(Math.toDegrees(getYaw())))
        telemetry.addData("Turret/Target", "%.2f°".format(Math.toDegrees(targetYaw)))
        telemetry.addData("Turret/Locked", if (isLocked) "YES" else "NO")
        telemetry.addData("Turret/RobotVel", "%.2f°/s".format(Math.toDegrees(robotAngularVelocity)))
        telemetry.addData("Turret/SOTM", "lookahead=%.2fs".format(sotmLookahead))
        telemetry.addData("Turret/Power", motor.power)
        telemetry.update()
    }

    private fun applyControl(targetYaw: Double, targetVelocity: Double = 0.0) {
        val clampedTarget = targetYaw.coerceIn(MIN_ANGLE, MAX_ANGLE)
        val currentYaw = getYaw()
        controller.goal = KineticState(clampedTarget, targetVelocity)
        val power = controller.calculate(KineticState(currentYaw, 0.0))
        motor.power = power.coerceIn(-maxPower, maxPower)
        this.targetYaw = clampedTarget
    }

    fun runLockedControl() {
        if (!Drive.poseValid) return

        //val predictedX = Drive.currentX + Drive.velocityX * sotmLookahead
        //val predictedY = Drive.currentY + Drive.velocityY * sotmLookahead

        val deltaX = goalX - currentX
        val deltaY = goalY - currentY
        val fieldAngleToGoal = atan2(deltaY, deltaX)

        val robotHeadingRad = Drive.currentHeading
        val rawTarget = normalizeAngle(fieldAngleToGoal - robotHeadingRad)

        applyControl(rawTarget, -robotAngularVelocity)
    }

     fun runResetControl() {
        val currentYaw = getYaw()
        val error = normalizeAngle(0.0 - currentYaw)
        val errorDeg = Math.toDegrees(abs(error))

        if (errorDeg < alignmentTolerance) {
            motor.power = 0.0
            targetYaw = 0.0
            currentState = State.IDLE
            return
        }

        applyControl(0.0)
    }

    private fun updateRobotAngularVelocity() {
        if (!Drive.poseValid) {
            robotAngularVelocity = 0.0
            return
        }

        val dt = velTimer.seconds()
        if (dt < 0.01 || dt > 0.2) {
            if (dt > 0.2) robotAngularVelocity = 0.0
            velTimer.reset()
            return
        }

        val currentHeading = Drive.currentHeading
        if (currentHeading.isNaN() || currentHeading.isInfinite()) {
            robotAngularVelocity = 0.0
            velTimer.reset()
            return
        }

        val deltaHeading = normalizeAngle(currentHeading - lastRobotHeading)
        robotAngularVelocity = deltaHeading / dt
        lastRobotHeading = currentHeading
        velTimer.reset()
    }

    fun lock() {
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

    fun getYaw(): Double = normalizeAngle(
        motor.currentPosition.toDouble() * ticksToRadians()
    )

    private fun ticksToRadians(): Double {
        val gearRatio = outputGearTeeth.toDouble() / motorGearTeeth.toDouble()
        return (2.0 * PI / ENCODER_CPR) * (1.0 / gearRatio)
    }

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }
}