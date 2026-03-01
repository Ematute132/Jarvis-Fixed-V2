package org.firstinspires.ftc.teamcode.AutoAim

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.HoodConstants
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.ShooterConstants
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import kotlin.math.*

@Configurable
object AutoAim : Subsystem {

    var enabled = false

    @JvmField var sotmVelocityThreshold = 6.0

    var currentDistanceInches = 0.0
    var targetVelocity = 0.0
    var targetHoodPosition = 0.0

    override fun initialize() {
        enabled = false
        currentDistanceInches = 0.0
        targetVelocity = 0.0
        targetHoodPosition = 0.0
    }

    override fun periodic() {
        if (!enabled) return

        val robotSpeed = sqrt(Drive.velocityX * Drive.velocityX + Drive.velocityY * Drive.velocityY)
        val useSOTM = robotSpeed > sotmVelocityThreshold

        // Use virtual goal for SOTM, actual goal when stationary
        currentDistanceInches = if (useSOTM) Drive.distanceToVirtualGoal() else Drive.distanceToGoal()

        val distanceMeters = currentDistanceInches * 0.0254

        // FIX: FlyWheel.setVelocity takes ticks/s not RPM
        // ShooterConstants presets are named RPM but used as ticks/s — keep consistent
        targetVelocity = calculateFlywheelVelocity(distanceMeters)
        FlyWheel.setVelocity(targetVelocity)

        targetHoodPosition = Hood.getPositionForDistance(distanceMeters)
        Hood.setPosition(targetHoodPosition)

        ActiveOpMode.telemetry.addData("AutoAim/Enabled", "YES")
        ActiveOpMode.telemetry.addData("AutoAim/SOTM", if (useSOTM) "YES (speed=%.1f)".format(sqrt(Drive.velocityX * Drive.velocityX + Drive.velocityY * Drive.velocityY)) else "NO")
        ActiveOpMode.telemetry.addData("AutoAim/Distance", "%.1f in".format(currentDistanceInches))
        ActiveOpMode.telemetry.addData("AutoAim/Velocity", "%.0f t/s".format(targetVelocity))
        ActiveOpMode.telemetry.addData("AutoAim/Hood", "%.2f".format(targetHoodPosition))
    }

    fun setAutoAim(enable: Boolean) {
        enabled = enable
    }

    private fun calculateFlywheelVelocity(distanceMeters: Double): Double {
        return when {
            distanceMeters < HoodConstants.DISTANCE_CLOSE_THRESHOLD ->
                ShooterConstants.FLYWHEEL_CLOSE_RPM

            distanceMeters < HoodConstants.DISTANCE_MID_THRESHOLD -> {
                val t = (distanceMeters - HoodConstants.DISTANCE_CLOSE_THRESHOLD) /
                        (HoodConstants.DISTANCE_MID_THRESHOLD - HoodConstants.DISTANCE_CLOSE_THRESHOLD)
                ShooterConstants.FLYWHEEL_CLOSE_RPM + t *
                        (ShooterConstants.FLYWHEEL_MID_RPM - ShooterConstants.FLYWHEEL_CLOSE_RPM)
            }

            else -> {
                val range = HoodConstants.DISTANCE_MID_THRESHOLD * 0.5
                val t = ((distanceMeters - HoodConstants.DISTANCE_MID_THRESHOLD) / range).coerceIn(0.0, 1.0)
                ShooterConstants.FLYWHEEL_MID_RPM + t *
                        (ShooterConstants.FLYWHEEL_FAR_RPM - ShooterConstants.FLYWHEEL_MID_RPM)
            }
        }
    }

    data class AimParams(
        val distanceInches: Double,
        val velocity: Double,
        val hoodPosition: Double,
        val angleToGoal: Double
    )
}