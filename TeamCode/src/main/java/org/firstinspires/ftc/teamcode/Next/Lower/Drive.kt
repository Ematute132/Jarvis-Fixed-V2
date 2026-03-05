package org.firstinspires.ftc.teamcode.Lower.Drive

import com.pedropathing.geometry.Pose

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.FieldConstants
import org.firstinspires.ftc.teamcode.SOTMConstants
import kotlin.math.*

/**
 * Drive Subsystem
 * Provides all position and aiming data from Pedro pathing
 */
object Drive : Subsystem {

    // ==================== STATE ====================
    var currentX = 0.0
    var currentY = 0.0
    var currentHeading = 0.0

    private var lastX = 0.0
    private var lastY = 0.0
    private var lastTime = 0L

    var velocityX = 0.0
    var velocityY = 0.0

    var poseValid = false

    // Saved pose for auto-to-teleop transfer
    var savedPose: Pose? = null
    
    // For heading drift correction
    private var headingCorrectionEnabled = true

    // ==================== ALLIANCE ====================
    enum class Alliance { RED, BLUE }
    var alliance = Alliance.BLUE

    val goalX: Double get() = if (alliance == Alliance.RED) FieldConstants.RED_GOAL_X else FieldConstants.BLUE_GOAL_X
    val goalY = FieldConstants.GOAL_Y

    // ==================== SHOOTING ZONES ====================
    // Exact shooting positions
    val closeShootingPose = Pose(20.0, 125.0, 0.0)
    val midShootingPose = Pose(69.0, 72.0, 0.0)
    val farShootingPose = Pose(80.0, 8.0, 0.0)

    // Distance thresholds
    const val CLOSE_THRESHOLD = 15.0  // inches
    const val MID_THRESHOLD = 20.0

    override fun initialize() {
        // Apply saved pose from auto if available
        savedPose?.let { pose ->
            follower.pose = pose
            currentX = pose.x
            currentY = pose.y
            currentHeading = pose.heading
        }
    }

    fun update() {
        val pose = follower.pose
        currentX = pose.x
        currentY = pose.y
        currentHeading = pose.heading
        poseValid = true

        // Calculate velocity
        val now = System.currentTimeMillis()
        val dt = (now - lastTime) / 1000.0
        if (dt > 0 && dt < 0.2) {
            velocityX = (currentX - lastX) / dt
            velocityY = (currentY - lastY) / dt
        }
        lastX = currentX
        lastY = currentY
        lastTime = now
    }

    // Save current pose (call at end of auto)
    fun savePose() {
        savedPose = follower.pose
    }

    // Load saved pose (call at start of teleop)
    fun loadSavedPose() {
        savedPose?.let { pose ->
            follower.pose = pose
            currentX = pose.x
            currentY = pose.y
            currentHeading = pose.heading
        }
    }

    // Clear saved pose
    fun clearSavedPose() {
        savedPose = null
    }

    // ==================== DISTANCE CALCULATIONS ====================
    fun distanceToGoal(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return sqrt(dx * dx + dy * dy)
    }

    fun distanceToGoalMeters(): Double = distanceToGoal() * 0.0254

    fun distanceToRedGoal(): Double {
        val dx = FieldConstants.RED_GOAL_X - currentX
        val dy = FieldConstants.GOAL_Y - currentY
        return sqrt(dx * dx + dy * dy)
    }

    fun distanceToBlueGoal(): Double {
        val dx = FieldConstants.BLUE_GOAL_X - currentX
        val dy = FieldConstants.GOAL_Y - currentY
        return sqrt(dx * dx + dy * dy)
    }

    // Distance to shooting poses
    fun distanceToClosePose(): Double {
        return sqrt((closeShootingPose.x - currentX)**2 + (closeShootingPose.y - currentY)**2)
    }
    
    fun distanceToMidPose(): Double {
        return sqrt((midShootingPose.x - currentX)**2 + (midShootingPose.y - currentY)**2)
    }
    
    fun distanceToFarPose(): Double {
        return sqrt((farShootingPose.x - currentX)**2 + (farShootingPose.y - currentY)**2)
    }

    // Get current shooting zone based on closest pose
    fun getShootingZone(): ShootingZone {
        val distClose = distanceToClosePose()
        val distMid = distanceToMidPose()
        val distFar = distanceToFarPose()
        
        return when {
            distClose <= CLOSE_THRESHOLD -> ShootingZone.CLOSE
            distMid <= MID_THRESHOLD -> ShootingZone.MID
            else -> ShootingZone.FAR
        }
    }

    enum class ShootingZone { CLOSE, MID, FAR }

    // ==================== ANGLE CALCULATIONS ====================
    fun angleToGoal(): Double {
        val dx = goalX - currentX
        val dy = goalY - currentY
        return Math.toDegrees(atan2(dy, dx))
    }

    fun angleToGoalRobotFrame(): Double {
        val fieldAngle = angleToGoal()
        val robotHeadingDeg = Math.toDegrees(currentHeading)
        var turretAngle = fieldAngle - robotHeadingDeg - 90.0
        return normalizeAngleDegrees(turretAngle)
    }

    // ==================== SHOOTING ZONE CHECK ====================
    fun isInShootingZone(): Boolean {
        return FieldConstants.isInMainShootingZone(currentX, currentY)
    }

    fun isInAnyShootingZone(): Boolean {
        return FieldConstants.isInMainShootingZone(currentX, currentY) ||
                FieldConstants.isInSecondaryShootingZone(currentX, currentY)
    }

    // ==================== SHOOTING ON THE MOVE ====================
    fun getVirtualGoalX(): Double {
        return goalX + velocityX * SOTMConstants.TIME_OF_FLIGHT
    }

    fun getVirtualGoalY(): Double {
        return goalY + velocityY * SOTMConstants.TIME_OF_FLIGHT
    }

    fun angleToVirtualGoal(): Double {
        val vx = getVirtualGoalX()
        val vy = getVirtualGoalY()
        val dx = vx - currentX
        val dy = vy - currentY
        return Math.toDegrees(atan2(dy, dx))
    }

    fun distanceToVirtualGoal(): Double {
        val vx = getVirtualGoalX()
        val vy = getVirtualGoalY()
        val dx = vx - currentX
        val dy = vy - currentY
        return sqrt(dx * dx + dy * dy)
    }

    // ==================== UTILITIES ====================
    private fun normalizeAngleDegrees(degrees: Double): Double {
        var angle = degrees % 360
        if (angle > 180) angle -= 360
        if (angle < -180) angle += 360
        return angle
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        update()

        ActiveOpMode.telemetry.run {
            addData("=== DRIVE ===", "")
            addData("Drive/Valid", if (poseValid) "YES" else "NO")
            if (poseValid) {
                addData("Drive/X", "%.1f".format(currentX))
                addData("Drive/Y", "%.1f".format(currentY))
                addData("Drive/Heading", "%.1f°".format(Math.toDegrees(currentHeading)))
                addData("Drive/In Zone", if (isInShootingZone()) "YES" else "NO")
                addData("Drive/Dist Goal", "%.1f\"".format(distanceToGoal()))
                addData("Drive/Angle Goal", "%.1f°".format(angleToGoal()))
                addData("Drive/Zone", getShootingZone().name)
            }
        }
    }
}
