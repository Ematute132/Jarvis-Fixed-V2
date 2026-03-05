package org.firstinspires.ftc.teamcode.Next.TeleOp

import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive.ShootingZone
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.*

/**
 * TeleOp - Red
 * 
 * CONTROLS:
 * 
 * Movement:
 * - Left Stick: Field-centric drive
 * - Right Stick X: Turn (field-centric rotation)
 * 
 * Intake:
 * - Right Trigger: Run intake
 * - Left Trigger: Reverse intake
 * 
 * Shooting:
 * - Cross (X): Open/close gate
 * - Dpad Up: Manual mid shot (1500 RPM)
 * - Dpad Down: Manual far shot (1900 RPM)
 * - Left Bumper: Toggle SOTM (auto aim)
 * - Square: Reset pose to corner (136, 8, 0)
 * - Triangle: Reset turret to point at goal
 * - Circle: Clear turret offset
 * 
 * Turret:
 * - Dpad Left: Nudge turret left
 * - Dpad Right: Nudge turret right
 * 
 * SOTM (Shooting On The Move):
 * When enabled (Left Bumper):
 * - Automatically adjusts flywheel velocity based on zone
 * - Automatically adjusts hood position
 * - Zone determined by distance to close/mid/far poses
 */

@TeleOp(name = "TeleOp - Red", group = "Competition")
class TeleOpRed : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    
    // ==================== SHOOTING VELOCITIES ====================
    // Flywheel RPM for each zone
    // Adjust these based on testing
    private val closeVelocity = 1100.0   // Close zone
    private val midVelocity = 1500.0     // Mid zone
    private val farVelocity = 1900.0     // Far zone
    
    // Auto aim enabled flag
    // When true, automatically adjusts velocity/hood based on zone
    private var autoAimEnabled = false

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                Drive, Intake, Gate, FlyWheel, Turret, Hood, Limelight, AutoAim
            ),
            BulkReadComponent, BindingsComponent
        )
    }

    /**
     * Initialize teleop
     * Sets alliance and loads saved pose from auto
     */
    override fun onInit() {
        // RED ALLIANCE
        Turret.alliance = Turret.Alliance.RED
        Drive.alliance = Drive.Alliance.RED
        
        // Load saved pose from auto - this is key!
        // Without this, robot starts at (0,0) every teleop
        Drive.loadSavedPose()
    }

    /**
     * Start button pressed - start teleop
     */
    override fun onStartButtonPressed() {
        // Start field-centric drive
        PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()
        
        bindControls()
    }

    /**
     * Bind gamepad controls
     * Each button/axis is mapped to a function
     */
    private fun bindControls() {
        // ========== INTAKE ==========
        
        // Right trigger: Run intake forward
        Gamepads.gamepad1.rightTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        // Left trigger: Reverse intake
        Gamepads.gamepad1.leftTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.reverse)
            .whenBecomesFalse(Intake.stop)

        // ========== GATE ==========
        
        // Cross (X): Toggle gate
        Gamepads.gamepad1.cross
            .whenBecomesTrue(Gate.open)
            .whenBecomesFalse(Gate.close)

        // ========== SHOOTING ==========
        
        // Dpad Up: Manual mid shot
        Gamepads.gamepad1.dpadUp.whenBecomesTrue {
            autoAimEnabled = false  // Disable auto aim
            FlyWheel.setVelocity(midVelocity)
            Hood.mid()
        }

        // Dpad Down: Manual far shot
        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            autoAimEnabled = false
            FlyWheel.setVelocity(farVelocity)
            Hood.far()
        }

        // ========== TURRET ==========
        
        // Dpad Left: Nudge left
        Gamepads.gamepad1.dpadLeft.whenBecomesTrue { 
            Turret.nudgeLeft() 
        }

        // Dpad Right: Nudge right
        Gamepads.gamepad1.dpadRight.whenBecomesTrue { 
            Turret.nudgeRight() 
        }
        
        // ========== AUTO AIM (SOTM) ==========
        
        // Left Bumper: Toggle shooting on the move
        Gamepads.gamepad1.leftBumper.whenBecomesTrue {
            autoAimEnabled = !autoAimEnabled
            if (autoAimEnabled) {
                Turret.lock()  // Enable auto aim
            }
        }

        // ========== POSE RESET ==========
        
        // Square: Reset pose to corner position
        // Heading 0 = facing right toward goal on red side
        Gamepads.gamepad1.square.whenBecomesTrue {
            follower.pose = Pose(136.0, 8.0, Math.toRadians(0.0))
            Turret.lock()
        }

        // Triangle: Reset turret to point at goal
        // Just zeros turret at current position
        Gamepads.gamepad1.triangle.whenBecomesTrue {
            Turret.resetToGoal()
        }
        
        // Circle: Clear turret offset
        Gamepads.gamepad1.circle.whenBecomesTrue {
            Turret.clearOffset()
        }
    }

    /**
     * Main loop - runs every cycle
     */
    override fun onUpdate() {
        // Update drive position
        Drive.update()
        
        // If auto aim is enabled, adjust based on zone
        if (autoAimEnabled) {
            // Get current shooting zone (CLOSE/MID/FAR)
            val zone = Drive.getShootingZone()
            
            // Set velocity based on zone
            val velocity = when (zone) {
                ShootingZone.CLOSE -> closeVelocity
                ShootingZone.MID -> midVelocity
                ShootingZone.FAR -> farVelocity
            }
            FlyWheel.setVelocity(velocity)
            
            // Set hood based on zone
            when (zone) {
                ShootingZone.CLOSE -> Hood.close()
                ShootingZone.MID -> Hood.mid()
                ShootingZone.FAR -> Hood.far()
            }
        }
        
        // Always keep turret locked during teleop
        // This maintains auto aim while driving
        Turret.currentState = Turret.State.LOCKED
        
        updateTelemetry()
    }

    /**
     * Stop teleop
     * Save pose for next opmode
     */
    override fun onStop() {
        // Stop all subsystems
        Intake.stop()
        Gate.close()
        Hood.close()
        FlyWheel.stop()
        Turret.currentState = Turret.State.IDLE
        
        // Save pose - CRITICAL for auto-to-teleop handoff
        // This stores the current position so next opmode knows where robot is
        Drive.savePose()
    }

    /**
     * Update telemetry on screen
     */
    private fun updateTelemetry() {
        telemetry.addData("=== RED TELEOP ===", "")
        
        // Position data
        telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Pose/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        
        // Hood data
        telemetry.addData("Hood/Position", "%.2f".format(Hood.currentPosition))
        telemetry.addData("Hood/Preset", Hood.currentPreset.name)
        
        // Zone info
        telemetry.addData("Zone", if (Drive.isInShootingZone()) "YES" else "NO")
        telemetry.addData("Shooting Zone", Drive.getShootingZone().name)
        
        // Auto aim status
        telemetry.addData("Auto Aim", if (autoAimEnabled) "ON" else "OFF")
        
        // Turret offset
        telemetry.addData("Turret Offset", "%.2f°".format(Math.toDegrees(Turret.angleOffsetRad)))
        
        telemetry.update()
        panelsTelemetry.update()
    }
}
