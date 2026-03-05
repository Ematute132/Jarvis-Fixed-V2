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

@TeleOp(name = "TeleOp - Blue", group = "Competition")
class TeleOpBlue : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    
    // Shooting velocities for each zone
    private val closeVelocity = 1100.0
    private val midVelocity = 1500.0
    private val farVelocity = 1900.0
    
    // Auto aim enabled flag
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

    override fun onInit() {
        Turret.alliance = Turret.Alliance.BLUE
        // Load saved pose from auto
        Drive.loadSavedPose()
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()
        bindControls()
    }

    private fun bindControls() {
        // Intake
        Gamepads.gamepad1.rightTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        Gamepads.gamepad1.leftTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.reverse)
            .whenBecomesFalse(Intake.stop)

        // Gate
        Gamepads.gamepad1.cross
            .whenBecomesTrue(Gate.open)
            .whenBecomesFalse(Gate.close)

        // Dpad Up - Manual Mid shot
        Gamepads.gamepad1.dpadUp.whenBecomesTrue {
            autoAimEnabled = false
            FlyWheel.setVelocity(midVelocity)
            Hood.mid()
        }

        // Dpad Down - Manual Far shot
        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            autoAimEnabled = false
            FlyWheel.setVelocity(farVelocity)
            Hood.far()
        }

        // Dpad Left - Nudge left
        Gamepads.gamepad1.dpadLeft.whenBecomesTrue { 
            Turret.nudgeLeft() 
        }

        // Dpad Right - Nudge right
        Gamepads.gamepad1.dpadRight.whenBecomesTrue { 
            Turret.nudgeRight() 
        }
        
        // Left Bumper - Toggle Auto Aim (SOTM)
        Gamepads.gamepad1.leftBumper.whenBecomesTrue {
            autoAimEnabled = !autoAimEnabled
            if (autoAimEnabled) {
                Turret.lock()
            }
        }

        // Square - Reset pose to corner
        Gamepads.gamepad1.square.whenBecomesTrue {
            follower.pose = Pose(136.0, 8.0, 0.0)
            Turret.lock()
        }

        // Triangle - Reset turret to point at goal
        Gamepads.gamepad1.triangle.whenBecomesTrue {
            Turret.resetToGoal()
        }
        
        // Circle - Clear turret offset
        Gamepads.gamepad1.circle.whenBecomesTrue {
            Turret.clearOffset()
        }
    }

    override fun onUpdate() {
        // Update drive
        Drive.update()
        
        // Auto aim based on zone
        if (autoAimEnabled) {
            val zone = Drive.getShootingZone()
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
        
        // Always keep turret locked
        Turret.currentState = Turret.State.LOCKED
        
        updateTelemetry()
    }

    override fun onStop() {
        Intake.stop()
        Gate.close()
        Hood.close()
        FlyWheel.stop()
        Turret.currentState = Turret.State.IDLE
        // Save pose for next opmode
        Drive.savePose()
    }

    private fun updateTelemetry() {
        telemetry.addData("=== BLUE TELEOP ===", "")
        telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Pose/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        telemetry.addData("Hood/Position", "%.2f".format(Hood.currentPosition))
        telemetry.addData("Hood/Preset", Hood.currentPreset.name)
        telemetry.addData("Zone", if (Drive.isInShootingZone()) "YES" else "NO")
        telemetry.addData("Shooting Zone", Drive.getShootingZone().name)
        telemetry.addData("Auto Aim", if (autoAimEnabled) "ON" else "OFF")
        telemetry.addData("Turret Offset", "%.2f°".format(Math.toDegrees(Turret.angleOffsetRad)))
        telemetry.update()
        panelsTelemetry.update()
    }
}
