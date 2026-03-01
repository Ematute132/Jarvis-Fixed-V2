package org.firstinspires.ftc.teamcode.TeleOp

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent

import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel

import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret.currentState
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret.getYaw
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret.isLocked
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret.motor
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret.robotAngularVelocity
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret.sotmLookahead
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret.targetYaw
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "TeleOp - Red", group = "Competition")
class TeleOpRed : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    // ==================== INITIALIZATION ====================
    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                Drive,
                Intake,
                Gate,
                FlyWheel,
                Turret,
                Hood,
                Limelight,
                AutoAim
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        // Set to blue alliance
        // follower.pose = Pose(72.0,72.0,0.0)
        Turret.alliance = Turret.Alliance.BLUE
        follower.pose = Drive.lastKnown

    }

    override fun onStartButtonPressed() {
        // Start drivetrain - field centric
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()
        bindControls()
    }

    // ==================== CONTROLS ====================
    private fun bindControls() {
        // Left Trigger: Intake
        Gamepads.gamepad1.rightTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        // Left Bumper: Reverse/Eject
        Gamepads.gamepad1.leftTrigger.greaterThan(0.1) whenBecomesTrue(Intake.reverse) whenBecomesFalse(Intake.stop)

        // Right Trigger: Manual fire (when at target)
        Gamepads.gamepad1.cross whenBecomesTrue Gate.open whenBecomesFalse Gate.close
        // --- FLYWHEEL PRESETS ---
        // D-Pad Up: Far
        Gamepads.gamepad1.dpadUp.whenBecomesTrue {
            FlyWheel.setVelocity(1900.0).also({Hood.far()})
        }

        // D-Pad Right: Mid
        Gamepads.gamepad1.dpadLeft.whenBecomesTrue {
            FlyWheel.setVelocity(1500.0).also({Hood.mid()})
        }

        // D-Pad Down: Close
        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            FlyWheel.setVelocity(1150.0).also({Hood.close()})
        }
        Gamepads.gamepad1.square.whenBecomesTrue { follower.pose = Pose(0.0,0.0,180.0) }

    }
    // ==================== UPDATE LOOP ====================
    override fun onUpdate() {
        // Update drive position
        Drive.update()
        currentState = Turret.State.LOCKED
        // Update telemetry
        updateTelemetry()
    }
    override fun onStop() {
        Intake.stop
        Gate.close
        Hood.close
        FlyWheel.stop
        currentState = Turret.State.IDLE
    }

    // ==================== TELEMETRY ====================
    private fun updateTelemetry() {
        // Position
        telemetry.addData("=== BLUE TELEOP ===", "")
        telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Pose/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        // Hood
        telemetry.addData("Hood/Position", "%.2f".format(Hood.currentPosition))
        telemetry.addData("Hood/Preset", Hood.currentPreset.name)
        telemetry.addData("Zone", if (Drive.isInShootingZone()) "YES" else "NO")

        telemetry.addData("last pose x",Drive.lastKnown.x)
        telemetry.addData("last pose y",Drive.lastKnown.y)
        telemetry.addData("last pose h",Drive.lastKnown.heading)

        telemetry.update()
        panelsTelemetry.update()
    }
}