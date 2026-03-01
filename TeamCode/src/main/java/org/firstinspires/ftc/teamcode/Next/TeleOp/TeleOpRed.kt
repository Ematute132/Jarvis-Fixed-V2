package org.firstinspires.ftc.teamcode.TeleOp

import com.bylazar.telemetry.JoinedTelemetry
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
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel

import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.pedroPathing.Constants


@TeleOp(name = "TeleOp - Red", group = "Competition")
class TeleOpRed : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    private val joinedTelemetry = JoinedTelemetry(telemetry, panelsTelemetry)

    private var autoAimEnabled = false

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
        Drive.alliance = Drive.Alliance.RED
        Turret.setAlliance(red = true)
        Drive.lastKnown = Pose(72.0, 72.0, 0.0)
    }

    override fun onStartButtonPressed() {
        // Field-centric drive
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()


        AutoAim.setAutoAim(true)

        bindControls()
    }

    // ==================== CONTROLS ====================
    private fun bindControls() {
        // --- INTAKE: left trigger ---
        Gamepads.gamepad1.leftTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Intake.stop)

        // --- OUTTAKE: right trigger ---
        Gamepads.gamepad1.rightTrigger.greaterThan(0.1)
            .whenBecomesTrue(Intake.reverse)
            .whenBecomesFalse(Intake.stop)

        // --- SHOOT: right bumper (hold) → open gate + feed intake, release → close gate ---
        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue(Gate.open)
            .whenBecomesTrue(Intake.run)
            .whenBecomesFalse(Gate.close)
            .whenBecomesFalse(Intake.stop)

        // --- FLYWHEEL / HOOD PRESETS (D-Pad) ---
        Gamepads.gamepad1.dpadUp.whenBecomesTrue {
            FlyWheel.setVelocity(1900.0).also { Hood.far() }
        }

        Gamepads.gamepad1.dpadRight.whenBecomesTrue {
            FlyWheel.setVelocity(1500.0).also { Hood.mid() }
        }

        Gamepads.gamepad1.dpadDown.whenBecomesTrue {
            FlyWheel.setVelocity(1000.0).also { Hood.close() }
        }

        Gamepads.gamepad1.dpadLeft.whenBecomesTrue {
            FlyWheel.setVelocity(FlyWheel.IDLE_VELOCITY)
        }
        Gamepads.gamepad1.cross whenBecomesTrue { !autoAimEnabled }
        Gamepads.gamepad1.square whenBecomesTrue {autoAimEnabled}
    }

    // ==================== UPDATE LOOP ====================
    override fun onUpdate() {
        Drive.update()
        Limelight.update()

        // Auto aim updates flywheel + hood based on distance


        // Turret tracks goal
        Turret.runLockedControl()

        // Flywheel PID loop


        updateTelemetry()
    }

    // ==================== TELEMETRY ====================
    private fun updateTelemetry() {
        telemetry.addData("=== RED TELEOP ===", "")
        telemetry.addData("Pose/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Pose/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Pose/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        telemetry.addData("Flywheel Ready", if (FlyWheel.isAtTarget()) "YES" else "NO")
        telemetry.addData("Hood/Position", "%.2f".format(Hood.currentPosition))
        telemetry.addData("Hood/Preset", Hood.currentPreset.name)
        telemetry.addData("AutoAim", if (autoAimEnabled) "ON" else "OFF")
        telemetry.addData("Zone", if (Drive.isInShootingZone()) "YES" else "NO")
        Limelight.updateTelemetry()

        panelsTelemetry.update()
    }
}