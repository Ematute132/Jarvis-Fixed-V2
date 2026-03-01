package org.firstinspires.ftc.teamcode.Test

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry

import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Shooter.Limelight.Limelight
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower

/**
 * Test Turret ODO Opmode
 *
 * Tests turret aiming with odometry only
 *
 * CONTROLS:
 * - Left Stick X: Manual turret control
 * - A Button: Enable odometry aiming
 * - B Button: Reset turret
 * - X Button: Stop turret
 */
@TeleOp(name = "TEST - Turret ODO", group = "Test")
class TestTurretODO : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Drive, Turret, Limelight),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        // Drive.lastKnown = Pose(72.0, 72.0, 0.0)
        follower.pose = Pose(72.0,72.0,0.0)
    }

    override fun onStartButtonPressed() {
        // Start drivetrain
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        ).schedule()
        bindControls()
    }

    private fun bindControls() {
        // Left Stick: Manual control

        // A: Enable odometry aiming
        Gamepads.gamepad1.a.whenBecomesTrue {
          Turret.currentState = Turret.State.LOCKED
        }

        // B: Reset turret
        // X: Stop
        Gamepads.gamepad1.x.whenBecomesTrue {
            Turret.stop()
        }
    }

    override fun onUpdate() {
        // Update drive
        Drive.update()
        Limelight.update()

        telemetry.addData("=== TURRET ODO TEST ===", "")
        telemetry.addData("State", Turret.currentState.name)


        // Show position
        telemetry.addData("", "")
        telemetry.addData("Robot/X", "%.1f".format(Drive.currentX))
        telemetry.addData("Robot/Y", "%.1f".format(Drive.currentY))
        telemetry.addData("Robot/Heading", "%.1f°".format(Math.toDegrees(Drive.currentHeading)))
        telemetry.addData("Dist to Goal", "%.1f\"".format(Drive.distanceToGoal()))

        // Show zone
        telemetry.addData("In Zone", if (Drive.isInShootingZone()) "YES" else "NO")

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("L Stick", "Manual")
        telemetry.addData("A", "Aim ODO")
        telemetry.addData("B", "Reset")
        telemetry.addData("X", "Stop")
        telemetry.update()
    }
}