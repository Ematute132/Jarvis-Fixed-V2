package org.firstinspires.ftc.teamcode.Test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.ftc.ActiveOpMode.telemetry
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive

import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.AutoAim.AutoAim
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel

import kotlin.math.abs

/**
 * Test AutoAim Opmode
 *
 * Tests automatic adjustment of flywheel and hood based on distance
 *
 * CONTROLS:
 * - A Button: Toggle auto-aim on/off
 * - D-Pad: Manual flywheel presets
 */
@TeleOp(name = "TEST - AutoAim", group = "Test")
class TestAutoAim : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Drive, FlyWheel, Hood, AutoAim),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        // Enable auto aim by default
        AutoAim.setAutoAim(true)

        bindControls()
    }

    private fun bindControls() {
        // Toggle auto aim
        Gamepads.gamepad1.a.whenBecomesTrue {
            AutoAim.enabled = !AutoAim.enabled
            AutoAim.setAutoAim(AutoAim.enabled)
        }

        // Manual flywheel
        Gamepads.gamepad1.dpadUp.whenBecomesTrue { FlyWheel.far() }
        Gamepads.gamepad1.dpadRight.whenBecomesTrue { FlyWheel.mid() }
        Gamepads.gamepad1.dpadDown.whenBecomesTrue { FlyWheel.close() }
        Gamepads.gamepad1.dpadLeft.whenBecomesTrue { FlyWheel.idle() }

        // Manual hood
        Gamepads.gamepad1.x.whenBecomesTrue { Hood.close() }
        Gamepads.gamepad1.b.whenBecomesTrue { Hood.mid() }
        Gamepads.gamepad1.y.whenBecomesTrue { Hood.far() }
    }

    override fun onUpdate() {
        // Update drive
        Drive.update()

        // Update auto aim


        telemetry.addData("=== AUTO AIM TEST ===", "")
        telemetry.addData("Enabled", if (AutoAim.enabled) "YES" else "NO")

        // Distance
        telemetry.addData("Distance", "%.2fm".format(Drive.distanceToGoalMeters()))

        // Auto aim values
        telemetry.addData("", "")
        //telemetry.addData("Auto/RPM", "%.0f".format(AutoAim.targetRpm))
        telemetry.addData("Auto/Hood", "%.2f".format(AutoAim.targetHoodPosition))

        // Actual values
        telemetry.addData("", "")

        telemetry.addData("Actual/Hood", "%.2f".format(Hood.currentPosition))

        // Comparison
        telemetry.addData("", "")

        telemetry.addData("Compare/Hood Match", if (abs(Hood.currentPosition - AutoAim.targetHoodPosition) < 0.05) "YES" else "NO")

        telemetry.addData("", "")
        telemetry.addData("Controls:", "")
        telemetry.addData("A", "Toggle AutoAim")
        telemetry.addData("DPad", "Manual Flywheel")
        telemetry.addData("X/B/Y", "Manual Hood")
        telemetry.update()
    }
}