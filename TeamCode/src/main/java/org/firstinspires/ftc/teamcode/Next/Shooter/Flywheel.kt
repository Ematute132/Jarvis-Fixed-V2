package org.firstinspires.ftc.teamcode.Next.Shooter

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control2.feedback.PIDController
import dev.nextftc.control2.feedforward.SimpleFFCoefficients
import dev.nextftc.control2.feedforward.SimpleFeedforward
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.MotorGroup
import dev.nextftc.hardware.impl.MotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.ActiveOpMode.hardwareMap
import kotlin.math.round

@Configurable
object FlyWheel: Subsystem {
    val topFlywheelMotor: MotorEx = MotorEx("Fly1")
    val bottomFlywheelMotor: MotorEx = MotorEx("Fly2")
    val flywheelMotors: MotorGroup = MotorGroup(topFlywheelMotor, bottomFlywheelMotor)

    private val battery: VoltageSensor by lazy { hardwareMap.get(VoltageSensor::class.java, "Control Hub") }

    @JvmField var kP: Double = 0.01
    @JvmField var kI: Double = 0.0
    @JvmField var kD: Double = 0.01

    @JvmField var ffKV: Double = 0.02
    @JvmField var ffKA: Double = 0.0002
    @JvmField var ffKS: Double = 0.0

    private const val V_NOMINAL = 12.0
    const val IDLE_VELOCITY: Double = 0.0

    var flywheelTarget: Double = 0.0

    private var velFilt = 0.0
    private var voltFilt = 12.0
    private const val ALPHA_VEL = 0.25
    private const val ALPHA_VOLT = 0.3

    @JvmField var voltageCompEnabled = true

    var usePID = true

    fun isAtTarget(): Boolean {
        val rounded = roundToNearest20(flywheelTarget)
        return ((rounded - 20.0) < flywheelMotors.velocity) && ((rounded + 40.0) > flywheelMotors.velocity)
    }

    // FIX: override periodic so NextFTC actually calls this every loop
    override fun periodic() {
        val pid = PIDController(kP, kI, kD)
        val ff = SimpleFeedforward(SimpleFFCoefficients(ffKV, ffKA, ffKS))

        val target = roundToNearest20(flywheelTarget)
        val velRaw = flywheelMotors.velocity
        val voltRaw = battery.voltage.coerceAtLeast(9.0)

        velFilt += ALPHA_VEL * (velRaw - velFilt)
        voltFilt += ALPHA_VOLT * (voltRaw - voltFilt)

        val error = target - velFilt

        val pidOut = pid.calculate(error = error)
        val ffOut = ff.calculate(target)

        var raw = (pidOut + ffOut).coerceIn(-1.0, 1.0)

        if (!usePID) {
            raw = ffOut.coerceIn(-1.0, 1.0)
        }

        val pow = if (voltageCompEnabled) {
            (raw * (V_NOMINAL / voltFilt)).coerceIn(-1.0, 1.0)
        } else {
            raw
        }

        flywheelMotors.power = pow

        ActiveOpMode.telemetry.addData("Flywheel/Power", pow)
        ActiveOpMode.telemetry.addData("Flywheel/Target", flywheelTarget)
        ActiveOpMode.telemetry.addData("Flywheel/Velocity", topFlywheelMotor.velocity)
        ActiveOpMode.telemetry.addData("Flywheel/AtTarget", if (isAtTarget()) "YES" else "NO")
        ActiveOpMode.telemetry.addData("Flywheel/kP", "%.4f".format(kP))
        ActiveOpMode.telemetry.addData("Flywheel/kV", "%.4f".format(ffKV))
    }

    fun roundToNearest20(velocity: Double): Double {
        return round(velocity / 20.0) * 20.0
    }

    fun setVelocity(velocity: Double) {
        flywheelTarget = velocity
    }

    val close = InstantCommand { setVelocity(1200.0) }
    val mid   = InstantCommand { setVelocity(1500.0) }
    val far   = InstantCommand { setVelocity(1900.0) }
    val idle  = InstantCommand { setVelocity(IDLE_VELOCITY) }
    val stop = InstantCommand{setVelocity(0.0)}
}