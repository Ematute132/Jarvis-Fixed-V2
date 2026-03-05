package org.firstinspires.ftc.teamcode.Autonomoous.Blue

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.Lower.Drive.Drive
import org.firstinspires.ftc.teamcode.Lower.Gate.Gate
import org.firstinspires.ftc.teamcode.Lower.Intake.Intake
import org.firstinspires.ftc.teamcode.Next.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.Next.Shooter.Turret
import org.firstinspires.ftc.teamcode.Shooter.Hood.Hood
import org.firstinspires.ftc.teamcode.Systems.Command
import org.firstinspires.ftc.teamcode.Systems.ShooterCommands
import org.firstinspires.ftc.teamcode.Systems.initCommands
import org.firstinspires.ftc.teamcode.Systems.intakeAuto
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Configurable
@Autonomous(name = "7-ball Blue", group = "Competition", preselectTeleOp = "TeleOp - Blue")
class AutoBlue7 : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(
                Drive, Intake, Gate, FlyWheel, Turret, Hood
            ),
            BulkReadComponent, BindingsComponent
        )
    }

    private var paths: Array<PathChain> = arrayOf()

    override fun onInit() {
        Turret.alliance = Turret.Alliance.BLUE
    }

    override fun onStartButtonPressed() {
        follower.setStartingPose(Pose(20.900, 123.200, Math.toRadians(144.0)))
        buildPaths()

        val main = SequentialGroup(
            initCommands.farInit(),

            // Drive to score pose, shoot preload
            FollowPath(paths[0], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            // intakegate: score pose → gate balls, intake, return, shoot
            FollowPath(paths[1], true, 1.0),
            intakeAuto.autoIntake(2.0),
            FollowPath(paths[2], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            // ClearGate: score pose → clear gate, intake, return, shoot
            FollowPath(paths[3], true, 1.0),
            intakeAuto.autoIntake(2.0),
            FollowPath(paths[4], true, 1.0),
            ShooterCommands.shootCommand(1.0),

            // spike1: score pose → spike, intake, return, shoot
            FollowPath(paths[5], true, 1.0),
            intakeAuto.autoIntake(2.0),
            FollowPath(paths[6], true, 1.0),
            ShooterCommands.shootCommand(1.0)
        )
        main.schedule()
    }

    private fun buildPaths() {
        paths = arrayOf()

        // Path 0: start → score pose
        val scorePose0 = follower.pathBuilder()
            .addPath(BezierLine(Pose(20.900, 123.200), Pose(58.700, 85.000)))
            .setLinearHeadingInterpolation(Math.toRadians(144.0), Math.toRadians(180.0))
            .build()

        // Path 1: score pose → gate balls
        val intakegate = follower.pathBuilder()
            .addPath(BezierCurve(
                Pose(58.700, 85.000),
                Pose(54.088, 56.595),
                Pose(19.945, 50.929),
                Pose(10.524, 60.286)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(144.0))
            .build()

        // Path 2: gate balls → score pose
        val scorePose1 = follower.pathBuilder()
            .addPath(BezierCurve(
                Pose(10.524, 60.286),
                Pose(53.469, 63.881),
                Pose(58.700, 85.000)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(144.0), Math.toRadians(180.0))
            .build()

        // Path 3: score pose → clear gate
        val clearGate = follower.pathBuilder()
            .addPath(BezierCurve(
                Pose(58.700, 85.000),
                Pose(54.088, 56.596),
                Pose(19.940, 50.920),
                Pose(10.520, 60.280)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(142.0))
            .build()

        // Path 4: clear gate → score pose
        val scorePose2 = follower.pathBuilder()
            .addPath(BezierCurve(
                Pose(10.520, 60.280),
                Pose(53.469, 63.880),
                Pose(58.700, 85.000)
            ))
            .setLinearHeadingInterpolation(Math.toRadians(144.0), Math.toRadians(180.0))
            .build()

        // Path 5: score pose → spike1
        val spike1 = follower.pathBuilder()
            .addPath(BezierLine(Pose(58.700, 85.000), Pose(16.200, 84.400)))
            .setTangentHeadingInterpolation()
            .build()

        // Path 6: spike1 → score pose
        val scorePose3 = follower.pathBuilder()
            .addPath(BezierLine(Pose(16.200, 84.400), Pose(58.700, 85.000)))
            .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(90.0))
            .build()

        paths += scorePose0
        paths += intakegate
        paths += scorePose1
        paths += clearGate
        paths += scorePose2
        paths += spike1
        paths += scorePose3
    }

    override fun onUpdate() {
        Drive.update()
        telemetry.update()
    }

    override fun onStop() {
        initCommands.autoStop().schedule()
        Drive.savePose()
    }
}