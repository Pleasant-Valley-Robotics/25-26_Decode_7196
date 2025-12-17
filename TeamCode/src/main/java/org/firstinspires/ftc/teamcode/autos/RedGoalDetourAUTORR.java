package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Config
@Autonomous(name = "RedGoalAUTORR", group = "Autonomous")
public class RedGoalDetourAUTORR extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-55.135, 49.0834, Math.toRadians(131.014));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Camera camera = new Camera(hardwareMap);

// This is supposed to go to the coordinates of the shooting distance (-30.6209, 21.5313) with heading 129.5463
//This is the coordinates for the ending position of Goal AUTO (-61.7134, 17.4823) with heading -177.7059
        Vector2d shootPosition = new Vector2d(-62.067, 11.5763);
        TrajectoryActionBuilder goToShoot = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(90.1629))
                .waitSeconds(1.0);


        Vector2d endingPosition = new Vector2d(-61.6131, 11.5718);

        while (!isStopRequested() && !opModeIsActive()) {
            //add anything for during initialization
        }
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        goToShoot.build(),
                        new SleepAction(1.0),
                        launcher.ShootBall(),
                        new SleepAction(1.0),
                        launcher.ShootBall(),
                        new SleepAction(1.0),
                        launcher.ShootBall(),
                        new SleepAction(1.0)
                )
        );
        TrajectoryActionBuilder goToEnd = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(endingPosition, Math.toRadians(-177.7059))
                .waitSeconds(1.0);
        Actions.runBlocking(
                goToEnd.build()
        );
    }
}