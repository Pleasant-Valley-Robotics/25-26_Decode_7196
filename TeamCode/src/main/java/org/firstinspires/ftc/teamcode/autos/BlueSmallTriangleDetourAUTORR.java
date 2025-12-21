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
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.utility.Storage;

@Config
@Autonomous(name = "BlueSmallTriangleDetourAUTORR", group = "Autonomous")
public class BlueSmallTriangleDetourAUTORR extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(62.1499, -17.8955, Math.toRadians(-179.1488));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Camera camera = new Camera(hardwareMap);

// This is supposed to go to the coordinates of the shooting distance (-30.6209, 21.5313) with heading 129.5463
//This is the coordinates for the ending position of Goal AUTO (-61.7134, 17.4823) with heading -177.7059
        Vector2d shootPosition = new Vector2d(-62.067, -11.5763);
        TrajectoryActionBuilder goToShoot = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(-90.1629))
                .waitSeconds(1.0);

        Vector2d endingPosition = new Vector2d(-62.0308, -12.9117);

        while (!isStopRequested() && !opModeIsActive()) {
            //add anything for during initialization
            if (gamepad1.b) {
                Storage.alliance = Storage.Alliance.RED;
            } else if (gamepad1.x) {
                Storage.alliance = Storage.Alliance.BLUE;
            }
            telemetry.addData("Press X", "for BLUE");
            telemetry.addData("Press B", "for RED");
            telemetry.addData("Selected Alliance", Storage.alliance);

            telemetry.addData("Shots To Cycle", camera.findShotsToCycle());
            telemetry.update();
        }



        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Shots To Cycle", camera.findShotsToCycle());
        telemetry.update();
        for (int ShotsToCycle = camera.findShotsToCycle(); ShotsToCycle>0; ShotsToCycle--)
        {
            Actions.runBlocking(
                    new SequentialAction(
                            launcher.IndexBall(),
                            new SleepAction(1.5)
                    )
            );
        } // end indexing

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
                .strafeToLinearHeading(endingPosition, Math.toRadians(-90.202))
                .waitSeconds(1.0);
        Actions.runBlocking(
                goToEnd.build()
        );
        Storage.pose = drive.localizer.getPose();
    }
}