package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.utility.Storage.alliance;

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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.Storage;

@Config
@Autonomous(name = "SmallTriangleAUTORR", group = "Autonomous")
public class SmallTriangleAUTORR extends LinearOpMode {
    int flipAuto = 1;
    @Override
    public void runOpMode() {

        while (!isStopRequested() && !opModeIsActive()) {
            //add anything for during initialization
            //telemetry.addData("Shots To Cycle", camera.findShotsToCycle());
            if (gamepad1.b) {
                flipAuto = 1;
                alliance = Storage.Alliance.RED;

            } else if (gamepad1.x) {
                flipAuto = -1;
                alliance = Storage.Alliance.BLUE;
            }

            telemetry.update();
            telemetry.addData("Press X","for BLUE");
            telemetry.addData("Press B","for RED");
            telemetry.addData("Selected Alliance", alliance);
        }

        Pose2d initialPose = new Pose2d(64.6427, 12.1699 * flipAuto, Math.toRadians(179.9565) * flipAuto);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        Vector2d endingPosition = new Vector2d(-61.6131, 11.5718 * flipAuto);

        waitForStart();

        if (isStopRequested()) return;
/*
        Vector2d indexPosition = new Vector2d(-22.2411, 6.7882);
        TrajectoryActionBuilder goToIndex = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(indexPosition, Math.toRadians(-150.2229))
                .waitSeconds(0.25);

            Actions.runBlocking(
                    new SequentialAction(
                            goToIndex.build()
                    )
            );

        for (int ShotsToCycle = camera.findShotsToCycle(); ShotsToCycle > 0; ShotsToCycle--)
        {
            if (isStopRequested()) return;
            Actions.runBlocking(
                    new SequentialAction(
                            launcher.IndexBall(),
                            new SleepAction(0.2)
                    )
            );
            telemetry.addData("Shots To Cycle", camera.findShotsToCycle());
            telemetry.update();
        }*/ // end indexing

        Vector2d shootPosition = new Vector2d(-30.6209, 21.5313 * flipAuto);
        TrajectoryActionBuilder goToShoot = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(shootPosition, Math.toRadians(129.5463) * flipAuto)
                //.turnTo(129.0 * Math.PI/180.0*flipAuto)
                //.turnTo(Math.9PI/2)
                .waitSeconds(0.1); // first time moving to the shooting position

        Actions.runBlocking(
                new SequentialAction(
                        goToShoot.build(),
                        new SleepAction(0.01),
                        launcher.ShootBall(),
                        intake.outtakeBall(),
                        intake.intakeBall(),
                        new SleepAction(0.1),
                        intake.stopIntakeBall(),
                        launcher.ShootBall(),
                        intake.outtakeBall(),
                        intake.intakeBall(),
                        new SleepAction(0.1),
                        intake.stopIntakeBall(),
                        new SleepAction(0.01),
                        launcher.ShootBall(),
                        new SleepAction(0.01)
                )
        ); // first round of shooting the pre-loaded artifacts

        Vector2d intakeOne = new Vector2d(-12.0, 27.0 * flipAuto);
        TrajectoryActionBuilder goToIntakeOne = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(intakeOne, Math.toRadians(90.0) * flipAuto)
                //.turnTo(Math.toRadians(90)*flipAuto)
                .waitSeconds(0.01); //First set of artifacts

        Actions.runBlocking(
                new SequentialAction(
                        goToIntakeOne.build()
                )
        ); //moving to the first position to intake artifacts

        TrajectoryActionBuilder goToIntakeOneCollect = drive.actionBuilder(drive.localizer.getPose())
                .lineToYConstantHeading(43.0 * flipAuto)
                .waitSeconds(0.1); // First set of artifacts intaked

        Actions.runBlocking(
          new SequentialAction((
                  intake.intakeBall()),
                  goToIntakeOneCollect.build()
                  )
        ); // The robot moves forward as it collects the artifacts

        Vector2d shootPositionOne = new Vector2d(-30.6209, 21.5313 * flipAuto);
        TrajectoryActionBuilder goToShootOne = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(shootPositionOne, 129.0 * (Math.PI/180.0 * flipAuto))
                //.turnTo(129.0*Math.PI/180.0*flipAuto)
                .waitSeconds(0.05); // first time moving to the shooting position

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(0.1),
                        goToShootOne.build(),
                        intake.stopIntakeBall(),
                        new SleepAction(0.01),
                        launcher.ShootBall(),
                        //intake.outtakeBall(),
                        intake.intakeBall(),
                        new SleepAction(0.1),
                        intake.stopIntakeBall(),
                        launcher.ShootBall(),
                        intake.outtakeBall(),
                        new SleepAction(0.35),
                        intake.intakeBall(),
                        new SleepAction(0.25),
                        intake.stopIntakeBall(),
                        new SleepAction(0.01),
                        launcher.ShootBall(),
                        new SleepAction(0.01)
                )
        ); // robot shoots the second round of artifacts after intaking

        Vector2d intakeTwo = new Vector2d(12.0, 27.0 * flipAuto);
        TrajectoryActionBuilder goToIntakeTwo = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(intakeTwo, Math.toRadians(90.0) * flipAuto)
                //.turnTo(Math.toRadians(90) * flipAuto)
                .waitSeconds(0.01); //First set of artifacts

        Actions.runBlocking(
                new SequentialAction(
                        goToIntakeTwo.build()
                )
        ); //moving to the first position to intake artifacts

        TrajectoryActionBuilder goToIntakeTwoCollect = drive.actionBuilder(drive.localizer.getPose())
                .lineToYConstantHeading(43.0 * flipAuto)
                .waitSeconds(0.1); // First set of artifacts intaked

        Actions.runBlocking(
                new SequentialAction((
                        intake.intakeBall()),
                        goToIntakeTwoCollect.build()
                )
        ); // The robot moves forward as it collects the artifacts

        Vector2d shootPositionTwo = new Vector2d(-30.6209, 21.5313 * flipAuto);
        TrajectoryActionBuilder goToShootTwo = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(shootPositionTwo, 129.0 * (Math.PI/180.0 * flipAuto))
                //.turnTo(129.0*Math.PI/180.0*flipAuto)
                .waitSeconds(0.05); // first time moving to the shooting position

        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(0.1),
                        goToShootOne.build(),
                        intake.stopIntakeBall(),
                        new SleepAction(0.01),
                        launcher.ShootBall()
                )
        ); // robot shoots the second round of artifacts after intaking

        Vector2d intakeThree = new Vector2d(36.0, 27.0 * flipAuto);
        TrajectoryActionBuilder goToIntakeThree = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(intakeThree, Math.toRadians(90.0) * flipAuto)
                //.turnTo(Math.toRadians(90) * flipAuto)
                .waitSeconds(0.01); //First set of artifacts

        Actions.runBlocking(
                new SequentialAction(
                        goToIntakeThree.build()
                )
        ); //moving to the first position to intake artifacts

        TrajectoryActionBuilder goToIntakeThreeCollect = drive.actionBuilder(drive.localizer.getPose())
                .lineToYConstantHeading(45.0 * flipAuto)
                .waitSeconds(0.1); // First set of artifacts intaked

        Actions.runBlocking(
                new SequentialAction((
                        intake.intakeBall()),
                        goToIntakeThreeCollect.build()
                )
        ); // The robot moves forward as it collects the artifacts

        /*
        TrajectoryActionBuilder goToEnd = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(endingPosition, Math.toRadians(-177.7059))
                .waitSeconds(0.01);

        Actions.runBlocking(
                goToEnd.build()
        );
        */
        Storage.pose = drive.localizer.getPose();
    }
}