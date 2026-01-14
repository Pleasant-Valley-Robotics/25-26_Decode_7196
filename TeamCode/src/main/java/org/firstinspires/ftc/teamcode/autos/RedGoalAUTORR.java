package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.utility.Storage;

@Config
@Autonomous(name = "RedGoalAUTORR", group = "Autonomous")
public class RedGoalAUTORR extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-55.135, 49.0834, Math.toRadians(131.014));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        Vector2d indexPosition = new Vector2d(-22.2411, 6.7882);
        TrajectoryActionBuilder goToIndex = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(indexPosition, Math.toRadians(-150.2229))
                .waitSeconds(0.25);

        Vector2d endingPosition = new Vector2d(-61.6131, 11.5718);

        while (!isStopRequested() && !opModeIsActive()) {
            //add anything for during initialization
            telemetry.addData("Shots To Cycle", camera.findShotsToCycle());
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;

/*            Actions.runBlocking(
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

        Vector2d shootPositionOne = new Vector2d(-30.6209, 21.5313);
        TrajectoryActionBuilder goToShootOne = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(shootPositionOne, Math.toRadians(129.5463))
                .waitSeconds(0.25);

        Actions.runBlocking(
                new SequentialAction(
                        goToShootOne.build(),
                        new SleepAction(0.15),
                        launcher.ShootBall(),
                        new SleepAction(0.15),
                        launcher.ShootBall(),
                        new SleepAction(0.15),
                        launcher.ShootBall(),
                        new SleepAction(0.15)
                )
        ); // first round of shooting the pre-loaded artifacts

        Vector2d intakeOne = new Vector2d(-12.0, 35.0);
        TrajectoryActionBuilder goToIntakeOne = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(intakeOne, Math.toRadians(90.0))
                .waitSeconds(0.25);

        Actions.runBlocking(
                new SequentialAction(
                        goToIntakeOne.build()
                )
        ); //moving to the first position to intake artifacts

        Vector2d intakeOneCollect = new Vector2d(-12.0, 55.0);
        TrajectoryActionBuilder goToIntakeOneCollect = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(intakeOneCollect, Math.toRadians(90.0))
                .lineToYConstantHeading(53.0)
                .waitSeconds(1.0);

        Actions.runBlocking(
          new SequentialAction((
                  intake.intakeBall()),
                  goToIntakeOneCollect.build()
                  )
        ); // The robot moves forward as it collects the artifacts

        Actions.runBlocking(
                new SequentialAction(
                        goToShootOne.build(),
                        intake.stopIntakeBall(),
                        new SleepAction(0.15),
                        launcher.ShootBall(),
                        intake.outtakeBall(),
                        intake.intakeBall(),
                        new SleepAction(0.5),
                        launcher.ShootBall(),
                        intake.outtakeBall()
                )
        );// robot shoots the second round of artifacts after intaking

        Vector2d intakeTwo = new Vector2d(12.0, 33.0);
        TrajectoryActionBuilder goToIntakeTwo = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(intakeTwo, Math.toRadians(90.0))
                .waitSeconds(0.25);

        Actions.runBlocking(
                new SequentialAction(
                        goToIntakeTwo.build()
                )
        );

        Vector2d intakeTwoCollect = new Vector2d(12.0, 70.0);
        TrajectoryActionBuilder goToIntakeTwoCollect = drive.actionBuilder(drive.localizer.getPose())
                //.strafeToLinearHeading(intakeOneCollect, Math.toRadians(90.0))
                .lineToYConstantHeading(53.0)
                .waitSeconds(0.25);

        Actions.runBlocking(
                new SequentialAction((
                        intake.intakeBall()),
                        goToIntakeTwoCollect.build()
                )
        ); // The robot moves forward as it collects the artifacts

        Vector2d shootPositionTwo = new Vector2d(-30.6209, 21.5313);
        TrajectoryActionBuilder goToShootTwo = drive.actionBuilder(drive.localizer.getPose())
                .lineToYConstantHeading(33.0)
                .strafeToLinearHeading(shootPositionTwo, Math.toRadians(129.5463))
                .waitSeconds(0.25);

        Actions.runBlocking(
                new SequentialAction(
                        goToShootTwo.build(),
                        intake.stopIntakeBall(),
                        new SleepAction(0.15),
                        launcher.ShootBall(),
                        intake.outtakeBall(),
                        intake.intakeBall(),
                        new SleepAction(0.5),
                        launcher.ShootBall(),
                        intake.outtakeBall()

                )
        );// robot shoots the second round of artifacts after intaking

        TrajectoryActionBuilder goToEnd = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(endingPosition, Math.toRadians(-177.7059))
                .waitSeconds(0.25);
        Actions.runBlocking(
                goToEnd.build()
        );
        Storage.pose = drive.localizer.getPose();
    }
}