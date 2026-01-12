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

        Vector2d shootPosition = new Vector2d(-30.6209, 21.5313);
        TrajectoryActionBuilder goToShoot = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(129.5463))
                .waitSeconds(1.0);

        Vector2d intakeOne = new Vector2d(-12.0, 36.0);
        TrajectoryActionBuilder goToIntakeOne = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(intakeOne, Math.toRadians(90.0))
                .waitSeconds(1.0);

        Vector2d intakeOneCollect = new Vector2d(-12.0, 48.0);
        TrajectoryActionBuilder goToIntakeOneCollect = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(intakeOneCollect, Math.toRadians(90.0))
                .waitSeconds(1.0);

        Vector2d endingPosition = new Vector2d(-61.6131, 11.5718);

        while (!isStopRequested() && !opModeIsActive()) {
            //add anything for during initialization
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
                        new SleepAction(1.0),
                        goToIntakeOne.build()
                )
        ); // first round of shooting the pre-loaded artifacts

        Actions.runBlocking(
          new ParallelAction((
                  intake.intakeBall()),
                  goToIntakeOneCollect.build()
                  )
        ); // The robot collects the artifacts as it moves to the next position

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
                      //  goToIntakeOne.build()
                )
        );

        TrajectoryActionBuilder goToEnd = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(endingPosition, Math.toRadians(-177.7059))
                .waitSeconds(1.0);
        Actions.runBlocking(
                goToEnd.build()
        );
        Storage.pose = drive.localizer.getPose();
    }
}