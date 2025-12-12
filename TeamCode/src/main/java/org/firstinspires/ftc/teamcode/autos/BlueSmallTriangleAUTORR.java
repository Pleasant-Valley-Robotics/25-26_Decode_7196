package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

@Config
@Autonomous(name = "BlueSmallTriangleAUTORR", group = "Autonomous")
public class BlueSmallTriangleAUTORR extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(62.1499, -17.8955, Math.toRadians(-179.1488));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Camera camera = new Camera(hardwareMap);

// This is supposed to go to the coordinates of the shooting distance (-30.6209, 21.5313) with heading 129.5463
//This is the coordinates for the ending position of Goal AUTO (-61.7134, 17.4823) with heading -177.7059
        Vector2d shootPosition = new Vector2d(-30.6209, -21.5313);
        TrajectoryActionBuilder goToShoot = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(-129.5463))
                .waitSeconds(1.0);


        Vector2d endingPosition = new Vector2d(-61.1642, -11.5718);

        //Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
        //        .strafeTo(new Vector2d(48, 12))
        //         .build();


        while (!isStopRequested() && !opModeIsActive()) {
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
                .strafeToLinearHeading(endingPosition, Math.toRadians(-178.1502))
                .waitSeconds(1.0);
        Actions.runBlocking(
                goToEnd.build()
        );
    }
}