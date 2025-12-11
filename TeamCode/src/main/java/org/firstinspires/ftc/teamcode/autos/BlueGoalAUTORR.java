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

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Config
@Autonomous(name = "BlueGoalAUTORR", group = "Autonomous")
public class BlueGoalAUTORR extends LinearOpMode {public class Launcher {
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private ElapsedTime feederTimer = new ElapsedTime();

    public Launcher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300,0,0,10));
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        leftFeeder.setPower(0.0);
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);

        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        rightFeeder.setPower(0.0);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);
    }

    public class ShootBall implements Action {
        private boolean initialized = false;
        private boolean startedShooting = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                startedShooting = false;
                launcher.setVelocity(0.0);
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                feederTimer.reset();
                feederTimer.startTime();
            }
            double vel = launcher.getVelocity();
            packet.put("launcherVelocity", vel);
            if (vel > 1225.0) {
                double tim = feederTimer.seconds();
                packet.put("feederTimer", tim);
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);
                launcher.setVelocity(1250.0);
                startedShooting = true;

            } else if (!startedShooting) {
                launcher.setVelocity(1250.0);
                feederTimer.reset();
                feederTimer.startTime();
            }
            if (feederTimer.seconds() > 0.50) {
                leftFeeder.setPower(0.0);
                rightFeeder.setPower(0.0);
                launcher.setVelocity(0.0);
                return false;
            }
            return true;
        }
    }
    public Action ShootBall()
    {
        return new ShootBall();
    }
}

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-55.135, -49.0834, Math.toRadians(-131.014));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);

// This is supposed to go to the coordinates of the shooting distance (-30.6209, 21.5313) with heading 129.5463
//This is the coordinates for the ending position of Goal AUTO (-61.7134, 17.4823) with heading -177.7059
        Vector2d shootPosition = new Vector2d(-30.6209, -21.5313);
        TrajectoryActionBuilder goToShoot = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(shootPosition, Math.toRadians(-127.5463))
                .waitSeconds(1.0);


        Vector2d endingPosition = new Vector2d(-62.2683, -14.212);

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
                .strafeToLinearHeading(endingPosition, Math.toRadians(178.9887))
                .waitSeconds(1.0);
        Actions.runBlocking(
                goToEnd.build()
        );
    }
}