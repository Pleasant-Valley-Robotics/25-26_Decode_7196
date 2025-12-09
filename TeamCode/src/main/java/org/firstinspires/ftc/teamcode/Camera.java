package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import androidx.annotation.NonNull;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    final double FEED_TIME = 0.5;
    final double BACK_TIME = 0.5;
    private int shotsToIndex = 2;
    double targetSpeed = 400.0;


    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    ElapsedTime feederTimer = new ElapsedTime();

    private AprilTagProcessor aprilTag;
    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera

    private VisionPortal visionPortal;


    public void Index (HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);

        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFeeder = hardwareMap.get(CRServo.class,"leftFeeder");
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);
        leftFeeder.setPower(0.0);

        rightFeeder = hardwareMap.get(CRServo.class,"leftFeeder");
        rightFeeder.setDirection(DcMotor.Direction.FORWARD);
        rightFeeder.setPower(0.0);
    }

    public Action spinUp (double targetSpeed) {
        return new Action(){
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher.setVelocity(targetSpeed);
                    initialized = true;
                }

                double vel = launcher.getVelocity();
                packet.put("Shooter Velocity", vel);
                return vel<targetSpeed;
            }
        };
    }

    public int findShotsToCycle() {
        int shotsToCycle = 0;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (!currentDetections.isEmpty()) {
            for (int i = 0; i < currentDetections.size(); i++) {
                if (currentDetections.get(i).id == 22) {
                    shotsToCycle = 2;
                    break;
                } else if (currentDetections.get(i).id == 23) {
                    shotsToCycle = 1;
                }
            }
        } return  shotsToCycle;
    }

    public Action StopIndex() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    launcher.setVelocity(0.0);
                    initialized = true;
                }

                double vel = launcher.getVelocity();
                packet.put("Shooter Shooter", vel);
                return vel !=0.0;
            }
        };

    }

    public Action FireBall() {
        return new Action() {
            private boolean movingForward = false;

            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                if (!movingForward) {
                    movingForward = true;

                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);

                    feederTimer.reset();

                    return true;
                }

                if (feederTimer.seconds() > BACK_TIME + FEED_TIME) {
                    leftFeeder.setPower(0.0);
                    rightFeeder.setPower(0.0);

                    return false;
                } else if (feederTimer.seconds() > BACK_TIME) {
                    leftFeeder.setPower(-1.0);
                    rightFeeder.setPower(-1.0);
                    return true;
                }

                return true;
            }
        };
    }

    private void initAprilTag(HardwareMap hardwareMap) {

        //Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        //Crete the vision portal by using a builder
        VisionPortal.Builder builder = new VisionPortal.Builder();

        //
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Set and enable the processor
        builder.addProcessor(aprilTag);

        //Build the Vision Portal, using the above settings
        visionPortal = builder.build();

        //Disable or re-enable the aprilTag processor at any time
        //vision.Portal.setProcessorEnabled(aprilTag, true);


    }
}
