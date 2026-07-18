package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Config
public class Camera {

    /*
     * These values are live-editable from the FTC Dashboard Config tab, so we can
     * adjust them during testing without recompiling and redeploying.
     *
     * APRILTAG_PIPELINE: which Limelight pipeline to run. Pipeline 0 is the one the
     * team has used for reading obelisk tag IDs; if botpose localization ends up on a
     * different pipeline, change this number from the dashboard and re-init.
     *
     * USE_MEGATAG2: MegaTag2 fuses the robot's known heading (from odometry) with the
     * tag detections for a more stable pose, but it requires updateHeading() to be
     * called every loop. MegaTag1 (false) works standalone.
     *
     * VISION_HEADING_OFFSET_DEGREES: corrects any constant rotation disagreement
     * between the Limelight's field frame and the RoadRunner/odometry field frame.
     */
    public static int APRILTAG_PIPELINE = 0;
    public static boolean USE_MEGATAG2 = false;
    public static double VISION_HEADING_OFFSET_DEGREES = 0.0;
    public static double MAX_STALENESS_MS = 500;

    private final Limelight3A limelight;

    public Camera (HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

    }
    public int findShotsToCycle() {
        int shotsToCycle = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                if (result.getFiducialResults().get(0).getFiducialId() == 22) {
                    shotsToCycle = 2;
                } else if (result.getFiducialResults().get(0).getFiducialId() == 23) {
                    shotsToCycle = 1;
                } // end of assigning each ID the number of artifacts to cycle
            }
        } return  shotsToCycle;
    } // end of find shots to cycle

    /*
     * Feeds the robot's current odometry heading to the Limelight. MegaTag2 needs this
     * every loop to compute its fused pose. Harmless to call when using MegaTag1.
     * headingRadians is the RoadRunner pose heading.
     */
    public void updateHeading(double headingRadians) {
        limelight.updateRobotOrientation(Math.toDegrees(headingRadians));
    }

    /*
     * Returns the robot's field pose as computed by the Limelight from visible
     * AprilTags (botpose), converted to the units and types the rest of the robot
     * code uses: a RoadRunner Pose2d in inches, field-centered origin.
     *
     * Returns null when there is no trustworthy vision fix (no result, invalid
     * result, stale result, or no botpose). Callers must handle null.
     */
    public Pose2d getRobotPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        if (result.getStaleness() > MAX_STALENESS_MS) {
            return null;
        }

        Pose3D botpose = USE_MEGATAG2 ? result.getBotpose_MT2() : result.getBotpose();
        if (botpose == null) {
            return null;
        }

        // The Limelight reports field position in meters; RoadRunner works in inches.
        Position positionInches = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double headingRadians = botpose.getOrientation().getYaw(AngleUnit.RADIANS)
                + Math.toRadians(VISION_HEADING_OFFSET_DEGREES);

        return new Pose2d(positionInches.x, positionInches.y, headingRadians);
    }

    /*
     * How many AprilTags contributed to the current botpose. Zero means the pose
     * is not vision-backed right now. Useful telemetry for students to see how
     * tag visibility affects localization quality.
     */
    public int getVisibleTagCount() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return 0;
        }
        return result.getBotposeTagCount();
    }

    /*
     * Milliseconds since the Limelight result was captured. High values mean the
     * vision pose is lagging behind reality.
     */
    public double getStalenessMs() {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            return -1;
        }
        return result.getStaleness();
    }
} //end of the camera class
