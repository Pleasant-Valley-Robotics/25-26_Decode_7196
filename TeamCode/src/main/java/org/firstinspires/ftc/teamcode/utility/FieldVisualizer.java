package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner_essentials.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Camera;

/*
 * Draws live robot tracking onto the FTC Dashboard field view
 * (http://192.168.43.1:8080/dash from the robot's WiFi).
 *
 * Two robot markers are drawn each loop:
 *   BLUE  = where odometry (Pinpoint) thinks the robot is
 *   GREEN = where the Limelight AprilTag botpose thinks the robot is
 * A gray line connects the two so drift between them is visible at a glance.
 *
 * Call update() once per OpMode loop. The vision pose may be null (no tags
 * visible, stale data); the odometry marker still draws.
 */
@Config
public class FieldVisualizer {

    public static boolean SHOW_ODOMETRY = true;
    public static boolean SHOW_VISION = true;
    public static String ODOMETRY_COLOR = "#3F51B5";
    public static String VISION_COLOR = "#4CAF50";
    public static String DRIFT_LINE_COLOR = "#9E9E9E";

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void update(Pose2d odometryPose, Pose2d visionPose, Camera camera) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        if (SHOW_ODOMETRY && odometryPose != null) {
            field.setStroke(ODOMETRY_COLOR);
            Drawing.drawRobot(field, odometryPose);

            packet.put("tracking/odometry x (in)", odometryPose.position.x);
            packet.put("tracking/odometry y (in)", odometryPose.position.y);
            packet.put("tracking/odometry heading (deg)", Math.toDegrees(odometryPose.heading.toDouble()));
        }

        if (SHOW_VISION && visionPose != null) {
            field.setStroke(VISION_COLOR);
            Drawing.drawRobot(field, visionPose);

            packet.put("tracking/vision x (in)", visionPose.position.x);
            packet.put("tracking/vision y (in)", visionPose.position.y);
            packet.put("tracking/vision heading (deg)", Math.toDegrees(visionPose.heading.toDouble()));
        }

        if (SHOW_ODOMETRY && SHOW_VISION && odometryPose != null && visionPose != null) {
            field.setStroke(DRIFT_LINE_COLOR);
            field.strokeLine(
                    odometryPose.position.x, odometryPose.position.y,
                    visionPose.position.x, visionPose.position.y);

            double driftInches = odometryPose.position.minus(visionPose.position).norm();
            packet.put("tracking/odometry vs vision drift (in)", driftInches);
        }

        if (camera != null) {
            packet.put("tracking/visible tags", camera.getVisibleTagCount());
            packet.put("tracking/vision staleness (ms)", camera.getStalenessMs());
            packet.put("tracking/vision has fix", visionPose != null);
        }

        dashboard.sendTelemetryPacket(packet);
    }
}
