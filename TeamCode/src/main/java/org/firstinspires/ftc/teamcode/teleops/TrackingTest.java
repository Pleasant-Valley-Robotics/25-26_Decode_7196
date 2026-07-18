package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.FieldVisualizer;

/*
 * Bench-test OpMode for the live field tracking feature. Runs only the drive
 * (for odometry) and the Limelight (for AprilTag botpose), no launcher or
 * intake, so it is safe to run with the robot on a cart or bench.
 *
 * Watch the dashboard field view: BLUE robot = odometry, GREEN robot = AprilTag.
 * Drive with gamepad1 (left stick moves, right stick turns), or just push the
 * robot around; odometry tracks either way.
 *
 * Press Y to reset the odometry pose to the vision pose (when a tag is visible),
 * which snaps the two markers together so drift is easy to watch accumulate.
 */
@TeleOp(name = "TrackingTest", group = "Test")
public class TrackingTest extends OpMode {

    private MecanumDrive mecanumDrive;
    private Camera camera;
    private final FieldVisualizer fieldVisualizer = new FieldVisualizer();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Bench test starts at field center; press Y later to snap to the vision pose.
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

        try {
            camera = new Camera(hardwareMap);
            telemetry.addData("Limelight", "connected");
        } catch (Exception e) {
            camera = null;
            telemetry.addData("Limelight", "NOT FOUND, odometry only: " + e.getMessage());
        }

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        mecanumDrive.updatePoseEstimate();
        Pose2d odometryPose = mecanumDrive.localizer.getPose();

        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                -gamepad1.right_stick_x));

        Pose2d visionPose = null;
        if (camera != null) {
            camera.updateHeading(odometryPose.heading.toDouble());
            visionPose = camera.getRobotPose();
        }

        if (gamepad1.yWasPressed() && visionPose != null) {
            mecanumDrive.localizer.setPose(visionPose);
        }

        fieldVisualizer.update(odometryPose, visionPose, camera);

        telemetry.addData("Odometry", "x %.1f  y %.1f  h %.1f",
                odometryPose.position.x, odometryPose.position.y,
                Math.toDegrees(odometryPose.heading.toDouble()));
        if (visionPose != null) {
            telemetry.addData("Vision", "x %.1f  y %.1f  h %.1f",
                    visionPose.position.x, visionPose.position.y,
                    Math.toDegrees(visionPose.heading.toDouble()));
        } else {
            telemetry.addData("Vision", "no fix (no tags visible or stale)");
        }
        if (camera != null) {
            telemetry.addData("Visible tags", camera.getVisibleTagCount());
        }
        telemetry.addData("Y button", "snap odometry to vision pose");
        telemetry.update();
    }
}
