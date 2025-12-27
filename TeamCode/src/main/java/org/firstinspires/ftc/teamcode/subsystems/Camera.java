package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Camera {
    private AprilTagProcessor aprilTag;
    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
    private Limelight3A limelight;


    public Camera (HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
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
                } // end of april tag 23 code
            }
        } return  shotsToCycle;
    } // end of find shots to cycle
} //end of the camera class
