package org.firstinspires.ftc.teamcode.subsystems;

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


    public Camera (HardwareMap hardwareMap) {
        initAprilTag(hardwareMap);
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
        VisionPortal visionPortal = builder.build();

        //Disable or re-enable the aprilTag processor at any time
        visionPortal.setProcessorEnabled(aprilTag, true);


    }
}
