package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;

public class Storage {
    public static enum Alliance {
        RED,
        BLUE;
    }
    public static Alliance alliance = Alliance.RED;

    public static enum AutoRan {
        GOAL,
        SMALLTRIANGLE
    }

    public static AutoRan autoRan = AutoRan.GOAL;
}

