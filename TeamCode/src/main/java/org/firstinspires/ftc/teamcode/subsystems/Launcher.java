package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private ElapsedTime feederTimer = new ElapsedTime();

    public Launcher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(350,0,0,12));
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
            if (vel > 1250.0) {
                double tim = feederTimer.seconds();
                packet.put("feederTimer", tim);
                launcher.setVelocity(1250.0);
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);
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

public class IndexBall implements Action {
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
        if (vel > 600.0) {
            double tim = feederTimer.seconds();
            packet.put("feederTimer", tim);
            startedShooting = true;

        } else if (!startedShooting) {
            launcher.setVelocity(600.0);
            feederTimer.reset();
            feederTimer.startTime();
        }
        if (feederTimer.seconds() > 0.15){
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(1.0);
        }
        if (feederTimer.seconds() > 0.55) {
            leftFeeder.setPower(0.0);
            rightFeeder.setPower(0.0);
            launcher.setVelocity(0.0);
            return false;
        }
        return true;
    }
}// end of index ball class
public Action IndexBall()
{
    return new IndexBall();
}// end of index ball action

} //end of launcher class
