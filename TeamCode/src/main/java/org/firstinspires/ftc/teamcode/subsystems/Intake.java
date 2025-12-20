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

public class Intake {
    private DcMotorEx intake;

    private ElapsedTime intakeTimer = new ElapsedTime();

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

    public class intakeBall implements Action {
        private boolean initialized = false;
        private boolean startedIntake = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                startedIntake = false;
                intake.setVelocity(0.0);
                intakeTimer.reset();
                intakeTimer.startTime();
            }
            double vel = intake.getVelocity();
            packet.put("intakeVelocity", vel);
            if (vel > 1250.0) {
                startedIntake = true;
                double tim = intakeTimer.seconds();
                packet.put("intakeTimer", tim);
                intake.setVelocity(1250.0);

            } else if (!startedIntake) {
                intake.setVelocity(1250.0);
                intakeTimer.reset();
                intakeTimer.startTime();
            }
            if (intakeTimer.seconds() > 0.50) {
                intake.setVelocity(0.0);
                return false;
            }
            return true;
        }
    } // end of intakeBall action

    public Action intakeBall()
    {
        return new Intake.intakeBall();
    }
} // end of Intake class
