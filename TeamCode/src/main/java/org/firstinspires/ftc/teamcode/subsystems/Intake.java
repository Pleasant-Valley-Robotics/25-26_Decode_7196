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
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }

    public class intakeBall implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
   /*         if (!initialized) {
                initialized = true;
                intake.setPower(0.0);
                intakeTimer.reset();
                intakeTimer.startTime();
            }

            double tim = intakeTimer.seconds();
            packet.put("intakeTimer", tim);
            intake.setPower(1.0);
            if (tim > 20.0) {
                intake.setPower(0.0);
                return false;
            }*/
            intake.setPower(1.0);
            return false;
        }
    } // end of intakeBall action

    public Action intakeBall()
    {
        return new Intake.intakeBall();
    }

    public class outtakeBall implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                intake.setPower(0.0);
                intakeTimer.reset();
                intakeTimer.startTime();
            }

            double tim = intakeTimer.seconds();
            packet.put("intakeTimer", tim);
            intake.setPower(-1.0);
            if (tim > 0.1) {
                intake.setPower(0.0);
                return false;
            }
            return true;
        }
    } // end of intakeBall action

    public Action outtakeBall()
    {
        return new Intake.outtakeBall();
    }

    public class stopIntakeBall implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
   /*         if (!initialized) {
                initialized = true;
                intake.setPower(0.0);
                intakeTimer.reset();
                intakeTimer.startTime();
            }

            double tim = intakeTimer.seconds();
            packet.put("intakeTimer", tim);
            intake.setPower(1.0);
            if (tim > 20.0) {
                intake.setPower(0.0);
                return false;
            }*/
            intake.setPower(0.0);
            return false;
        }
    } // end of intakeBall action

    public Action stopIntakeBall()
    {
        return new Intake.stopIntakeBall();
    }

} // end of Intake class
