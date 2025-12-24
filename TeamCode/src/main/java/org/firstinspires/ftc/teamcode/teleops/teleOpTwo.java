/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleops;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.utility.Storage.alliance;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import org.firstinspires.ftc.teamcode.autos.RedGoalAUTORR;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner_essentials.*;
import org.firstinspires.ftc.teamcode.utility.Storage;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;


/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "teleOpTwo", group = "StarterBot")
public class teleOpTwo extends OpMode {
    final double FEED_TIME_SECONDS = 0.60; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double TURN_SPEED = 0.05;

    // here are all the values for the normal shooting velocity
    final double HIGH_LAUNCH = 1650;
    final double LAUNCHER_TARGET_VELOCITY = 1200;
    final double LAUNCHER_MIN_VELOCITY = 1050;

    // here are all the values for indexing
    final double LAUNCHER_INDEX_TARGET_VELOCITY = 615.0;
    final double LAUNCHER_INDEX_MIN_VELOCITY = 600.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    //here are the neutral set values for the goals used in auto locking
    double GOAL_X = -72.0;
    double GOAL_Y = 72.0;

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotor intake = null;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Telemetry.Item patternName;
    Telemetry.Item display;
    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */

    //these are the actions for launching
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    //these are the actions for indexing
    private enum IndexState {
        IDLE_INDEX,
        START_INDEX,
        INDEX_BALL,
        STOP_INDEX

    }

    private LaunchState launchState;
    private IndexState indexState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    public MecanumDrive mecanumDrive;
    double selectedLaunchVelocity = 0;

// here are the values for auto locking and the automatic velocity calcluation
    boolean autoAim = false;
    boolean autoVelocity = false;
    double P_autoAim = 1.0/30.0;
    double targetVelocity = 0.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        indexState = IndexState.IDLE_INDEX;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        mecanumDrive = new MecanumDrive(hardwareMap, Storage.pose);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        launcher.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER run mode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */

        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);

        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        //launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
// this is to switch between the alliances for any reason like running the wrong autonomous program
        if (gamepad1.b) {
            alliance = Storage.Alliance.RED;
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(pattern);
        } else if (gamepad1.x) {
            alliance = Storage.Alliance.BLUE;
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(pattern);
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
// updates the robots position constantly
        mecanumDrive.updatePoseEstimate();

        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        //arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        //mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //This is where we add the slow movement option for any precise movements on the field
        double driveMultiplier = gamepad1.right_bumper ? TURN_SPEED : FULL_SPEED;

//here is where the goal location values are differentiated for the two different alliances
        if(alliance == Storage.Alliance.BLUE) {
            GOAL_Y = -72.0;
        }
        else if(alliance == Storage.Alliance.RED) {
            GOAL_Y = 72.0;
        }

// these are the values and equations to calculate the velocity
        double X_DISTANCE = GOAL_X - mecanumDrive.localizer.getPose().position.x;
        double Y_DISTANCE = GOAL_Y - mecanumDrive.localizer.getPose().position.y;

        double GOAL_DISTANCE = Math.sqrt((X_DISTANCE * X_DISTANCE) + (Y_DISTANCE * Y_DISTANCE));


// the values and equations of the auto lock part of the teleop
        double GoalHeading = (Math.atan2((GOAL_Y - mecanumDrive.localizer.getPose().position.y), GOAL_X - mecanumDrive.localizer.getPose().position.x) * (180.0/Math.PI));
        double AutoAimError = GoalHeading - (mecanumDrive.localizer.getPose().heading.toDouble() * (180.0/Math.PI));
        while (AutoAimError > 180) AutoAimError -= 360;
        while (AutoAimError <= -180) AutoAimError += 360;

        double AutoAimPower = AutoAimError * -P_autoAim;

        if (gamepad1.xWasPressed()) {
            autoAim = !autoAim;
        }

// resets the robot's zero position to however it currently is on the field
        if (gamepad1.bWasPressed()) {
            mecanumDrive.localizer.setPose(new Pose2d(0.0,0.0, 0.0));
        }

        if(gamepad2.y) {
            selectedLaunchVelocity = HIGH_LAUNCH;
        } else if (gamepad2.a) {
            selectedLaunchVelocity = LAUNCHER_TARGET_VELOCITY;
        } else if (gamepad2.xWasPressed()) {
            autoVelocity = !autoVelocity;
        } else if (gamepad2.b) {
            launcher.setVelocity(selectedLaunchVelocity);
        }

        if (autoVelocity) {
        //    targetVelocity = 5.1059 * GOAL_DISTANCE + (905.26);
        //    selectedLaunchVelocity = targetVelocity;
        //    launcher.setVelocity(selectedLaunchVelocity);
        }


        if (!autoAim) {
            mecanumDrive(
                    -gamepad1.left_stick_y * driveMultiplier, gamepad1.left_stick_x * driveMultiplier, gamepad1.right_stick_x * driveMultiplier
            );
        }
        else {
            mecanumDrive(-gamepad1.left_stick_y * driveMultiplier, gamepad1.left_stick_x * driveMultiplier, AutoAimPower);
        }

        // These are the statements that indicate what to do if the robot's position is in a certain part of the field

        if (GOAL_DISTANCE <= 57.7246) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
            blinkinLedDriver.setPattern(pattern);
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE;
            blinkinLedDriver.setPattern(pattern);
        }

        //launch(gamepad2.rightBumperWasPressed());
        index(gamepad2.leftBumperWasPressed());
        intake.setPower(-gamepad2.left_stick_y);
        launcher.setPower(-gamepad2.right_stick_y);

        //telemetry.addData("State", launchState);
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, backLeftPower, frontRightPower, backRightPower);
        //telemetry.addData("motorSpeed", launcher.getVelocity());

        // these are all of the basic values of the robot's position and the goal's heading
        telemetry.addData("X", mecanumDrive.localizer.getPose().position.x);
        telemetry.addData("Y", mecanumDrive.localizer.getPose().position.y);
        telemetry.addData("Heading", mecanumDrive.localizer.getPose().heading.toDouble() * (180.0/Math.PI));
        telemetry.addData("Goal Heading", GoalHeading);

        // this telemetry specifically targets the status of the error and the auto lock
        telemetry.addData("Auto Aim Status", autoAim);
        telemetry.addData("Auto Aim Error", AutoAimError);
        telemetry.addData("Auto Aim Power", AutoAimPower);

        // this is specific to automatically calculating the velocity
        telemetry.addData("Distance from Goal", GOAL_DISTANCE);
        telemetry.addData("Velocity", launcher.getVelocity());
        telemetry.addData("Set Launcher Velocity", selectedLaunchVelocity);

        telemetry.addData("Too Close to the Goal: STROBE_RED", pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        telemetry.addData("Not inside the launch triangle: HOT_PINK", pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        telemetry.addData("", pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);

        telemetry.update();
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    void mecanumDrive(double axial, double lateral, double yaw) {
        double max = 0.0;
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }
/*
    void arcadeDrive(double forward, double rotate) {
        frontLeftPower = forward + rotate;
        backLeftPower = forward + rotate;
        frontRightPower = forward - rotate;
        backRightPower = forward - rotate;

*/
 /*
         * Send calculated power to wheels
         */
         /*
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }
    */


    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
            }
        } // end of launch void

//Here is the indexing part of the TeleOp
    void index(boolean indexRequested) {
        switch (indexState) {
            case IDLE_INDEX:
                if (indexRequested) {
                    indexState = IndexState.START_INDEX;
                }
                break;
            case START_INDEX:
                launcher.setVelocity(LAUNCHER_INDEX_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_INDEX_MIN_VELOCITY) {
                    indexState = IndexState.INDEX_BALL;
                }
                break;
            case INDEX_BALL:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                indexState = IndexState.STOP_INDEX;
                break;
            case STOP_INDEX:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    indexState = IndexState.IDLE_INDEX;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    } // end of indexing void

} // end of the complete program