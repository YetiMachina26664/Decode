package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name="Odometry Driver", group="Linear OpMode")
public class OdoDriver extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    private DcMotor belt = null;
    private DcMotorEx lFlywheel = null;
    private DcMotorEx rFlywheel = null;
    private DcMotor intake = null;

    boolean aPressed;       // Tracks if A is toggled ON or OFF
    boolean previousAState; // Tracks the button state from the previous loop cycle

    boolean yPressed;       // Tracks if Y is toggled ON or OFF
    boolean previousYState; // Tracks the button state from the previous loop cycle

    boolean xPressed;       // Tracks if X is toggled ON or OFF
    boolean previousXState; // Tracks the button state from the previous loop cycle

    boolean bPressed;       // Tracks if B is toggled ON or OFF
    boolean previousBState; // Tracks the button state from the previous loop cycle

    boolean dPadDownToggle;         //Tracks if D-Pad Down is toggled ON or OFF
    boolean previousDPadDownState;  //Tracks the button state from the previous loop cycle

    boolean dPadUpToggle;         //Tracks if D-Pad Up is toggled ON or OFF
    boolean previousDPadUpState;  //Tracks the button state from the previous loop cycle

    boolean dPadULeftToggle;         //Tracks if D-Pad Up is toggled ON or OFF
    boolean previousDPadLeftState;  //Tracks the button state from the previous loop cycle

    boolean adjustSpeed;         //Checks whether or not speed was adjusted

    //Initial flywheel percentages for far and close shots respectively
    double liftoffHigh = 0.22;
    double liftoffLow = 0.19;

    //Intake and Belt initial speeds
    double intakeSpeed = 0;
    double beltSpeed = 0;

    public static final double MAX_TICKS_PER_SECOND = 5376;

    public Pose REDGOAL = new Pose(134, 135);
    public Pose BLUEGOAL = new Pose(9, 135);

    boolean isBlueTeam = false;

    double distFromGoal;
    double tgtTheta;
    double hypotenuse;

    //function to get the distance from the robot to the goal, relies on calcHypotenuse function
    public double getDist(boolean blueTeam, Follower follower) {
        if (blueTeam) {
            return calcHypotenuse(
                    BLUEGOAL.getX() - follower.getPose().getX(),
                    BLUEGOAL.getY() - follower.getPose().getY());
        } else {
            return calcHypotenuse(
                    REDGOAL.getX() - follower.getPose().getX(),
                    REDGOAL.getY() - follower.getPose().getY());
        }
    }

    //get the target heading for the robot to the goal
    public double getTargetAngle(boolean blueTeam, Follower follower) {
        double tgtY, tgtX;
        if (blueTeam) {
            tgtY = BLUEGOAL.getY() - follower.getPose().getY();
            tgtX = BLUEGOAL.getX()- follower.getPose().getX();
        } else {
            tgtY = REDGOAL.getY() - follower.getPose().getY();
            tgtX = REDGOAL.getX() - follower.getPose().getX();
        }
        return Math.atan2(tgtY, tgtX);
    }

    //calculation for the straightline distance between the robot and the goal
    public double calcHypotenuse(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //Hardware map declarations
        belt = hardwareMap.get(DcMotor.class, "belt");
        lFlywheel = hardwareMap.get(DcMotorEx.class, "leftFly");
        rFlywheel = hardwareMap.get(DcMotorEx.class, "rightFly");
        intake = hardwareMap.get(DcMotor.class, "intake");

        //Zero power behaviors for Flywheels
        lFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Flywheel modes for velocity running instead of power percentage
        lFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reverse left flywheel to oppose right
        lFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        aPressed = false;       // Tracks if A is toggled ON or OFF
        previousAState = false; // Tracks the button state from the previous loop cycle

        yPressed = false;       // Tracks if Y is toggled ON or OFF
        previousYState = false; // Tracks the button state from the previous loop cycle

        xPressed = false;       // Tracks if X is toggled ON or OFF
        previousXState = false; // Tracks the button state from the previous loop cycle

        bPressed = false;       // Tracks if B is toggled ON or OFF
        previousBState = false; // Tracks the button state from the previous loop cycle

        dPadDownToggle = false;         //Tracks if D-Pad Down is toggled ON or OFF
        previousDPadDownState = false;  //Tracks the button state from the previous loop cycle

        dPadUpToggle = false;         //Tracks if D-Pad Up is toggled ON or OFF
        previousDPadUpState = false;  //Tracks the button state from the previous loop cycle

        dPadULeftToggle = false;         //Tracks if D-Pad Up is toggled ON or OFF
        previousDPadLeftState = false;  //Tracks the button state from the previous loop cycle

        adjustSpeed = true;         //Checks whether or not speed was adjusted
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );


        tgtTheta = getTargetAngle(isBlueTeam, follower);
        distFromGoal = getDist(isBlueTeam, follower);
        hypotenuse = calcHypotenuse(follower.getPose().getX(), follower.getPose().getY());

        // --- TOGGLE LOGIC ---
        // This detects a single button press event for each button.
        if (gamepad1.a && !previousAState) { aPressed = !aPressed; }
        previousAState = gamepad1.a;

        if (gamepad1.y && !previousYState) { yPressed = !yPressed; }
        previousYState = gamepad1.y;

        if (gamepad1.x && !previousXState) { xPressed = !xPressed; }
        previousXState = gamepad1.x;

        if (gamepad1.b && !previousBState) { bPressed = !bPressed; }
        previousBState = gamepad1.b;

        if (gamepad1.dpad_down && !previousDPadDownState) { dPadDownToggle = !dPadDownToggle; }
        previousDPadDownState = gamepad1.dpad_down;

        if (gamepad1.dpad_up && !previousDPadUpState) { dPadUpToggle = !dPadUpToggle; }
        previousDPadUpState = gamepad1.dpad_up;

        if (gamepad1.dpad_left && !previousDPadLeftState) { dPadULeftToggle = !dPadULeftToggle; }
        previousDPadLeftState = gamepad1.dpad_left;

        // --- ACTION LOGIC ---
        //Moved from outside of the While OpMode is active, do we need a while loop? Can we use
        //a pedropath to get in position without changing how we handle the wheels
        //use the orient function??

        // Set Belt speed. X (reverse) overrides A (forward).
        //Do we want an encoder for the belt motor so that we have a constant speed, not subject to batter
        //battery power control can impact speed that the balls hit the flywheels
        if (xPressed) {
            beltSpeed = 0.5;
        } else if (aPressed) {
            beltSpeed = -1.0;
        } else if (aPressed && rFlywheel.getPower() > 0.0) {
            beltSpeed = -0.8; //we want this give flywheels time to get back to speed
        } else {
            beltSpeed = 0.0;
        }

        // Set intake speed.
        if (dPadDownToggle) {
            intakeSpeed = 1.0;
        } else {
            intakeSpeed = 0.0;
        }

        // Increment and Decrement Flywheel Speed
        // max speed is 40% of max rotation, min is 10% of min rotation.
        if (gamepad1.b && liftoffLow >= 0.1 && adjustSpeed) {
            liftoffHigh -= 0.01;
            liftoffLow -= 0.01;
            adjustSpeed = false;
        }
        // Check if speed was adjusted.
        if (gamepad1.bWasReleased()) {
            adjustSpeed = true;
        }
        if (gamepad1.y & liftoffHigh <= 0.4 && adjustSpeed) {
            liftoffHigh += 0.01;
            liftoffLow += 0.01;
            adjustSpeed = false;
        }
        // Check if speed was adjusted.
        if (gamepad1.yWasReleased()) {
            adjustSpeed = true;
        }

        if (gamepad1.dpad_left && !follower.isBusy()) {
            follower.turnTo(tgtTheta);
        }

        // Set Intake power percentage
        intake.setPower(intakeSpeed);
        // Set belt power percentage
        belt.setPower(beltSpeed);

        //liftoff = powerRegression(getDist(isBlueTeam, follower.getPose()));
        // Set flywheel velocities (LB = High power shot, RB = Low power shot)
        if (gamepad1.left_bumper) {
            lFlywheel.setVelocity(MAX_TICKS_PER_SECOND * liftoffHigh);
            rFlywheel.setVelocity(MAX_TICKS_PER_SECOND * liftoffHigh);
        } else if (gamepad1.right_bumper) {
            lFlywheel.setVelocity(MAX_TICKS_PER_SECOND * liftoffLow);
            rFlywheel.setVelocity(MAX_TICKS_PER_SECOND * liftoffLow);
        } else if (dPadUpToggle) { // Make sure that reverse motion can be toggled so we don't get fouled :/
            lFlywheel.setVelocity(MAX_TICKS_PER_SECOND * -0.01);
            rFlywheel.setVelocity(MAX_TICKS_PER_SECOND * -0.01);
        } else { // Set 0 by default.
            lFlywheel.setVelocity(0);
            rFlywheel.setVelocity(0);
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("L Current Velocity", lFlywheel.getVelocity());
        telemetryM.debug("R Current Velocity", rFlywheel.getVelocity());
        telemetryM.debug("Target V % High", liftoffHigh);
        telemetryM.debug("Target V % Low", liftoffLow);
        telemetryM.debug("belt", beltSpeed);
        telemetryM.debug("intake", intakeSpeed);
        telemetryM.debug("adjustSpeed", adjustSpeed);
    }
}
