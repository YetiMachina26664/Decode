package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@Autonomous(name="Comp Auto")
public class OdoAuto extends OpMode {

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Follower follower;
    private TelemetryManager telemetryM;

    private RevColorSensorV3 color;

    private DcMotor belt = null;
    private DcMotorEx lFlywheel = null;
    private DcMotorEx rFlywheel = null;
    private DcMotor intake = null;



    //Initial flywheel percentages for far and close shots respectively
    double liftoffPoly;
    double liftoffLin;

    //Intake and Belt initial speeds
    double intakeSpeed = 0;
    double beltSpeed = 0;

    public static final double MAX_TICKS_PER_SECOND = 5376;
    public static final double MAX_TICKS_BELT = 88;
    public Pose REDGOAL = new Pose(144, 0);
    public Pose BLUEGOAL = new Pose(144, 144);

    public Pose tempPose = new Pose(0,0,0);

    double distFromGoal;
    double tgtTheta;
    double hypotenuse;

    double XTemp;
    double YTemp;
    boolean isAtPose = false;

    boolean isBlueTeam;
    boolean backStart;

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
            tgtY = Math.abs(BLUEGOAL.getY() - follower.getPose().getY());
            tgtX = Math.abs(BLUEGOAL.getX() - follower.getPose().getX());
            return Math.abs(((Math.PI)/2) - Math.atan2(tgtX, tgtY));
        } else {
            tgtY = Math.abs(REDGOAL.getY() - follower.getPose().getY());
            tgtX = Math.abs(REDGOAL.getX() - follower.getPose().getX());
            return -Math.abs(((Math.PI)/2) - Math.atan2(tgtX, tgtY));
        }
    }

    //calculation for the straightline distance between the robot and the goal
    public double calcHypotenuse(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public double powerRegressionPoly(double x) {
        return (0.0014 * Math.pow(x, 2)) + (2.5179 * x) + 788.18;
    }

    public double powerRegressionLin(double x) {
        return (3.0058 * x) + 756.46;
    }

    public Pose getStartingPose(NormalizedRGBA colors) {
        Pose pose = new Pose();

        int red   = (int) (colors.red * 255);
        int green = (int) (colors.green * 255);
        int blue  = (int) (colors.blue * 255);

        if ((red > 155 && red < 175)
                && (green > 72 && green < 92)
                && (blue > 39 && blue < 59)) {
            isBlueTeam = false;
            backStart = false;
            pose = new Pose(9, 56, 0);
            return pose;
            //Cyan
        } else if ((red > 35 && red < 55)
                && (green > 124 && green < 144)
                && (blue > 244 && blue <= 255)) {
            isBlueTeam = true;
            backStart = false;
            pose = new Pose(9, 88, 0);
            return pose;
            //Ivory
        } else if ((red > 245 && red <= 255)
                && (green > 245 && green <= 255)
                && (blue > 245 && blue <= 255)) {
            isBlueTeam = false;
            backStart = true;
            pose = new Pose(124, 122, -0.925025);
            return pose;
            //Wood (Chat)
        } else if ((red > 88 && red < 108)
                && (green > 140 && green < 160)
                && (blue > 81 && blue < 101)) {
            isBlueTeam = true;
            backStart = true;
            pose = new Pose(124, 22, 0.925025);
            return pose;
        }
        return pose;
    }

    public PathChain getPathChain() {
        PathChain pathChain;
        Pose startPose;
        if (isBlueTeam && backStart) {
            startPose = new Pose(124, 22, 0.925025);
        } else if (isBlueTeam && !backStart) {
            startPose = new Pose(9, 88, 0);
        } else if (!isBlueTeam && backStart) {
            startPose = new Pose(124, 122, -0.925025);
        } else {
            startPose = new Pose(9, 56, 0);
        }
        return pathChain = follower.pathBuilder()
                .addPath(startPose, )
    };


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        color = hardwareMap.get(RevColorSensorV3.class, "color");
        /*NormalizedColorSensor normalizedSensor = color;
        normalizedSensor.setGain(40);
        NormalizedRGBA colors = normalizedSensor.getNormalizedColors();

        follower.setStartingPose(getStartingPose(colors));*/
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //pathChain = () -> follower.pathBuilder()
        //        .addPath(new Path(new BezierLine()))

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
    }

    public void init_loop() {
        NormalizedColorSensor normalizedSensor = color;
        normalizedSensor.setGain(40);
        NormalizedRGBA colors = normalizedSensor.getNormalizedColors();
        while (follower.getPose() == null || follower.getPose().getX() < 1) {
            follower.setStartingPose(getStartingPose(colors));
            follower.update();
        }

        follower.update();

        telemetry.addData("Red", colors.red * 255);
        telemetry.addData("Green", colors.green * 255);
        telemetry.addData("Blue", colors.blue * 255);
        telemetry.addData("Starting Pose", follower.getPose());
        telemetry.update();
    }


    @Override
    public void start() {
        setPathState(0);
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


        //tgtTheta = getTargetAngle(isBlueTeam, follower);
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
        } else if (aPressed && rFlywheel.getVelocity() > 0.0) {
            beltSpeed = -0.5;
        } else if (aPressed) {
            beltSpeed = -0.95;
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
        /* if (gamepad1.b && liftoffLow >= 0.1 && adjustSpeed) {
            liftoffHigh -= 0.001;
            liftoffLow -= 0.001;
            adjustSpeed = false;
        }
        // Check if speed was adjusted.
        if (gamepad1.bWasReleased()) {
            adjustSpeed = true;
        }
        if (gamepad1.y & liftoffHigh <= 0.4 && adjustSpeed) {
            liftoffHigh += 0.001;
            liftoffLow += 0.001;
            adjustSpeed = false;
        }
        // Check if speed was adjusted.
        if (gamepad1.yWasReleased()) {
            adjustSpeed = true;
        }

        // Check if speed was adjusted.
        if (gamepad1.yWasReleased()) {
            adjustSpeed = true;
        }*/

        distFromGoal = getDist(isBlueTeam, follower);

        liftoffPoly = powerRegressionPoly(distFromGoal);
        liftoffLin = powerRegressionLin(distFromGoal);


        if (gamepad1.dpad_left && !follower.isBusy()) {
            tgtTheta = getTargetAngle(isBlueTeam, follower);
            follower.turnTo(tgtTheta);
            //follower.holdPoint(new Pose(follower.getPose().getX(), follower.getPose().getY(), tgtTheta));
            isAtPose = follower.atPose(tempPose,1,1,.01);
        }

        if (gamepad1.dpad_right) {
            follower.startTeleopDrive();
        }

        // Set Intake power percentage
        intake.setPower(intakeSpeed);
        // Set belt power percentage
        belt.setPower(beltSpeed);

        //liftoff = powerRegression(getDist(isBlueTeam, follower.getPose()));
        // Set flywheel velocities (LB = High power shot, RB = Low power shot)
        if (gamepad1.left_bumper) {
            lFlywheel.setVelocity(liftoffPoly);
            rFlywheel.setVelocity(liftoffPoly);
        } else if (gamepad1.right_bumper) {
            lFlywheel.setVelocity(liftoffLin);
            rFlywheel.setVelocity(liftoffLin);
        } else if (dPadUpToggle) { // Make sure that reverse motion can be toggled so we don't get fouled :/
            lFlywheel.setVelocity(MAX_TICKS_PER_SECOND * -0.01);
            rFlywheel.setVelocity(MAX_TICKS_PER_SECOND * -0.01);
        } else { // Set 0 by default.
            lFlywheel.setVelocity(0);
            rFlywheel.setVelocity(0);
        }

        telemetry.addData("Flywheel Target TPS LIN/POLY", "%4.2f, %4.2f", liftoffLin, liftoffPoly);
        telemetry.addData("Flywheel TPS Left/Right", "%4.2f, %4.2f", rFlywheel.getVelocity(), lFlywheel.getVelocity());
        telemetry.addData("Belt", "%4.2f", beltSpeed);
        telemetry.addData("Intake", "%4.2f", intakeSpeed);
        telemetry.addData("Heading", "%.2f degrees", follower.getHeading()*57.296);
        telemetry.addData("PedroPose", "X: %.2f in, \nY: %.2f in, \nHeading %.2f degrees", follower.getPose().getX(), follower.getPose().getY(),Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Auto-Aim Active", follower.isBusy() ? "Yes" : "No");
        telemetry.addData("Target Heading","%4.2f degrees", tgtTheta*57.296);
        telemetry.addData("Target Distance","%4.2f in", distFromGoal);
        telemetry.addData("GOAL POS","X: %4.2f in, Y: %4.2f in", REDGOAL.getX(), REDGOAL.getY());
        telemetry.addData("isBusy",follower.isBusy());
        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("L Current Velocity", lFlywheel.getVelocity());
        telemetryM.debug("R Current Velocity", rFlywheel.getVelocity());
        telemetryM.debug("Target V % Polynomial", liftoffPoly);
        telemetryM.debug("Target V % Linear", liftoffLin);
        telemetryM.debug("belt", beltSpeed);
        telemetryM.debug("intake", intakeSpeed);
        telemetryM.debug("adjustSpeed", adjustSpeed);
        telemetryM.debug("isAtPose", isAtPose);
    }
}
