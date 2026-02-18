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
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name="Odometry Driver", group="Linear OpMode")
public class OdoDriver extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    private TelemetryManager telemetryM;

    private RevColorSensorV3 color;

    private DcMotorEx belt = null;
    private DcMotorEx lFlywheel = null;
    private DcMotorEx rFlywheel = null;
    private DcMotor intake = null;

    private Servo ballz = null;

    boolean aPressed;       // Tracks if A is toggled ON or OFF
    boolean previousAState; // Tracks the button state from the previous loop cycle

    boolean yPressed;       // Tracks if Y is toggled ON or OFF
    boolean previousYState; // Tracks the button state from the previous loop cycle

    boolean xPressed = true;       // Tracks if X is toggled ON or OFF, start with up position
    boolean previousXState; // Tracks the button state from the previous loop cycle

    boolean bPressed;       // Tracks if B is toggled ON or OFF
    boolean previousBState; // Tracks the button state from the previous loop cycle

    boolean dPadDownToggle;         //Tracks if D-Pad Down is toggled ON or OFF
    boolean previousDPadDownState;  //Tracks the button state from the previous loop cycle

    boolean dPadUpToggle;         //Tracks if D-Pad Up is toggled ON or OFF
    boolean previousDPadUpState;  //Tracks the button state from the previous loop cycle

    boolean dPadLeftToggle;         //Tracks if D-Pad Up is toggled ON or OFF
    boolean previousDPadLeftState;  //Tracks the button state from the previous loop cycle

    boolean previousDPadRightState;  //Tracks the button state from the previous loop cycle
    boolean dPadRightToggle;         //Tracks if D-Pad Up is toggled ON or OFF
    boolean greenLight;              //flywheel speeds meet requirements to shoot

    boolean adjustSpeed;         //Checks whether or not speed was adjusted

    //Initial flywheel percentages for far and close shots respectively
    double liftoffPoly;
    double liftoffLin;

    //Intake and Belt initial speeds
    double intakeSpeed = 0.0;
    double beltSpeed = 0.0;

    double ballzPos = 0.0;

    public static final double MAX_TICKS_PER_SECOND = 2840;
    public static final double MAX_TICKS_BELT = 2840;
    public Pose REDGOAL = new Pose(144, 0);
    public Pose BLUEGOAL = new Pose(144, 144);

    public Pose tempPose = new Pose(0,0,0);

    double distFromGoal;
    double tgtTheta;
    double hypotenuse;
    double negativeTol;
    double positiveTol;

    boolean isAtPose = false;

    boolean isBlueTeam;

    double adjustedSpeed = 0.0;

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
        if (x < 60) {
            return 1000;
        } else if (x < 140) {
            //return (int) (Math.round((1227 - 14.4 * x + 0.194 * Math.pow(x, 2) - .0007 * Math.pow(x, 3))/ 20) * 20);
            return (int) (Math.round((380 + 10.2 * x )/ 20) * 20);
         } else {
            return 1840;
        }
        //return (0.0014 * Math.pow(x, 2)) + (2.5179 * x) + 788.18;
    }

    public double powerRegressionLin(double x) {
        return (3.0058 * x) + 756.46;
    }

    public Pose getStartingPose(NormalizedRGBA colors) {
        Pose pose = new Pose();

        int red   = (int) (colors.red * 255);
        int green = (int) (colors.green * 255);
        int blue  = (int) (colors.blue * 255);

        //Scarlet
        if ((red > 155 && red < 175)
                && (green > 72 && green < 92)
                && (blue > 39 && blue < 59)) {
            isBlueTeam = false;
            pose = new Pose(9, 56, 0);
            return pose;
            //Cyan
        } else if ((red > 35 && red < 55)
                && (green > 124 && green < 144)
                && (blue > 244 && blue <= 255)) {
            isBlueTeam = true;
            pose = new Pose(9, 88, 0);
            return pose;
            //Ivory
        } else if ((red > 245 && red <= 255)
                && (green > 245 && green <= 255)
                && (blue > 245 && blue <= 255)) {
            isBlueTeam = false;
            pose = new Pose(124, 22, -Math.toRadians(45));
            return pose;
            //Wood (Chat)
        } else if ((red > 88 && red < 108)
                && (green > 140 && green < 160)
                && (blue > 81 && blue < 101)) {
            isBlueTeam = true;
            pose = new Pose(124, 122, Math.toRadians(45));
            return pose;
        }
        return pose;
    }

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

        //Hardware map declarations
        ballz = hardwareMap.get(Servo.class, "stopper");
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        lFlywheel = hardwareMap.get(DcMotorEx.class, "leftFly");
        rFlywheel = hardwareMap.get(DcMotorEx.class, "rightFly");
        intake = hardwareMap.get(DcMotor.class, "intake");


        ballz.setDirection(Servo.Direction.REVERSE);

        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Zero power behaviors for Flywheels
        lFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set Flywheel modes for velocity running instead of power percentage
        lFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //comment out set modes for the fly wheels above if we want to use the PID for the flywheels
        //PIDFCoefficients newPID = new PIDFCoefficients(3.0,0.8,0.0,10.0);
        //lFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPID);
        //rFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPID);

        //Reverse left flywheel to oppose right
        lFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        aPressed = false;       // Tracks if A is toggled ON or OFF
        previousAState = false; // Tracks the button state from the previous loop cycle

        yPressed = false;       // Tracks if Y is toggled ON or OFF
        previousYState = false; // Tracks the button state from the previous loop cycle

        xPressed = true;       // Tracks if X is toggled ON or OFF
        previousXState = false; // Tracks the button state from the previous loop cycle

        bPressed = false;       // Tracks if B is toggled ON or OFF
        previousBState = false; // Tracks the button state from the previous loop cycle

        dPadDownToggle = false;         //Tracks if D-Pad Down is toggled ON or OFF
        previousDPadDownState = false;  //Tracks the button state from the previous loop cycle

        dPadUpToggle = false;         //Tracks if D-Pad Up is toggled ON or OFF
        previousDPadUpState = false;  //Tracks the button state from the previous loop cycle

        dPadLeftToggle = false;         //Tracks if D-Pad Up is toggled ON or OFF
        previousDPadLeftState = false;  //Tracks the button state from the previous loop cycle

        dPadRightToggle = false;
        previousDPadRightState = false;

        adjustSpeed = true;         //Checks whether or not speed was adjusted
        greenLight = false;         //permission to shoot
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


        //tgtTheta = getTargetAngle(isBlueTeam, follower);
        distFromGoal = getDist(isBlueTeam, follower);
        hypotenuse = calcHypotenuse(follower.getPose().getX(), follower.getPose().getY());
        //calculate shooting speed tolerances based on distance from the goal
        if (distFromGoal < 110) {
            negativeTol = 20.0;
            positiveTol = 80.0;
        } else {
            negativeTol = 20.0;
            positiveTol = 60.0;
        }

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

        if (gamepad1.dpad_left && !previousDPadLeftState) { dPadLeftToggle = !dPadLeftToggle; }
        previousDPadLeftState = gamepad1.dpad_left;

        if (gamepad1.dpad_right&& !previousDPadRightState) { dPadRightToggle = !dPadRightToggle; }
        previousDPadRightState = gamepad1.dpad_left;

        // --- ACTION LOGIC ---
        //Moved from outside of the While OpMode is active, do we need a while loop? Can we use
        //a pedropath to get in position without changing how we handle the wheels
        //use the orient function??

        //determine if there is a greenLight to shoot
        if (lFlywheel.getVelocity() >= liftoffPoly - negativeTol && lFlywheel.getVelocity() <= liftoffPoly + positiveTol && rFlywheel.getVelocity() >= liftoffPoly - negativeTol && rFlywheel.getVelocity() <= liftoffPoly + positiveTol) {
            greenLight = true;
        } else {
            greenLight = false;
        }

        // Set Belt speed. X (reverse) overrides A (forward).
        if (bPressed) {
            beltSpeed = 2000;
        } else if (yPressed) {
            beltSpeed = -2400;
        } else if (greenLight) {
            beltSpeed = -2100;
        }
        else {
            beltSpeed = 0.0;
        }

        // Set intake speed.
        if (aPressed) {
            intakeSpeed = 1.0;
            beltSpeed = -2400;
        } else if (greenLight) {
            intakeSpeed = 0.5;
        }
        else {
            intakeSpeed = 0.0;
        }

        if (xPressed) {
            ballzPos = 0.7;
        } else {
            ballzPos = 0.0;
        }

        distFromGoal = getDist(isBlueTeam, follower);

        liftoffPoly = powerRegressionPoly(distFromGoal) + adjustedSpeed;
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

        // Increment and Decrement Flywheel Speed
        // max speed is 40% of max rotation, min is 10% of min rotation.
        if (gamepad1.right_bumper && liftoffPoly >= 0.1 && adjustSpeed) {
            adjustedSpeed += 20.0;
            adjustSpeed = false;
        }
        // Check if speed was adjusted.
        if (gamepad1.rightBumperWasReleased()) {
            adjustSpeed = true;
        }

        if (gamepad1.left_bumper && liftoffPoly >= 0.1 && adjustSpeed) {
            adjustedSpeed -= 20.0;
            adjustSpeed = false;
        }
        // Check if speed was adjusted.
        if (gamepad1.leftBumperWasReleased()) {
            adjustSpeed = true;
        }

        // Set ballz position
        ballz.setPosition(ballzPos);

        // Set Intake power percentage
        intake.setPower(intakeSpeed);
        // Set belt power percentage
        belt.setVelocity(beltSpeed);

        // Set flywheel velocities (LB = High power shot, RB = Low power shot)
        if (gamepad1.right_trigger > 0) {
            lFlywheel.setVelocity(liftoffPoly);
            rFlywheel.setVelocity(liftoffPoly);
            ballz.setPosition(0.0);
        } else if (dPadRightToggle) { // Make sure that reverse motion can be toggled so we don't get fouled :/
            lFlywheel.setVelocity(MAX_TICKS_PER_SECOND * -0.01);
            rFlywheel.setVelocity(MAX_TICKS_PER_SECOND * -0.01);
        } else { // Set -100 by default.
            lFlywheel.setVelocity(0);
            rFlywheel.setVelocity(0);
            greenLight = false;
        }

        telemetry.addData("Flywheel Target TPS", "%4.2f", liftoffPoly);
        telemetry.addData("Flywheel TPS Left/Right", "%4.2f, %4.2f", rFlywheel.getVelocity(), lFlywheel.getVelocity());
        telemetry.addData("Belt TPS", "%4.2f", belt.getVelocity());
        telemetry.addData("Ballz target", "%4.2f", ballzPos);
        telemetry.addData("Ballz", "%4.2f", ballz.getPosition());
        telemetry.addData("Intake", "%4.2f", intakeSpeed);
        telemetry.addData("Heading", "%.2f degrees", follower.getHeading()*57.296);
        telemetry.addData("PedroPose", "X: %.2f in, \nY: %.2f in, \nHeading %.2f degrees", follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Auto-Aim Active", follower.isBusy() ? "Yes" : "No");
        telemetry.addData("Target Heading","%4.2f degrees", tgtTheta*57.296);
        telemetry.addData("Target Distance","%4.2f in", distFromGoal);
        telemetry.addData("GOAL POS", isBlueTeam ? BLUEGOAL.getPose() : REDGOAL.getPose());
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
