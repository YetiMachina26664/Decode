package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@Autonomous(name="Competition Auto")
public class OdoAuto extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Follower follower;
    private TelemetryManager telemetryM;

    private RevColorSensorV3 color;

    private DcMotor belt = null;
    private DcMotorEx lFlywheel = null;
    private DcMotorEx rFlywheel = null;
    private DcMotor intake = null;

    // motor declarations - copilot instructed to add
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rf = null;
    private DcMotor rb = null;

    //Initial flywheel percentages for far and close shots respectively
    double liftoffPoly;
    double liftoffLin;

    //Intake and Belt initial speeds
    double intakeSpeed = 0;
    double beltSpeed = 0;
    double ballzPos = 0.0;

    public static final double MAX_TICKS_PER_SECOND = 2840;
    public static final double MAX_TICKS_BELT = 2840;
    public static final Pose REDGOAL = new Pose(144, 0);
    public static final Pose BLUEGOAL = new Pose(144, 144);

    //initial poses
    public static final Pose redBackStart = new Pose(124.000, 22.000, -Math.toRadians(54));
    public static final Pose redFrontStart = new Pose(9.000, 56.000, Math.toRadians(0));
    public static final Pose blueBackStart = new Pose(124.000, 122.000, Math.toRadians(54));
    public static final Pose blueFrontStart = new Pose(9.000, 88.000, Math.toRadians(0));
    //shooting poses
    public static final Pose redFrontShoot = new Pose()
    //collection poses

    //final poses
    public static final Pose redBackEnd = new Pose(100.000, 24.000, -Math.toRadians(25));
    public static final Pose redFrontEnd = new Pose(30.000, 56.000, -Math.toRadians(40));
    public static final Pose blueBackEnd = new Pose(100.000, 120.000, Math.toRadians(25));
    public static final Pose blueFrontEnd = new Pose(30.000, 88.000, Math.toRadians(40));

    double distFromGoal;
    double tgtTheta;
    double hypotenuse;
    double negativeTol;
    double positiveTol;

    double XTemp;
    double YTemp;
    boolean isAtPose = false;
    private boolean pathStarted = false;
    private boolean pathFinished = false;

    boolean isBlueTeam;
    boolean backStart;

    private boolean isAiming = false;
    private boolean isMovingToBalls = false;
    private boolean isMovingToShootZone = false;
    private boolean isMovingToFinalPos = false;
    private boolean collectingBalls = false;

    private double shootStartTime = 0.0;
    private double collectingStartTime = 0.0;

    private Pose finalParkPose = new Pose(30.0,30.0,0.0);

    private Pose ballRowPose;
    private Pose shootZonePose;
    private Path toBallsPath;
    private Path toShootZonePath;
    private Path toFinalPath;
    private boolean extraPathsInitialized = false;

    Path blueTeamFront;
    Path redTeamFront;
    Path blueTeamBack;
    Path redTeamBack;

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
            pose = new Pose(124, 22, -0.925025);
            return pose;
            //Wood (Chat)
        } else if ((red > 88 && red < 108)
                && (green > 140 && green < 160)
                && (blue > 81 && blue < 101)) {
            isBlueTeam = true;
            backStart = true;
            pose = new Pose(124, 122, 0.925025);
            return pose;
        }
        return pose;
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        color = hardwareMap.get(RevColorSensorV3.class, "color");

        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        blueTeamBack = new Path(new BezierLine(blueBackStart, blueBackEnd));
        blueTeamBack.setLinearHeadingInterpolation(blueBackStart.getHeading(), blueBackEnd.getHeading());

        blueTeamFront = new Path(new BezierLine(blueFrontStart, blueFrontEnd));
        blueTeamFront.setLinearHeadingInterpolation(blueFrontStart.getHeading(), blueFrontEnd.getHeading());

        redTeamBack = new Path(new BezierLine(redBackStart, redBackEnd));
        redTeamBack.setLinearHeadingInterpolation(redBackStart.getHeading(), redBackEnd.getHeading());

        redTeamFront = new Path(new BezierLine(redFrontStart, redFrontEnd));
        redTeamFront.setLinearHeadingInterpolation(redFrontStart.getHeading(), redFrontEnd.getHeading());

        //Hardware map declarations
        belt = hardwareMap.get(DcMotor.class, "belt");
        lFlywheel = hardwareMap.get(DcMotorEx.class, "leftFly");
        rFlywheel = hardwareMap.get(DcMotorEx.class, "rightFly");
        intake = hardwareMap.get(DcMotor.class, "intake");


        // *** NEW: map drive motors ***
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        // *** NEW: force drive motors to RUN_WITHOUT_ENCODER ***
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        runtime.reset();
    }

    @Override
    public void loop() {
        //if (runtime.seconds() >= 4) {
        //    follower.pausePathFollowing();
        //}

        follower.update();
        telemetryM.update();

        //modified so that it only goes through this followpath command once
        if (!pathStarted) {
            if (isBlueTeam && backStart) {
                follower.followPath(blueTeamBack, true);
            } else if (isBlueTeam && !backStart) {
                follower.followPath(blueTeamFront, true);
            } else if (!isBlueTeam && backStart) {
                follower.followPath(redTeamBack, true);
            } else if (!isBlueTeam && !backStart) {
                follower.followPath(redTeamFront, true);
            }
            pathStarted = true;
        }

        //Check if the follower is still busy following the path
        if (pathStarted && !pathFinished && !follower.isBusy()) {
            pathFinished = true;
        }

        if (pathFinished) {
            // Optional: once done, you can hold current pose or hold the goal pose
            Pose current = follower.getPose();
            // hold where you are
            follower.holdPoint(new Pose(current.getX(), current.getY(), current.getHeading()));
        }

        //Get TPS value for shooting
        tgtTheta = getTargetAngle(isBlueTeam, follower); //use this to compare to current pose and only shoot if within threshold
        distFromGoal = getDist(isBlueTeam, follower);
        liftoffPoly = powerRegressionPoly(distFromGoal);
        liftoffLin = powerRegressionLin(distFromGoal);

        telemetry.addData("Flywheel Target TPS LIN/POLY", "%4.2f, %4.2f", liftoffLin, liftoffPoly);
        telemetry.addData("Flywheel TPS Left/Right", "%4.2f, %4.2f", rFlywheel.getVelocity(), lFlywheel.getVelocity());
        telemetry.addData("Belt", "%4.2f", beltSpeed);
        telemetry.addData("Intake", "%4.2f", intakeSpeed);
        telemetry.addData("Heading", "%.2f degrees", Math.toDegrees(follower.getHeading()));
        telemetry.addData("PedroPose", "X: %.2f in, \nY: %.2f in, \nHeading %.2f degrees", follower.getPose().getX(), follower.getPose().getY(),Math.toDegrees(follower.getPose().getHeading()));
        //telemetry.addData("Target Pose","X: %.2f in, \nY: %.2f in, \nHeading %.2f degrees", follower.getCurrentTargetPose().getX(),follower.getCurrentTargetPose().getY(),Math.toDegrees(follower.getCurrentTargetPose().getHeading()));
        //telemetry.addData("At Target Pose?", follower.atTarget());
        telemetry.addData("Auto-Aim Active", follower.isBusy() ? "Yes" : "No");
        telemetry.addData("Target Heading","%4.2f degrees", Math.toDegrees(tgtTheta));
        telemetry.addData("Target Distance","%4.2f in", distFromGoal);
        telemetry.addData("GOAL POS","X: %4.2f in, Y: %4.2f in", REDGOAL.getX(), REDGOAL.getY());
        telemetry.addData("isBusy",follower.isBusy());
        telemetry.addData("Runtime", "%4.2f", runtime.seconds());
        telemetry.update();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("L Current Velocity", lFlywheel.getVelocity());
        telemetryM.debug("R Current Velocity", rFlywheel.getVelocity());
        telemetryM.debug("Target V % Polynomial", liftoffPoly);
        telemetryM.debug("Target V % Linear", liftoffLin);
        telemetryM.debug("belt", beltSpeed);
        telemetryM.debug("intake", intakeSpeed);
        telemetryM.debug("isAtPose", isAtPose);
    }
}
