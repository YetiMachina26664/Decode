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
import com.qualcomm.robotcore.hardware.Servo;
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

    private DcMotorEx belt = null;
    private DcMotorEx lFlywheel = null;
    private DcMotorEx rFlywheel = null;
    private DcMotor intake = null;

    private Servo ballz = null;

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
    public static final Pose redFrontShoot = new Pose(12,60,-Math.toRadians(24.44));
    public static final Pose blueFrontShoot = new Pose(12,84,-Math.toRadians(24.44));
    public static final Pose redBackShoot = new Pose(88,56,-Math.toRadians(45.0));
    public static final Pose blueBackShoot = new Pose(88,88,-Math.toRadians(45.0));

    //collection start poses; Line 1 is the closest to the front (away from the goal)
    public static final Pose redLine1CollectStart = new Pose(36,39,Math.toRadians(90.0));
    public static final Pose redLine2CollectStart = new Pose(60,39,Math.toRadians(90.0));
    public static final Pose redLine3CollectStart = new Pose(84,39,Math.toRadians(90.0));
    public static final Pose blueLine1CollectStart = new Pose(36,105,-Math.toRadians(90.0));
    public static final Pose blueLine2CollectStart = new Pose(60,105,-Math.toRadians(90.0));
    public static final Pose blueLine3CollectStart = new Pose(84,105,-Math.toRadians(90.0));

    //collection finish poses
    public static final Pose redLine1CollectFinish = new Pose(36,16,Math.toRadians(90.0));
    public static final Pose redLine2CollectFinish = new Pose(60,16,Math.toRadians(90.0));
    public static final Pose redLine3CollectFinish = new Pose(84,16,Math.toRadians(90.0));
    public static final Pose blueLine1CollectFinish = new Pose(36,128,-Math.toRadians(90.0));
    public static final Pose blueLine2CollectFinish = new Pose(60,128,-Math.toRadians(90.0));
    public static final Pose blueLine3CollectFinish = new Pose(84,128,-Math.toRadians(90.0));

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


    boolean isAtPose = false;

    boolean greenLight;
    boolean isCollecting;
    boolean isBlueTeam;
    boolean backStart;

    private Path initialScore;
    private PathChain pickupStart,pickupEnd,safeZone,secondShoot;

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
            return 960;
        } else if (x < 140) {
            //return (int) (Math.round((1227 - 14.4 * x + 0.194 * Math.pow(x, 2) - .0007 * Math.pow(x, 3))/ 20) * 20);
            return (int) (Math.round((660 + 5.0 * x )/ 20) * 20);
        } else {
            return 1360;
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

    public void buildPaths(boolean blueTeam,boolean back) {
        //set paths for each action

        if (blueTeam && !back) {
            initialScore = new Path(new BezierLine(blueFrontStart,blueFrontShoot));
            initialScore.setLinearHeadingInterpolation(blueFrontStart.getHeading(),blueFrontShoot.getHeading());

            pickupStart = follower.pathBuilder()
                    .addPath(new BezierLine(blueFrontShoot,blueLine1CollectStart))
                    .setLinearHeadingInterpolation(blueFrontShoot.getHeading(),blueLine1CollectStart.getHeading())
                    .build();

            pickupEnd = follower.pathBuilder()
                    .addPath(new BezierLine(blueLine1CollectStart,blueLine1CollectFinish))
                    .setLinearHeadingInterpolation(blueLine1CollectStart.getHeading(),blueLine1CollectFinish.getHeading())
                    .build();

            safeZone = follower.pathBuilder()
                    .addPath(new BezierLine(blueLine1CollectFinish,blueLine1CollectStart))
                    .setLinearHeadingInterpolation(blueLine1CollectFinish.getHeading(),blueLine1CollectStart.getHeading())
                    .build();

            secondShoot = follower.pathBuilder()
                    .addPath(new BezierLine(blueLine1CollectStart,blueFrontShoot))
                    .setLinearHeadingInterpolation(blueLine1CollectStart.getHeading(),blueFrontShoot.getHeading())
                    .build();

        } else if (blueTeam && back) {
            initialScore = new Path(new BezierLine(blueBackStart,blueBackShoot));
            initialScore.setLinearHeadingInterpolation(blueBackStart.getHeading(),blueBackShoot.getHeading());

            pickupStart = follower.pathBuilder()
                    .addPath(new BezierLine(blueBackShoot,blueLine1CollectStart))
                    .setLinearHeadingInterpolation(blueBackShoot.getHeading(),blueLine1CollectStart.getHeading())
                    .build();

            pickupEnd = follower.pathBuilder()
                    .addPath(new BezierLine(blueLine1CollectStart,blueLine1CollectFinish))
                    .setLinearHeadingInterpolation(blueLine1CollectStart.getHeading(),blueLine1CollectFinish.getHeading())
                    .build();

            safeZone = follower.pathBuilder()
                    .addPath(new BezierLine(blueLine1CollectFinish,blueLine1CollectStart))
                    .setLinearHeadingInterpolation(blueLine1CollectFinish.getHeading(),blueLine1CollectStart.getHeading())
                    .build();

            secondShoot = follower.pathBuilder()
                    .addPath(new BezierLine(blueLine1CollectStart,blueBackShoot))
                    .setLinearHeadingInterpolation(blueLine1CollectStart.getHeading(),blueBackShoot.getHeading())
                    .build();
        } else if (!blueTeam && !back) {
            initialScore = new Path(new BezierLine(redFrontStart,redFrontShoot));
            initialScore.setLinearHeadingInterpolation(redFrontStart.getHeading(),redFrontShoot.getHeading());

            pickupStart = follower.pathBuilder()
                    .addPath(new BezierLine(redFrontShoot,redLine1CollectStart))
                    .setLinearHeadingInterpolation(redFrontShoot.getHeading(),redLine1CollectStart.getHeading())
                    .build();

            pickupEnd = follower.pathBuilder()
                    .addPath(new BezierLine(redLine1CollectStart,redLine1CollectFinish))
                    .setLinearHeadingInterpolation(redLine1CollectStart.getHeading(),redLine1CollectFinish.getHeading())
                    .build();

            safeZone = follower.pathBuilder()
                    .addPath(new BezierLine(redLine1CollectFinish,redLine1CollectStart))
                    .setLinearHeadingInterpolation(redLine1CollectFinish.getHeading(),redLine1CollectStart.getHeading())
                    .build();

            secondShoot = follower.pathBuilder()
                    .addPath(new BezierLine(redLine1CollectStart,redFrontShoot))
                    .setLinearHeadingInterpolation(redLine1CollectStart.getHeading(),redFrontShoot.getHeading())
                    .build();
        } else {
            initialScore = new Path(new BezierLine(redBackStart,redBackShoot));
            initialScore.setLinearHeadingInterpolation(redBackStart.getHeading(),redBackShoot.getHeading());

            pickupStart = follower.pathBuilder()
                    .addPath(new BezierLine(redBackShoot,redLine1CollectStart))
                    .setLinearHeadingInterpolation(redBackShoot.getHeading(),redLine1CollectStart.getHeading())
                    .build();

            pickupEnd = follower.pathBuilder()
                    .addPath(new BezierLine(redLine1CollectStart,redLine1CollectFinish))
                    .setLinearHeadingInterpolation(redLine1CollectStart.getHeading(),redLine1CollectFinish.getHeading())
                    .build();

            safeZone = follower.pathBuilder()
                    .addPath(new BezierLine(redLine1CollectFinish,redLine1CollectStart))
                    .setLinearHeadingInterpolation(redLine1CollectFinish.getHeading(),redLine1CollectStart.getHeading())
                    .build();

            secondShoot = follower.pathBuilder()
                    .addPath(new BezierLine(redLine1CollectStart,redBackShoot))
                    .setLinearHeadingInterpolation(redLine1CollectStart.getHeading(),redBackShoot.getHeading())
                    .build();
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        greenLight = false;
        isCollecting = false;
        pathTimer = new Timer();
        color = hardwareMap.get(RevColorSensorV3.class, "color");

        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //Hardware map declarations
        ballz = hardwareMap.get(Servo.class, "stopper");
        belt = hardwareMap.get(DcMotorEx.class, "belt");
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
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Zero power behaviors for Flywheels
        lFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Flywheel modes for velocity running instead of power percentage
        lFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        belt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reverse left flywheel to oppose right
        lFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        //gate direction
        ballz.setDirection(Servo.Direction.REVERSE);
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
        buildPaths(isBlueTeam,backStart);
    }
    public void autonomousPathUpdate() {







        switch (pathState) {
            case 0:
                follower.followPath(initialScore,true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 6.0) {
                        setPathState(2);
                    }
                    //Get TPS value for shooting
                    tgtTheta = getTargetAngle(isBlueTeam, follower); //use this to compare to current pose and only shoot if within threshold
                    distFromGoal = getDist(isBlueTeam, follower);
                    liftoffPoly = powerRegressionPoly(distFromGoal);
                    liftoffLin = powerRegressionLin(distFromGoal);

                    //calculate shooting speed tolerances based on distance from the goal
                    if (distFromGoal < 110) {
                        negativeTol = 20.0;
                        positiveTol = 80.0;
                    } else {
                        negativeTol = 20.0;
                        positiveTol = 60.0;
                    }

                    lFlywheel.setVelocity(liftoffPoly);
                    rFlywheel.setVelocity(liftoffPoly);
                    ballz.setPosition(0.0);

                    //determine if there is a greenLight to shoot
                    if (lFlywheel.getVelocity() >= liftoffPoly - negativeTol && lFlywheel.getVelocity() <= liftoffPoly + positiveTol && rFlywheel.getVelocity() >= liftoffPoly - negativeTol && rFlywheel.getVelocity() <= liftoffPoly + positiveTol) {
                        greenLight = true;
                    } else {
                        greenLight = false;
                    }
                    if (greenLight) {
                        beltSpeed = -2100;
                        intakeSpeed = 0.5;
                    } else {
                        intakeSpeed = 0.0;
                        beltSpeed = 0.0;
                    }
                    belt.setVelocity(beltSpeed);
                    intake.setPower(intakeSpeed);

                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    lFlywheel.setVelocity(0.0);
                    rFlywheel.setVelocity(0.0);
                    ballz.setPosition(0.7);
                    belt.setVelocity(-2100);
                    intake.setPower(1.0);
                    follower.followPath(pickupStart);
                    setPathState(3);
                }

                break;
            case 3:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pickupEnd, 0.4, true);
                    if (pathTimer.getElapsedTimeSeconds() >= 4.0) {
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(safeZone);
                    setPathState(6);
                }
                break;
            case 6:

                if (!follower.isBusy()) {
                    belt.setVelocity(-2100);
                    intake.setPower(1.0);
                    follower.followPath(secondShoot,true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() >= 6.0) {
                        setPathState(8);
                    }
                    //Get TPS value for shooting
                    tgtTheta = getTargetAngle(isBlueTeam, follower); //use this to compare to current pose and only shoot if within threshold
                    distFromGoal = getDist(isBlueTeam, follower);
                    liftoffPoly = powerRegressionPoly(distFromGoal);
                    liftoffLin = powerRegressionLin(distFromGoal);

                    //calculate shooting speed tolerances based on distance from the goal
                    if (distFromGoal < 110) {
                        negativeTol = 20.0;
                        positiveTol = 80.0;
                    } else {
                        negativeTol = 20.0;
                        positiveTol = 60.0;
                    }

                    lFlywheel.setVelocity(liftoffPoly);
                    rFlywheel.setVelocity(liftoffPoly);
                    ballz.setPosition(0.0);

                    //determine if there is a greenLight to shoot
                    if (lFlywheel.getVelocity() >= liftoffPoly - negativeTol && lFlywheel.getVelocity() <= liftoffPoly + positiveTol && rFlywheel.getVelocity() >= liftoffPoly - negativeTol && rFlywheel.getVelocity() <= liftoffPoly + positiveTol) {
                        greenLight = true;
                    } else {
                        greenLight = false;
                    }
                    if (greenLight) {
                        beltSpeed = -2100;
                        intakeSpeed = 0.5;
                    } else {
                        intakeSpeed = 0.0;
                        beltSpeed = 0.0;
                    }
                    belt.setVelocity(beltSpeed);
                    intake.setPower(intakeSpeed);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    lFlywheel.setVelocity(0.0);
                    rFlywheel.setVelocity(0.0);
                    ballz.setPosition(0.7);
                    belt.setVelocity(0);
                    intake.setPower(0.0);
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {


        follower.update();
        telemetryM.update();
        autonomousPathUpdate();



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
