package org.firstinspires.ftc.teamcode;

//import pedropathing functions
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

//import qualcomm functions
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//import ftc functions - navigation
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//import constants from this folder
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//defines 'Driver Pos Based' for TeleOp
@TeleOp(name="Driver Pos Based", group="Linear OpMode")
public class RDriverLocBased extends LinearOpMode {

    // Runtime Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Hardware Map Declarations
    private DcMotor leftF = null;
    private DcMotor leftB = null;
    private DcMotor rightF = null;
    private DcMotor rightB = null;
    private DcMotor belt = null;
    private DcMotorEx lFlywheel = null;
    private DcMotorEx rFlywheel = null;
    private DcMotor intake = null;
    private GoBildaPinpointDriver odo;

    // Flywheel Constants (Max ticks/second, velocity calculations)
    // ---DO NOT CHANGE---
    public static final double MAX_TICKS_PER_SECOND = 5376;
    public double flywheelTargetVelocity = 0.0;

    //defining booleans for which alliance and where we start
    public boolean isBlueTeam = false;
    public boolean backStart = false;

    //defining objects for location of robot
    public Follower follower;

    //define position of goals, uses Pose2D from FTC navigation instead of Pedropathing, is this a problem?
    public Pose2D REDGOAL = new Pose2D(DistanceUnit.INCH, 134, 135, AngleUnit.DEGREES, 0);
    public Pose2D BLUEDGOAL = new Pose2D(DistanceUnit.INCH, 9, 135, AngleUnit.DEGREES, 0);

    //Don't know what this does, come back to it - Orient is a class
    Orient paths;

    //function to get the distance from the robot to the goal, relies on calcHypotenuse function
    //relies on blueTeam boolean and the current position given from Pose2D
    public double getDist(boolean blueTeam, Pose2D currentPos) {
        if (blueTeam) {
            return calcHypotenuse(
                    BLUEDGOAL.getX(DistanceUnit.INCH) - currentPos.getX(DistanceUnit.INCH),
                    BLUEDGOAL.getY(DistanceUnit.INCH) - currentPos.getY(DistanceUnit.INCH));
        } else {
            return calcHypotenuse(
                    REDGOAL.getX(DistanceUnit.INCH) - currentPos.getX(DistanceUnit.INCH),
                    REDGOAL.getY(DistanceUnit.INCH) - currentPos.getY(DistanceUnit.INCH));
        }
    }

    //get the target heading for the robot to the goal
    public double getTargetAngle(boolean blueTeam, Pose currentPos) {
        double tgtY, tgtX;
        if (blueTeam) {
            tgtY = BLUEDGOAL.getY(DistanceUnit.INCH) - currentPos.getY();
            tgtX = BLUEDGOAL.getX(DistanceUnit.INCH) - currentPos.getX();
        } else {
            tgtY = REDGOAL.getY(DistanceUnit.INCH) - currentPos.getY();
            tgtX = REDGOAL.getX(DistanceUnit.INCH) - currentPos.getX();
        }
        return Math.atan2(tgtY, tgtX);
    }

    //calculation for the straightline distance between the robot and the goal
    public double calcHypotenuse(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    //defines pedropath to orient the robot to align with the target heading
    public static class Orient {
        RDriverLocBased obj = new RDriverLocBased();

        public PathChain Path1;

        public Orient(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(follower.getPose(), follower.getPose())

                    )
                    .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(obj.getTargetAngle(obj.isBlueTeam, follower.getPose())))
                    .build();
        }
    }

    //function to run while in teleop
    @Override
    public void runOpMode() {
        isBlueTeam = false;
        backStart = false;

// 1. Create the follower object
        follower = Constants.createFollower(hardwareMap);

// 2. Initialize and fully configure your odometry driver
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-140, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.recalibrateIMU();

// 4. Now set the starting pose in the follower (which now uses your odo)
        if (isBlueTeam && backStart) {
            // ... your starting pose logic ...
        } else {
            follower.setStartingPose(new Pose(88, 17, 90)); //red start against wall
        }

// 5. UNknown value - delete this step? Or add to previous? ~~~Finally, create your paths. They will now be based on the correct, unified odometry.
        //paths = new RDriverLocBased.Orient(follower);


        //odo.recalibrateIMU();


        //if (isBlueTeam && backStart) {
        //    follower.setStartingPose(new Pose(28, 133, 324));
        //} else if (isBlueTeam && !backStart) {
        //    follower.setStartingPose(new Pose(56, 17, 90));
        //} else if (!isBlueTeam && backStart) {
        //    follower.setStartingPose(new Pose(116, 133, 216));
        //} else {
        //    follower.setStartingPose(new Pose(88, 17, 90));
        //}

        //paths = new RDriverLocBased.Orient(follower);

        // ... inside while(opModeIsActive())

// Get the latest data from your hardware
        //GoBilda currentPos2D provides X, Y, heading infor to pedropathing (currentPos)
        follower.update();
        odo.update();
        Pose2D currentPos2D = odo.getPosition();
        Pose currentPos = new Pose(currentPos2D.getX(DistanceUnit.INCH), currentPos2D.getY(DistanceUnit.INCH), currentPos2D.getHeading(AngleUnit.DEGREES));



// ... rest of your loop (flywheel, intake, telemetry, etc.) ...
        telemetry.addData("Auto-Aim Active", gamepad1.right_bumper);
        telemetry.addData("Target Heading", "%.2f deg", Math.toDegrees(getTargetAngle(isBlueTeam, currentPos)));
// ... your other telemetry ...
        telemetry.update();

        // Odometry initialization and offsets, etc.
        //odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //odo.setOffsets(-137.5, -195.0); // These are tuned for 3110-0002-0001 Product Insight #1
        //odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //odo.resetPosAndIMU(); */
        //        paths = new Orient(follower);


        // Mapping the hardware to previous declarations
        leftF = hardwareMap.get(DcMotor.class, "lf");

        // Mapping the hardware to previous declarations
        leftF = hardwareMap.get(DcMotor.class, "lf");
        leftB = hardwareMap.get(DcMotor.class, "lb");
        rightF = hardwareMap.get(DcMotor.class, "rf");
        rightB = hardwareMap.get(DcMotor.class, "rb");

        belt = hardwareMap.get(DcMotor.class, "belt");

        lFlywheel = hardwareMap.get(DcMotorEx.class, "leftFly");
        rFlywheel = hardwareMap.get(DcMotorEx.class, "rightFly");

        intake = hardwareMap.get(DcMotor.class, "intake");


        // Set motor directions to drive correctly
        leftF.setDirection(DcMotor.Direction.REVERSE);
        leftB.setDirection(DcMotor.Direction.REVERSE);
        rightF.setDirection(DcMotor.Direction.FORWARD);
        rightB.setDirection(DcMotor.Direction.FORWARD);

        //Zero power behaviors for motors
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //Toggle states for later toggle logic.
        boolean aPressed = false;       // Tracks if A is toggled ON or OFF
        boolean previousAState = false; // Tracks the button state from the previous loop cycle

        boolean yPressed = false;       // Tracks if Y is toggled ON or OFF
        boolean previousYState = false; // Tracks the button state from the previous loop cycle

        boolean xPressed = false;       // Tracks if X is toggled ON or OFF
        boolean previousXState = false; // Tracks the button state from the previous loop cycle

        boolean bPressed = false;       // Tracks if B is toggled ON or OFF
        boolean previousBState = false; // Tracks the button state from the previous loop cycle

        boolean dPadDownToggle = false;         //Tracks if D-Pad Down is toggled ON or OFF
        boolean previousDPadDownState = false;  //Tracks the button state from the previous loop cycle

        boolean dPadUpToggle = false;         //Tracks if D-Pad Up is toggled ON or OFF
        boolean previousDPadUpState = false;  //Tracks the button state from the previous loop cycle

        boolean adjustSpeed = true;         //Checks whether or not speed was adjusted

        // Initial flywheel percentages for far and close shots respectively
        double liftoff;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            follower.update();
            // Update Odometry for tracking
            odo.update();
            // Get current robot pos for tracking
            //Pose2D currentPos = odo.getPosition();

            // Initial intake and belt speeds
            //Why is this in the While loop?
            double intakeSpeed = 0;
            double beltSpeed = 0;

            // BEGIN DRIVETRAIN CALCULATIONS
            // (This serves the purpose of making sure no motor goes over 100% power)
            //Put this before the while loop?
            double max = 0;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //move sensitivity value to beginning of code as a variable
            //Should have a section of modifiable variables
            //Can these be moved outside of the While loop?
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x * 0.66; // Note: Multiplied by 0.66 as turning is sensitive, slows it down

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            // Can this be moved outside of the While loop?
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // --- TOGGLE LOGIC ---
            // This detects a single button press event for each button.
            if (gamepad1.a && !previousAState) {
                aPressed = !aPressed;
            }
            previousAState = gamepad1.a;

            if (gamepad1.y && !previousYState) {
                yPressed = !yPressed;
            }
            previousYState = gamepad1.y;

            if (gamepad1.x && !previousXState) {
                xPressed = !xPressed;
            }
            previousXState = gamepad1.x;

            if (gamepad1.b && !previousBState) {
                bPressed = !bPressed;
            }
            previousBState = gamepad1.b;

            if (gamepad1.dpad_down && !previousDPadDownState) {
                dPadDownToggle = !dPadDownToggle;
            }
            previousDPadDownState = gamepad1.dpad_down;

            if (gamepad1.dpad_up && !previousDPadUpState) {
                dPadUpToggle = !dPadUpToggle;
            }
            previousDPadUpState = gamepad1.dpad_up;

            // --- ACTION LOGIC ---
            //Moved from outside of the While OpMode is active, do we need a while loop? Can we use
            //a pedropath to get in position without changing how we handle the wheels
            //use the orient function??
            // --- AUTO-AIM LOGIC ---
            if (gamepad1.right_bumper) { // Use right_bumper to trigger auto-aim
                // 1. Calculate the target angle
                double targetAngleRad = getTargetAngle(isBlueTeam, currentPos);

                // 2. Get the current robot heading in radians
                double currentAngleRad = currentPos.getHeading();

                // 3. Calculate the angle error (how much we need to turn)
                // This must handle wrapping around (e.g., turning from 350 deg to 10 deg)
                double angleError = targetAngleRad - currentAngleRad;
                while (angleError > Math.PI) {
                    angleError -= 2 * Math.PI;
                }
                while (angleError < -Math.PI) {
                    angleError += 2 * Math.PI;
                }

                // 4. Use a P-controller to determine turn speed
                double P_TURN_GAIN = 0.8; // This is a tuning constant! Start with ~0.8 and adjust.
                double turnPower = angleError * P_TURN_GAIN;

                // Apply a minimum power to prevent the robot from stalling when it's close
                double MIN_TURN_POWER = 0.15; // Tuning constant
                if (Math.abs(angleError) > Math.toRadians(2.0)) { // Only apply min power if error is > 2 degrees
                    if (Math.abs(turnPower) < MIN_TURN_POWER) {
                        turnPower = Math.signum(turnPower) * MIN_TURN_POWER;
                    }
                } else {
                    turnPower = 0; // We are aimed, stop turning.
                }

                // 5. Send power to the wheels. Auto-aim overrides driver's yaw but keeps strafe/forward.
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = turnPower; // <<< The auto-aim power!

                // Your existing drive logic will now use the calculated yaw
                double frontLeftPower = axial + lateral + yaw;
                double frontRightPower = axial - lateral - yaw;
                double backLeftPower = axial - lateral + yaw;
                double backRightPower = axial + lateral - yaw;

                // BEGIN DRIVETRAIN CALCULATIONS
                // (This serves the purpose of making sure no motor goes over 100% power)
                double max = 0;

                // Normalize and send power to motors (copying your existing code)
                max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
                max = Math.max(max, Math.abs(backLeftPower));
                max = Math.max(max, Math.abs(backRightPower));

                if (max > 1.0) {
                    frontLeftPower /= max;
                    frontRightPower /= max;
                    backLeftPower /= max;
                    backRightPower /= max;
                }

                leftF.setPower(frontLeftPower);
                rightF.setPower(frontRightPower);
                leftB.setPower(backLeftPower);
                rightB.setPower(backRightPower);

                //add instructions here to shoot once you have the robot aimed
                //how long do we want to shoot for?

            } else {
                //Do we need this since the code is elswhere in the While Loop
                // --- NORMAL DRIVER CONTROL ---
                // This is your existing manual drive code. It runs when right_bumper is NOT pressed.
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = -gamepad1.right_stick_x * 0.66;

                // ... (the rest of your normal power calculation and normalization) ...

                double frontLeftPower = axial + lateral + yaw;
                double frontRightPower = axial - lateral - yaw;
                double backLeftPower = axial - lateral + yaw;
                double backRightPower = axial + lateral - yaw;

                // ... normalization code ...

                // Send previously calculated power to wheels
                leftF.setPower(frontLeftPower);
                rightF.setPower(frontRightPower);
                leftB.setPower(backLeftPower);
                rightB.setPower(backRightPower);
            }

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

            // Set Intake power percentage
            intake.setPower(intakeSpeed);
            // Set belt power percentage
            belt.setPower(beltSpeed);

            //liftoff = powerRegression(getDist(isBlueTeam, follower.getPose()));
            liftoff = 0;
            // Set flywheel velocities.
            if (gamepad1.left_bumper) {
                lFlywheel.setVelocity(MAX_TICKS_PER_SECOND * liftoff);
                rFlywheel.setVelocity(MAX_TICKS_PER_SECOND * liftoff);
            } else if (dPadUpToggle) { // Make sure that reverse motion can be toggled so we don't get fouled :/
                lFlywheel.setVelocity(MAX_TICKS_PER_SECOND * -0.01);
                rFlywheel.setVelocity(MAX_TICKS_PER_SECOND * -0.01);
            } else { // Set 0 by default.
                lFlywheel.setVelocity(0);
                rFlywheel.setVelocity(0);
            }

            while (gamepad1.left_trigger > 0) {
                follower.followPath(paths.Path1);
            }

            // Send previously calculated power to wheels
            leftF.setPower(frontLeftPower);
            rightF.setPower(frontRightPower);
            leftB.setPower(backLeftPower);
            rightB.setPower(backRightPower);

            // Show the elapsed game time, wheel powers, flywheel velocity data, belt and intake speed, and positioning data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Flywheel Target Percent", "%4.2f Percent", liftoff * 100);
            telemetry.addData("Flywheel Target RPM", "%4.2f RPM", MAX_TICKS_PER_SECOND * liftoff);
            telemetry.addData("Flywheel Velocity Left/Right", "%4.2f RPM, %4.2f RPM", rFlywheel.getVelocity(), lFlywheel.getVelocity());
            telemetry.addData("Belt", "%4.2f", beltSpeed);
            telemetry.addData("Intake", "%4.2f", intakeSpeed);
            telemetry.addData("PP Position", "X: %.2f in, Y: %.2f in", follower.getPose().getX(), follower.getPose().getY());
            telemetry.addData("GB Position", "X: %.2f in, Y: %.2f in", currentPos.getX(), currentPos.getX());
            telemetry.addData("Heading", "%.2f degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Heading", "%.2f degrees", currentPos.getHeading());
            telemetry.update();
        }
    }
}

