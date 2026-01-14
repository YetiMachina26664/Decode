package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Driver", group="Linear OpMode")
public class RobotDriver extends LinearOpMode {

    // Runtime Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Hardware Map Declarations
    private DcMotor leftF       = null;
    private DcMotor leftB       = null;
    private DcMotor rightF      = null;
    private DcMotor rightB      = null;
    private DcMotor belt        = null;
    private DcMotorEx lFlywheel   = null;
    private DcMotorEx rFlywheel   = null;
    private DcMotor intake      = null;
    private GoBildaPinpointDriver odo;

    // Flywheel Constants (Max ticks/second, velocity calculations)
    // ---DO NOT CHANGE---
    public static final double MAX_TICKS_PER_SECOND = 5376;
    public double flywheelTargetVelocity = 0.0;


    // OPMODE!!!!
    @Override
    public void runOpMode() {
        // Odometry initialization and offsets, etc.
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-137.5, -195.0); // These are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        // Mapping the hardware to previous declarations
        leftF   = hardwareMap.get(DcMotor.class, "lf");
        leftB   = hardwareMap.get(DcMotor.class, "lb");
        rightF  = hardwareMap.get(DcMotor.class, "rf");
        rightB  = hardwareMap.get(DcMotor.class, "rb");

        belt    = hardwareMap.get(DcMotor.class, "belt");

        lFlywheel    = hardwareMap.get(DcMotorEx.class, "leftFly");
        rFlywheel    = hardwareMap.get(DcMotorEx.class, "rightFly");

        intake  = hardwareMap.get(DcMotor.class, "intake");


        // Set motor directions to drive correctly
        leftF.setDirection(DcMotor.Direction.FORWARD);
        leftB.setDirection(DcMotor.Direction.FORWARD);
        rightF.setDirection(DcMotor.Direction.REVERSE);
        rightB.setDirection(DcMotor.Direction.REVERSE);

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

        //wait for start
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
        double liftoffHigh = 0.22;
        double liftoffLow = 0.19;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Update Odometry for tracking
            odo.update();
            // Get current robot pos for tracking
            Pose2D pos = odo.getPosition();

            // Initial intake and belt speeds
            double intakeSpeed  = 0;
            double beltSpeed    = 0;

            // BEGIN DRIVETRAIN CALCULATIONS
            // (This serves the purpose of making sure no motor goes over 100% power)
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x * 0.66; // Note: Multiplied by 0.66 as turning is sensitive, slows it down

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
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

            // --- ACTION LOGIC ---

            // Set Belt speed. X (reverse) overrides A (forward).
            if (xPressed) {
                beltSpeed   =  0.5;
            } else if (aPressed) {
                beltSpeed   = -1.0;
            } else if (aPressed && rFlywheel.getPower() > 0.0){
                beltSpeed   = -0.8;
            } else {
                beltSpeed   = 0.0;
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

            // Set Intake power percentage
            intake.setPower(intakeSpeed);
            // Set belt power percentage
            belt.setPower(beltSpeed);

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


            // Send previously calculated power to wheels
            leftF.setPower(frontLeftPower);
            rightF.setPower(frontRightPower);
            leftB.setPower(backLeftPower);
            rightB.setPower(backRightPower);

            // Show the elapsed game time, wheel powers, flywheel velocity data, belt and intake speed, and positioning data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Flywheel Target Percent LOW/HIGH", "%4.2f, %4.2f", liftoffLow * 100, liftoffHigh * 100);
            telemetry.addData("Flywheel Target RPM LOW/HIGH", "%4.2f, %4.2f", MAX_TICKS_PER_SECOND * liftoffLow, MAX_TICKS_PER_SECOND * liftoffHigh);
            telemetry.addData("Flywheel Velocity Left/Right", "%4.2f, %4.2f", rFlywheel.getVelocity(), lFlywheel.getVelocity());
            telemetry.addData("Belt", "%4.2f", beltSpeed);
            telemetry.addData("Intake", "%4.2f", intakeSpeed);
            telemetry.addData("Position", "X: %.2f in, %.2f in", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.2f degrees", pos.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
