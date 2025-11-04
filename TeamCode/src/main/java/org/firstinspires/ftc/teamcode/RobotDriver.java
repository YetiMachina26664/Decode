package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver", group="Linear OpMode")
public class RobotDriver extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftF       = null;
    private DcMotor leftB       = null;
    private DcMotor rightF      = null;
    private DcMotor rightB      = null;
    private DcMotor belt        = null;
    private DcMotor lFlywheel   = null;
    private DcMotor rFlywheel   = null;
    private CRServo intake      = null;


    @Override
    public void runOpMode() {

        // HARDWAAAAAAAAREEEE MAAAAAAAAAAAAAAAAAAPPPPIIIIIIIIING
        leftF   = hardwareMap.get(DcMotor.class, "lf");
        leftB   = hardwareMap.get(DcMotor.class, "lb");
        rightF  = hardwareMap.get(DcMotor.class, "rf");
        rightB  = hardwareMap.get(DcMotor.class, "rb");

        belt    = hardwareMap.get(DcMotor.class, "belt");

        lFlywheel    = hardwareMap.get(DcMotor.class, "leftFly");
        rFlywheel    = hardwareMap.get(DcMotor.class, "rightFly");

        intake  = hardwareMap.get(CRServo.class, "intake");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftF.setDirection(DcMotor.Direction.REVERSE);
        leftB.setDirection(DcMotor.Direction.REVERSE);
        rightF.setDirection(DcMotor.Direction.FORWARD);
        rightB.setDirection(DcMotor.Direction.FORWARD);

        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean aPressed = false;       // Tracks if A is toggled ON or OFF
        boolean previousAState = false; // Tracks the button state from the previous loop cycle

        boolean yPressed = false;       // Tracks if Y is toggled ON or OFF
        boolean previousYState = false; // Tracks the button state from the previous loop cycle

        boolean xPressed = false;       // Tracks if X is toggled ON or OFF
        boolean previousXState = false; // Tracks the button state from the previous loop cycle

        boolean bPressed = false;       // Tracks if B is toggled ON or OFF
        boolean previousBState = false; // Tracks the button state from the previous loop cycle



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double intakeSpeed  = 0;

            double liftoff      = 0;
            double beltSpeed    = 0;

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  -gamepad1.right_stick_x;

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

            // Check if the 'a' button is currently pressed and wasn't pressed on the last loop iteration.
            // This detects a single button press event.
            if (gamepad1.a && !previousAState) {
                // Flip the state of our toggle
                aPressed = !aPressed;
            }

            // Update the previous state for the next loop iteration.
            previousAState = gamepad1.a;

            if (gamepad1.y && !previousYState) {
                // Flip the state of our toggle
                yPressed = !yPressed;
            }

            // Update the previous state for the next loop iteration.
            previousYState = gamepad1.y;

            if (gamepad1.x && !previousXState) {
                // Flip the state of our toggle
                xPressed = !xPressed;
            }

            // Update the previous state for the next loop iteration.
            previousXState = gamepad1.x;

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

            // --- ACTION LOGIC ---

            // Set Intake and Belt speeds. X (reverse) overrides A (forward).
            if (xPressed) {
                intakeSpeed =  0.0; // Assuming intake reverses direction with belt
                beltSpeed   = -0.6;
            } else if (aPressed) {
                intakeSpeed = -1.0;
                beltSpeed   =  1.0;
            } else {
                intakeSpeed = 0.0;
                beltSpeed   = 0.0;
            }

            // Set Flywheel speeds. B (0.47) overrides Y (0.43).
            if (bPressed) {
                liftoff = 0.46;
            } else if (yPressed) {
                liftoff = 0.41;
            } else {
                liftoff = 0.0;
            }

            intake.setPower(intakeSpeed);

            belt.setPower(beltSpeed);
            lFlywheel.setPower(-liftoff);
            rFlywheel.setPower(liftoff);

            // Send calculated power to wheels
            leftF.setPower(frontLeftPower);
            rightF.setPower(frontRightPower);
            leftB.setPower(backLeftPower);
            rightB.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("Flywheel", "%4.2f", liftoff);
            telemetry.addData("Belt", "%4.2f", beltSpeed);
            telemetry.addData("Intake", "%4.2f", intakeSpeed);
            telemetry.update();
        }
    }
}