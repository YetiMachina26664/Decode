package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoTemporary", group="Auto")
public class TemporaryAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftF       = null;
    private DcMotor leftB       = null;
    private DcMotor rightF      = null;
    private DcMotor rightB      = null;

    @Override
    public void runOpMode() {

        // HARDWAAAAAAAAREEEE MAAAAAAAAAAAAAAAAAAPPPPIIIIIIIIING
        leftF   = hardwareMap.get(DcMotor.class, "lf");
        leftB   = hardwareMap.get(DcMotor.class, "lb");
        rightF  = hardwareMap.get(DcMotor.class, "rf");
        rightB  = hardwareMap.get(DcMotor.class, "rb");


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
        leftF.setDirection(DcMotor.Direction.FORWARD);
        leftB.setDirection(DcMotor.Direction.FORWARD);
        rightF.setDirection(DcMotor.Direction.REVERSE);
        rightB.setDirection(DcMotor.Direction.REVERSE);

        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && runtime.seconds() < 2) {
            telemetry.addData("Status", "Driving Forward");
            telemetry.addData("Time", "%.2f seconds", runtime.seconds());
            telemetry.update();

            leftF.setPower(0.25);
            rightF.setPower(0.25);
            leftB.setPower(0.25);
            rightB.setPower(0.25);
        }

        // Step 2: Stop all motors
        // This code executes immediately after the loop above finishes.
        telemetry.addData("Status", "Stopping");
        telemetry.update();

        leftF.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);
        rightB.setPower(0);
    }
}