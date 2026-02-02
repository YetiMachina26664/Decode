package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.0)
            .strafePodX(0.0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8) //0.8
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .xVelocity(49.909)
            .yVelocity(41.175)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.8)
            .forwardZeroPowerAcceleration(-34.431) //-34.431
            .lateralZeroPowerAcceleration(-59.724) // -59.724
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2,0.0,0.005,0.001))
            .translationalPIDFSwitch(4) //from example constants
            .headingPIDFCoefficients(new PIDFCoefficients(1.8,0.0,0.1,0.001))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.00035,0.6,0.015)) //2,0,0.1,0.5,0.001 switch to Pedro example
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.000005,0.6,.01)) //from example constants
            .drivePIDFSwitch(15) //from example constants
            .centripetalScaling(0.0005)
            .turnHeadingErrorThreshold(0.03) //0.01
            ;

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    ); //from example onstants
    //public static PathConstraints pathConstraints = new PathConstraints(0.6, 60, 0.3, 0.3);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
