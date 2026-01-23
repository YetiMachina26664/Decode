package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class ColorSenseiTest extends LinearOpMode {
    RevColorSensorV3 color;

    @Override
    public void runOpMode() {
        if ((color.red() > 212 && color.red() < 232)
                && (color.green() > 57 && color.green() < 77)
                && (color.blue() > 57 && color.blue() < 77)) {

        }
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }
    }
}
