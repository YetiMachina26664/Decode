package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="Color Sensei Test")
public class ColorSenseiTest extends LinearOpMode {
    RevColorSensorV3 color;

    @Override
    public void runOpMode() {
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        telemetry.addLine("Color sensor initialized. waiting to start...");
        telemetry.update();

        NormalizedColorSensor normalizedSensor = (NormalizedColorSensor) color;
        normalizedSensor.setGain(40);

        waitForStart();

        while (opModeIsActive()) {
            // 3. Get the normalized colors (values 0.0 to 1.0)
            NormalizedRGBA colors = normalizedSensor.getNormalizedColors();

            // 4. Scale to 0-255
            int red   = (int) (colors.red * 255);
            int green = (int) (colors.green * 255);
            int blue  = (int) (colors.blue * 255);

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);

            //Scarlet
            if ((red > 155 && red < 175)
                    && (green > 72 && green < 92)
                    && (blue > 39 && blue < 59)) {
                telemetry.addLine("Scarlet");
                telemetry.update();
                //Cyan
            } else if ((red > 35 && red < 55)
                    && (green > 124 && green < 144)
                    && (blue > 244 && blue <= 255)) {
                telemetry.addLine("Cyan");
                telemetry.update();
                //Ivory
            } else if ((red > 245 && red <= 255)
                    && (green > 245 && green <= 255)
                    && (blue > 245 && blue <= 255)) {
                telemetry.addLine("Ivory");
                telemetry.update();
                //Wood (Chat)
            } else if ((red > 88 && red < 108)
                    && (green > 140 && green < 160)
                    && (blue > 81 && blue < 101)) {
                telemetry.addLine("Wood");
                telemetry.update();
            }

            telemetry.update();
        }
    }
}
