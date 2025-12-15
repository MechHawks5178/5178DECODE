package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensor {
    private NormalizedColorSensor colorSensor;
    private final boolean DEBUG_ON = true;

    public enum DetectedColor
    {
        PURPLE,
        GREEN,
        UNK
    }

    public void init(HardwareMap hwMap, String device_name)
    {
        colorSensor = hwMap.get(NormalizedColorSensor.class, device_name);
        colorSensor.setGain(10);
    }

    public DetectedColor getDetectedColor(Telemetry telem)
    {
        DetectedColor retval = DetectedColor.UNK;
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //returns red/green/blue/alpha
        float hue = JavaUtil.colorToHue(colors.toColor());

        if ((hue >= 220.0) && (hue <= 240.0) )
        {
            retval = DetectedColor.PURPLE;
        }
        else if ((hue >= 150.0) && (hue <= 165.0) )
        {
            retval = DetectedColor.GREEN;
        }

        if (DEBUG_ON)
        {
            float normRed =   colors.red / colors.alpha; //normalize against alpha (light)
            float normGreen = colors.green / colors.alpha;
            float normBlue =  colors.blue / colors.alpha;

            telem.addData("red: ", normRed);
            telem.addData("green: ", normGreen);
            telem.addData("blue: ", normBlue);
            telem.addData("alpha: ", colors.alpha);
            telem.addData("Hue: ", JavaUtil.colorToHue(colors.toColor()));
            telem.addData("Color: ", colors.toColor());
        }

        return retval;
    }
}
