package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

public class CheckPatternSubsystem
{
    private final IndexerSubsystem indexer;
    private boolean checking;
    private String[] indexerPattern;
    private int step;
    private boolean moving;
    private NormalizedColorSensor colorSensor;
    private NormalizedRGBA colors;
    private float[] hsv;
    private float hue;
    private float saturation;
    private float value;
    
    // color values(need to test)
    private final int green = 120;
    private final int purple = 300;
    private final int colorAccuracy = 50;

    public CheckPatternSubsystem(HardwareMap hardwareMap, IndexerSubsystem indexer, String colorSensorName)
    {
        this.indexer = indexer;
        checking = false;
        indexerPattern = {"empty", "empty", "empty"};
        step = 0;
        moving = false;
        this.colorSensor = hardwareMap.get(NormalizedColorSensor.class, colorSensorName);
        colors = colorSensor.getNormalizedColors();
        hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);
        hue = hsv[0];
        saturation = hsv[1];
        value = hsv[2];
    }

    public void update()
    {
        if (checking)
        {
            //steps, each step moves to a position and adds its color to the indexer
            if (step == 0)
            {
                if (!moving)
                {
                    moving = true;
                    indexer.setIndexerPosition("Launch1");
                }
                else if (moving && !indxer.isMoving())
                {
                    indexerPattern[0] = getColor();
                    moving = false;
                    step++;
                }
            }
            else if (step == 1)
            {
                if (!moving)
                {
                    moving = true;
                    indexer.setIndexerPosition("Launch2");
                }
                else if (moving && !indxer.isMoving())
                {
                    indexerPattern[1] = getColor();
                    moving = false;
                    step++;
                }
            }
            else if (step == 2)
            {
                if (!moving)
                {
                    moving = true;
                    indexer.setIndexerPosition("Launch3");
                }
                else if (moving && !indxer.isMoving())
                {
                    indexerPattern[2] = getColor();
                    moving = false;
                    checking = false;
                }
            }
        }
        else
        {
            // if not checking, reset all variables
            step = 0;
            moving = false;
        }
    }

    // calculates and returns the String of the ball color
    private String getColor()
    {
        hsv = getHsv();
        hue = hsv[0];
        saturation = hsv[1];
        value = hsv[2];
        if (hue > green - colorAccuracy && hue < green + colorAccuracy)
        {
            return "green";
        }
        else if (hue > purple - colorAccuracy && hue < purple + colorAccuracy)
        {
            return "purple";
        }
        /*
        else if (<black>)
        {
            return "empty";
        }
        */
        else
        {
            return "unknown";
        }
    }

    public void stop()
    {
        checking = false;
    }

    public void start()
    {
        checking = true;
    }

    public boolean getStatus()
    {
        return checking;
    }

    public String[] getIndexerPattern()
    {
        return indexerPattern;
    }

    public float[] getHsv()
    {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);
        return hsv;
    }
}