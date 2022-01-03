package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.vision.HSVColor;
import org.firstinspires.ftc.teamcode.vision.Region;
import org.firstinspires.ftc.teamcode.vision.TeamElementDetector;

import java.util.HashMap;

/**
 * Add all config values to this class
 */
public class Configuration {
    public HashMap<TeamElementDetector.TeamElementPosition, Region> regions;
    public HSVColor target;
    public Double threshold;
}
