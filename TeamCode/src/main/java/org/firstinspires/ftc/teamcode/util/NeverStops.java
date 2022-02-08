package org.firstinspires.ftc.teamcode.util;

public class NeverStops implements External {

    @Override
    public boolean shouldStop() {
        return false;
    }
}
