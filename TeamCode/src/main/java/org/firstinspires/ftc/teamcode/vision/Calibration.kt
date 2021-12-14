package org.firstinspires.ftc.teamcode.vision

import com.google.gson.Gson

class Calibration(
        val regions: Map<TeamElementDetector.TeamElementPosition, Region>,
        val target: HSVColor,
        val threshold: Double,
        ) {
    fun serialize(): String {
        val gson: Gson = Gson();
        return gson.toJson(this);
    }
}