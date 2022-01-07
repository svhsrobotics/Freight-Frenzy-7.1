package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logger {
    private final Telemetry telemetry;
    private final boolean debug;

    public Logger() {
        this(null, true);
    }

    public Logger(Telemetry telemetry) {
        this(telemetry, true);
    }

    public Logger(Telemetry telemetry, boolean debug) {
        this.telemetry = telemetry;
        this.debug = debug;

        if (telemetry != null) {
            this.telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            this.telemetry.update(); // Apply change in display format
        } else {
            warning("Telemetry was null, logs will not be available in telemetry!");
        }
    }

    // LOG LEVELS

    public void info(String msg) {
        android.util.Log.i(getCallerTag(getCallerClassName()), msg);
        if (telemetry != null) {
            telemetry.log().add(formatColor(prefix(msg, "INFO"), "#4CAF50"));
            telemetry.update();
        }
    }

    public void warning(String msg) {
        android.util.Log.w(getCallerTag(getCallerClassName()), msg);
        if (telemetry != null) {
            telemetry.log().add(formatColor(prefix(msg, "WARNING"), "#FFC107"));
            telemetry.update();
        }
    }

    public void error(String msg) {
        android.util.Log.e(getCallerTag(getCallerClassName()), msg);
        if (telemetry != null) {
            telemetry.log().add(formatColor(prefix(msg, "ERROR"), "#F44336"));
            telemetry.update();
        }
    }

    public void debug(String msg) {
        android.util.Log.d(getCallerTag(getCallerClassName()), msg);
        if (debug && telemetry != null) {
            telemetry.log().add(formatColor(prefix(msg, "DEBUG"), "#2196F3"));
            telemetry.update();
        }
    }

    // HELPER FUNCTIONS

    private static String prefix(String msg, String prefix) {
        return String.format("<b>[%s]</b> %s", prefix, msg);
    }

    private static String formatColor(String msg, String color) {
        return String.format("<span style=\"color:%s\">%s</span>", color, msg);
    }

    private static String getCallerTag(String name) {
        return name.substring(name.lastIndexOf('.') + 1);
    }
    private static String getCallerClassName() {
        StackTraceElement[] stElements = Thread.currentThread().getStackTrace();
        for (int i=1; i<stElements.length; i++) {
            StackTraceElement ste = stElements[i];
            // If it's not Logger (us) and it's not Thread, return it
            if (!ste.getClassName().equals(Logger.class.getName()) && ste.getClassName().indexOf("java.lang.Thread")!=0) {
                return ste.getClassName();
            }
        }
        return null;
    }
}
