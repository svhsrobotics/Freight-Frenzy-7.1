package org.firstinspires.ftc.teamcode.util;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class HardwareNotFoundException extends RuntimeException {
    private final String name;
    private final String type;

    public HardwareNotFoundException(IllegalArgumentException e, String type) {
        this.type = type;
        this.name = extractNameFromMessage(e.getMessage());

    }
    public HardwareNotFoundException(IllegalArgumentException e) {
        String message = e.getMessage();
        this.type = extractClassFromMessage(message);
        this.name = extractNameFromMessage(message);
    }

    private static String extractNameFromMessage(String message) {
        Pattern pattern = Pattern.compile("\"([^\"]*)\"");
        Matcher matcher = pattern.matcher(message);
        if (matcher.find()) {
            return matcher.group(1);
        } else {
            return null;
        }
    }

    private static String extractClassFromMessage(String message) {
        Pattern pattern = Pattern.compile("(?<=type ).*$");
        Matcher matcher = pattern.matcher(message);
        if (matcher.find()) {
            return matcher.group(0);
        } else {
            return null;
        }
    }

    public HardwareNotFoundException(String name, Class<?> type) {
        this.name = name;
        this.type = type.getSimpleName();
    }

    public String getName() {
        return name;
    }

    public String getType() {
        return type;
    }

    @Override
    public String getMessage() {
        return String.format("Unable to find a hardware device with name \"%s\" and type %s", name, type);
    }
}
