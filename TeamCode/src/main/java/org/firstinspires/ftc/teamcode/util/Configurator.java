package org.firstinspires.ftc.teamcode.util;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.lang.reflect.Type;

/**
 * This is a helper class used to store and retrieve non-volatile configuration values
 * as JSON files.
 *
 * This version is *not* backed by a map, instead, *all* types must be defined in Config.
 *
 * These files can be accessed by looking in the FIRST folder on the Control Hub.
 *
 * Note: on macOS you may need <a href="https://www.android.com/filetransfer/">Android File Transfer</a>.
 */
public class Configurator {

    public static Configuration load() {
        return load("config.json");
    }

    public static void save(Configuration config) {
        save(config, "config.json");
    }


    /**
     * Loads the configuration from the JSON file
     * Note that if no configuration file exists, a new one will be created automatically.
     */
    public static <T> T load(String filename) {
        Gson gson = new GsonBuilder()
                .setPrettyPrinting()
                .create();
        File file = AppUtil.getInstance().getSettingsFile(filename);
        // Use reflection to get the type
        Type type = new TypeToken<T>(){}.getType();
        return gson.fromJson(ReadWriteFile.readFile(file), type);
    }

    /**
     * Saves the configuration to the JSON file
     */
    public static <T> void save(T config, String filename) {
        Gson gson = new GsonBuilder()
                .setPrettyPrinting()
                .create();
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, gson.toJson(config));
    }
}
