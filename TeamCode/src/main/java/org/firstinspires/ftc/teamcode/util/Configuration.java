package org.firstinspires.ftc.teamcode.util;

import com.google.gson.Gson;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

/**
 * This is a helper class used to store and retrieve non-volatile configuration values
 * as JSON files.
 * These files can be accessed by looking in the FIRST folder on the Control Hub.
 *
 * Note: on macOS you may need <a href="https://www.android.com/filetransfer/">Android File Transfer</a>.
 */
public class Configuration {

    private final HashMap<String, Object> map = new HashMap<>();
    private final String filename;

    private final Gson gson = new Gson();

    public Configuration(String filename) {
        this.filename = filename;
        load();
    }

    public Configuration() {
        this("config.json");
    }

    /**
     * Clears the internal map
     */
    public void clear() {
        map.clear();
    }

    /**
     * Appends the contents of a map to the configuration
     * @param map to append
     */
    public void append(Map<String, Object> map) {
        for (String key : map.keySet()) {
            this.map.put(key, map.get(key));
        }
        save();
    }

    /**
     * Sets a configuration value
     * This function will automatically save
     * @param key String to be used as a key in the configuration
     * @param value Value
     */
    public void set(String key, Object value) {
        this.map.put(key, value);
        save();
    }


    /**
     * Unsets a configuration value
     * @param key Key to unset
     */
    public void unset(String key) {
        this.map.remove(key);
        save();
    }

    /**
     * Gets a configuration value
     * @param key configuration value to get
     */
    public Object get(String key) {
        return this.map.get(key);
    }

    /**
     * Loads the configuration from the JSON file
     * Note that if no configuration file exists, a new one will be created automatically.
     */
    public void load() {
        File file = AppUtil.getInstance().getSettingsFile(filename);
        clear(); // Note: loading from file will clear the map
        // Because append calls save, if the json does not exist, it will be created.
        append(gson.fromJson(ReadWriteFile.readFile(file), Map.class));
    }

    /**
     * Saves the configuration to the JSON file
     */
    public void save() {
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, gson.toJson(map));
    }
}
