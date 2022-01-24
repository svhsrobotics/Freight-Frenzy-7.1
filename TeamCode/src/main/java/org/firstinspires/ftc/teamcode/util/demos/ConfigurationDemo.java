package org.firstinspires.ftc.teamcode.util.demos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.teamcode.util.Configurator;

@TeleOp(name = "Configuration Demo", group = "Software Demos")
@Disabled
public class ConfigurationDemo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Load the configuration from the JSON file, or create a blank one if it does not exist
        Configuration config = Configurator.load(); // Note the use of Configurat*or* not Configuration

        // Check if the value you want is null (meaning it was never set before)
        if (config.target == null) {
            // Do something sensible?
            // We can set the value like this:
            //config.target = new HSVColor(1.0,1.0,1.0);
        }

        // Save the config when you're done modifying it.
        // No need to save if you only read from it, though.
        Configurator.save(config);

        // NOTE:
        // If you want to add new configuration values, you need to modify the Configuration class.
    }
}
