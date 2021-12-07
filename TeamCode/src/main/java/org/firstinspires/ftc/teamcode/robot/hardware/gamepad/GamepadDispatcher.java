package org.firstinspires.ftc.teamcode.robot.hardware.gamepad;

import static java.util.concurrent.Executors.newSingleThreadExecutor;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

public class GamepadDispatcher {
    private class PollingTask implements Runnable {
        GamepadDispatcher dispatcher = GamepadDispatcher.this;

        @Override
        public void run() {
            while (true) {
                // poll the gamepad
                if (dispatcher.gamepad.a) {
                    dispatcher.press(Key.A);
                }
            }
        }
    }

    private final List<KeyHandler> handlers = new ArrayList<>();
    private final Gamepad gamepad;

    public GamepadDispatcher(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public GamepadDispatcher(String name, HardwareMap hwmap) {
        this.gamepad = hwmap.get(Gamepad.class, name);
    }

    public void add_handler(KeyHandler handler) {
        handlers.add(handler);
    }

    private void press(Key key) {
        for (KeyHandler handler : this.handlers) {
            handler.onPress(key);
        }
    }

    public void start() {
        ExecutorService executor = newSingleThreadExecutor();
        executor.submit(new PollingTask());
    }
}
