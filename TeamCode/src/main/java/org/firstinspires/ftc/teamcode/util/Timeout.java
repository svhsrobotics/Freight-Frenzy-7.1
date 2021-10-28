package org.firstinspires.ftc.teamcode.util;

public class Timeout {
    private long startTime;
    private int duration;
    public Timeout(int duration) {
        this.startTime = System.nanoTime();
    }
    public boolean timeout() {
        return (System.nanoTime() - this.startTime) > (this.duration * 1000 * 1000 * 1000);
    }
}
