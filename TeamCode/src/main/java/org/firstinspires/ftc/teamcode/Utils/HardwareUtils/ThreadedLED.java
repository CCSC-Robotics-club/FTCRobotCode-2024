package org.firstinspires.ftc.teamcode.Utils.HardwareUtils;

import com.qualcomm.robotcore.hardware.LED;

public class ThreadedLED extends ThreadedSensor {
    private final LED led;
    public ThreadedLED(LED led) {
        super(() -> 0);
        this.led = led;
    }

    @Override
    public void update() {
        this.led.enable(super.enabled);
    }

    @Override
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
