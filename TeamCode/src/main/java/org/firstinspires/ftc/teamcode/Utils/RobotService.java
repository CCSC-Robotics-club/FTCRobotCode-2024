package org.firstinspires.ftc.teamcode.Utils;

import java.util.HashMap;
import java.util.Map;
import java.util.TimerTask;

public abstract class RobotService extends ModulesCommanderMarker {
    private long previousTime = -1;
    public abstract void init();

    public void periodic() {
        if (previousTime == -1) previousTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        periodic((currentTime-previousTime)/1000.0f);
        this.previousTime = currentTime;
    }
    public abstract void periodic(double dt);
    public abstract void onDestroy();
    public abstract void reset();

    public Map<String, Object> getDebugMessages() {
        return new HashMap<>(0);
    }
}
