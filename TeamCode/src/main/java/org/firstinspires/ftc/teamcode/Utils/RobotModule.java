package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TimerTask;

public abstract class RobotModule extends ModulesCommanderMarker {
    private List<ModulesCommanderMarker> owners = new ArrayList<>(1);
    private long previousTime = -1;
    public final String moduleName;
    private final int desiredUpdateFrequency;
    protected boolean enabled = true;
    private long timeAfterLastUpdateTimesCheckMillis;
    private int updatesPerSecond, updateCountDuringThisSecond;
    protected boolean terminated = false;

    public RobotModule(String name) {
        this(name, 50);
    }

    public RobotModule(String name, int desiredUpdateFrequency) {
        this.moduleName = name;
        timeAfterLastUpdateTimesCheckMillis = System.currentTimeMillis();
        updatesPerSecond = updateCountDuringThisSecond = 0;
        this.desiredUpdateFrequency = desiredUpdateFrequency;
    }
    public void gainOwnerShip(ModulesCommanderMarker owner) {
        if (!isOwner(owner))
            owners.add(owner);
    }
    public void clearOwnerShips() {
        owners = new ArrayList<>(1);
    }
    /** if a module or service is its owner, null is always its owner */
    protected boolean isOwner(ModulesCommanderMarker operator) {
        return owners.contains(operator) || operator == null;
    }

    public int updateCount = 0;
    public Runnable getRunnable(ProgramRunningStatusChecker checker) {
        final double periodMS = 1000.0f / desiredUpdateFrequency;
        return new Runnable() {
            long t = System.currentTimeMillis();
            @Override
            public void run() {
                while (checker.isProgramActive() && (!terminated)) {
                    updateCount++;
                    if (enabled) periodic();
                    while (System.currentTimeMillis() - t + 1 < periodMS)
                        try {
                            Thread.sleep(2);
                        } catch (InterruptedException ignored) {}
                    t = System.currentTimeMillis();
                }
            }
        };
    }

    public abstract void init();

    public void periodic() {
        if (previousTime == -1) previousTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();

        periodic((currentTime-previousTime)/1000.0f);

        if ((timeAfterLastUpdateTimesCheckMillis += (currentTime - previousTime)) > 1000) {
            this.updatesPerSecond = updateCountDuringThisSecond;
            timeAfterLastUpdateTimesCheckMillis = this.updateCountDuringThisSecond = 0;
        }

        updateCountDuringThisSecond++;
        this.previousTime = currentTime;
    }
    /**
     * calls periodically
     * @param dt the difference in time between two calls, in seconds
     */
    protected abstract void periodic(double dt);
    protected abstract void onDestroy();
    public void terminate() {
        this.onDestroy();
        this.terminated = true;
    }
    public abstract void reset();
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public int getDesiredUpdateFrequency() {
        return this.desiredUpdateFrequency;
    }

    public Map<String, Object> getDebugMessages() {
        return new HashMap<>(0);
    }

    public int getUpdateCountPerSecond() {
        return updatesPerSecond;
    }

    @NonNull
    @Override
    public String toString() {
        return "[Robot Module <" + moduleName + ">]";
    }
}
