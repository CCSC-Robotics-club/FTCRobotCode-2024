package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Servo;

public class DualServoClaw implements Claw {
    private final SingleServoClaw claw1, claw2;
    public DualServoClaw(SingleServoClaw claw1, SingleServoClaw claw2) {
        this.claw1 = claw1;
        this.claw2 = claw2;
    }
    public DualServoClaw(Servo servo1, Servo servo2, ServoProfile servo1Profile, ServoProfile servo2Profile) {
        claw1 = new SingleServoClaw(servo1, servo1Profile);
        claw2 = new SingleServoClaw(servo2, servo2Profile);
    }

    @Override
    public void close() {
        claw1.close();
        claw2.close();
    }

    @Override
    public void open() {
        claw1.open();
        claw2.open();
    }

    @Override
    public boolean isClosed() {
        return claw1.isClosed() && claw2.isClosed();
    }
}
