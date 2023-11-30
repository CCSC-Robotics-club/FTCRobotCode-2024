package org.firstinspires.ftc.teamcode.Utils;

public interface Claw {
    void close();
    void open();
    boolean isClosed();
    final class ServoProfile {
        public final double openValue, closeValue;
        public ServoProfile(double openValue, double closeValue) {
            this.openValue = openValue;

            this.closeValue = closeValue;
        }
    }
}
