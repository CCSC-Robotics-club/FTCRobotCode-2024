package org.firstinspires.ftc.teamcode.Utils;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import java.util.List;

public interface RawObjectDetectionCamera {
    void startRecognizing();
    void stopRecognizing();
    List<PixelTargetRaw> getPixelTargets();

    final class PixelTargetRaw {
        public final double x, y;
        public PixelTargetRaw(double x, double y) {
            this.x = x;
            this.y = y;
        }

        @SuppressLint("DefaultLocale")
        @NonNull
        @Override
        public String toString() {
            return String.format("pixel target at (%.2f, %.2f)", x, y);
        }
    }
}
