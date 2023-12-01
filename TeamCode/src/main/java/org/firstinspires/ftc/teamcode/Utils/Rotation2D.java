package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import java.util.Map;

public class Rotation2D extends Transformation2D {
    private final double radian;
    public Rotation2D(double radian, Map<String, Object>debugMessages) {
        super();
        radian = AngleUtils.simplifyAngle(radian);
        double[] iHat = { Math.cos(radian), Math.sin(radian) };
        double[] jHat = { Math.cos(radian + Math.PI / 2), Math.sin(radian + Math.PI / 2) };
        super.setIHat(iHat);
        super.setJHat(jHat);
        debugMessages.put("ihat when created", iHat[0] + "," +iHat[1]);
        debugMessages.put("jhat when created", jHat[0] + "," +jHat[1]);

        this.radian =radian;
    }
    public Rotation2D(double radian) {
        super();
        radian = AngleUtils.simplifyAngle(radian);
        double[] iHat = { Math.cos(radian), Math.sin(radian) };
        double[] jHat = { Math.cos(radian + Math.PI / 2), Math.sin(radian + Math.PI / 2) };
        super.setIHat(iHat);
        super.setJHat(jHat);

        this.radian =radian;
    }

    public double getRadian() {
        return radian;
    }

    @NonNull
    @Override
    public String toString() {
        return "rotation with radian: " + this.getRadian() + "\nand vector value: " +  super.toString();
    }
}

