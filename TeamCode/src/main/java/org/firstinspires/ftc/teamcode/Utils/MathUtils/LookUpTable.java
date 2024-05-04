package org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class LookUpTable {
    public final double[] xValues;
    public final double[] yValues;
    final int n;

    public LookUpTable(double[] xValues, double[] yValues) {
        if (xValues.length != yValues.length)
            throw new IllegalArgumentException("look up table length not match");

        this.xValues = xValues;
        this.yValues = yValues;
        this.n = xValues.length;
    }

    public double getYPrediction(double x) {
        final double
                xLowerBound = Math.min(xValues[xValues.length-1], xValues[0]),
                xUpperBound = Math.max(xValues[xValues.length-1], xValues[0]);
        x = Math.min(xUpperBound, x);
        x = Math.max(xLowerBound, x);

        for (int i = 0; i < n-1; i++)
            if ((xValues[i] <= x && x <= xValues[i+1]) || (xValues[i+1] <= x && x <= xValues[i]))
                return linearInterpretation(xValues[i], yValues[i], xValues[i+1], yValues[i+1], x);

        System.out.println("Warning, look up table cannot match");
        return 0;
    }

    public static double linearInterpretation(double x1, double y1, double x2, double y2, double x) {
        return y1 + (x-x1) * (y2-y1) / (x2-x1);
    }
}
