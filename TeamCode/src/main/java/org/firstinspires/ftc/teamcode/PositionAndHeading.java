package org.firstinspires.ftc.teamcode;

public class PositionAndHeading {
    double xPosition;
    double yPosition;
    double heading;
    /**
     * value 0 = not initialized OR fixed point in space
     * value 1 = Vuforia
     * value 2 = IMU
     */
    public final static int NOT_INITIALIZED = 0;
    public final static int VUFORIA = 1;
    public final static int IMU = 2;
    public final static int FIXED = 3;

    int valueSource;

    public PositionAndHeading() {
        // Empty constructor
        // Default values:
        xPosition = 0;
        yPosition = 0;
        heading = 0;
        valueSource = 0;
    }

    public PositionAndHeading(double x, double y, double heading, int valueSource) {
        xPosition = x;
        yPosition = y;
        this.heading = heading;
        this.valueSource = valueSource;
    }
}
