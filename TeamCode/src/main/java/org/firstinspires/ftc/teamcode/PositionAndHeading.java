package org.firstinspires.ftc.teamcode;

public class PositionAndHeading {
    double xPosition;
    double yPosition;
    double vuforiaHeading;

    public PositionAndHeading() {
        // Empty constructor
        // Default values:
        xPosition = 0;
        yPosition = 0;
        vuforiaHeading = 0;
    }

    public PositionAndHeading(double x, double y, double heading) {
        xPosition = x;
        yPosition = y;
        vuforiaHeading = heading;
    }
}
