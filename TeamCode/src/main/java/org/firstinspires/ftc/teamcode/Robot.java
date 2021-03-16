package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    private HardwareMap hardwareMap;
    //CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT
    private float cameraForwardDisplacement;
    private float cameraLeftDisplacement;
    private float cameraVerticalDisplacement;

    /**
     * These variables are used to adjust the x and y position of the camera in relation to the robot.
     */
    private float cameraAdjustX;
    private float cameraAdjustY;


    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public float getCameraForwardDisplacement() {
        return cameraForwardDisplacement;
    }

    public void setCameraForwardDisplacement(float cameraForwardDisplacement) {
        this.cameraForwardDisplacement = cameraForwardDisplacement;
    }

    public float getCameraLeftDisplacement() {
        return cameraLeftDisplacement;
    }

    public void setCameraLeftDisplacement(float cameraLeftDisplacement) {
        this.cameraLeftDisplacement = cameraLeftDisplacement;
    }

    public float getCameraVerticalDisplacement() {
        return cameraVerticalDisplacement;
    }

    public void setCameraVerticalDisplacement(float cameraVerticalDisplacement) {
        this.cameraVerticalDisplacement = cameraVerticalDisplacement;
    }

    public float getCameraAdjustX() {
        return cameraAdjustX;
    }

    public void setCameraAdjustX(float cameraAdjustX) {
        this.cameraAdjustX = cameraAdjustX;
    }

    public float getCameraAdjustY() {
        return cameraAdjustY;
    }

    public void setCameraAdjustY(float cameraAdjustY) {
        this.cameraAdjustY = cameraAdjustY;
    }
}

