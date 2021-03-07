package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    private HardwareMap hardwareMap;
    //CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT
    private float cameraForwardDisplacement;
    private float cameraLeftDisplacement;
    private float cameraVerticalDisplacement;

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
}

