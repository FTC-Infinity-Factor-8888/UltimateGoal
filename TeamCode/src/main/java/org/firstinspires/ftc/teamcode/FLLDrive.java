package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FLLDrive{
    private UltimateGoalRobot creator;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx lfMotor;
    private DcMotorEx rfMotor;
    private DcMotorEx lrMotor;
    private DcMotorEx rrMotor;
    private BNO055IMU imu;
    private Rev2mDistanceSensor proximitySensor;

    private double ticksPerMotorRev = 530.3;
    private double wheelCircumferenceInInches = 10.0625;
    private double ticksPerInch = ticksPerMotorRev / wheelCircumferenceInInches;

    private void setMotorMode(DcMotor.RunMode mode) {
        lfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rfMotor.setMode(mode);
        rrMotor.setMode(mode);
    }

    private void initializeMotors() {
        lfMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LR Motor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RR Motor");

        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initializeIMU() {
        BNO055IMU.Parameters imuParameters;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
    }

    public void init() {
        initializeMotors();
        initializeIMU();
        proximitySensor = hardwareMap.get(Rev2mDistanceSensor.class, "ProximitySensor");
    }

    private double getImuHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        // Add 90 degrees, because we want to match a polar coordinate system, and Vuforia
        return angles.firstAngle;
    }

    public Rev2mDistanceSensor getProximitySensor() {
        return proximitySensor;
    }

    private void setMotorDistanceToTravel(double distance, int[] direction) {
        if(direction.length != 4) {
            throw new IllegalArgumentException("You must provide an array with exactly 4 elements!");
        }
        for(int i = 0; i < 4; i++) {
            if(direction[i] > 1 || direction[i] < -1) {
                throw new IllegalArgumentException("Elements must be -1, 0, or 1.");
            }
        }

        double distanceInTicks = distance * ticksPerInch;
        int leftFrontTargetPosition = (int) (lfMotor.getCurrentPosition() + distanceInTicks);
        int leftRearTargetPosition = (int) (lrMotor.getCurrentPosition() + distanceInTicks);
        int rightFrontTargetPosition = (int) (rfMotor.getCurrentPosition() + distanceInTicks);
        int rightRearTargetPosition = (int) (rrMotor.getCurrentPosition() + distanceInTicks);

        lfMotor.setTargetPosition(direction[0] * leftFrontTargetPosition);
        lrMotor.setTargetPosition(direction[1] * leftRearTargetPosition);
        rfMotor.setTargetPosition(direction[2] * rightFrontTargetPosition);
        rrMotor.setTargetPosition(direction[3] * rightRearTargetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void fllDrive(double distance) {
        double desiredHeading = getImuHeading();
        setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});

        double accelDecelInches;

        if (distance / 8 < 2) {
            accelDecelInches = 2;
        }
        else {
            accelDecelInches = distance / 8;
        }

        
    }
}
