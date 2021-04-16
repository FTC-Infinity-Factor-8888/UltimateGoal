package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

import static org.firstinspires.ftc.teamcode.PositionAndHeading.IMU;
import static org.firstinspires.ftc.teamcode.PositionAndHeading.VUFORIA;

public class Robot {

    // Maximum amount of ticks/second.
    private int maximumRobotTps = 2350;
    private double minimumRobotSpeed = 0.25;
    private double maximumRobotSpeed = 1.0;
    private double speedAdjust = 0.08;
    double ticksPerMotorRev = 530.3;
    // Convert 75mm wheel to inches
    double WheelCircumferanceinMM = 75*Math.PI;
    double WheelCircumferenceInInches = WheelCircumferanceinMM/25.4;
    double ticksPerInch = ticksPerMotorRev/ WheelCircumferenceInInches;

    private LinearOpMode creator;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx lfMotor;
    private DcMotorEx rfMotor;
    private DcMotorEx lrMotor;
    private DcMotorEx rrMotor;
    private BNO055IMU imu;
    private Rev2mDistanceSensor proximitySensor;

    // Instance variables so we can display them on the dashboard
    double desiredPolarHeading;
    double delta;
    double deltaThreshold;

    double robotSpeed = 0.5;

    //CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT
    private float cameraForwardDisplacement;
    private float cameraLeftDisplacement;
    private float cameraVerticalDisplacement;

    /**
     * These variables are used to adjust the x and y position of the camera in relation to the robot.
     */
    private float cameraAdjustX;
    private float cameraAdjustY;

    public Robot() {}
    public Robot(LinearOpMode creator) {
      this.creator = creator;
      this.telemetry = creator.telemetry;
    }

    public void init() {
        proximitySensor = hardwareMap.get(Rev2mDistanceSensor.class, "ProximitySensor");
    }

    /**
     * Configure motor direction and modes.
     */
    public void initializeMotors() {
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

    public void initializeIMU() {
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

    private void setMotorMode(DcMotor.RunMode mode) {
        lfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rfMotor.setMode(mode);
        rrMotor.setMode(mode);
    }

    private void navigationProbe(final double maximumDistance){
        debug("navigationProbe is called");
        // Drive should be straight along the heading
        desiredPolarHeading = getPolarHeading();
        debug("Heading " + desiredPolarHeading);

        // Makes sure we're in Encoder Mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double maximumDistanceInTicks = maximumDistance * ticksPerInch;

        int LfMotorMaximumTicks = (int) (lfMotor.getCurrentPosition() + maximumDistanceInTicks);
        int LrMotorMaximumTicks = (int) (lrMotor.getCurrentPosition() + maximumDistanceInTicks);
        int RfMotorMaximumTicks = (int) (rfMotor.getCurrentPosition() + maximumDistanceInTicks);
        int RrMotorMaximumTicks = (int) (rrMotor.getCurrentPosition() + maximumDistanceInTicks);

        double leftSpeed = robotSpeed;
        double rightSpeed = robotSpeed;
        powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        debug("Motors On");

        double priorDelta = 0.0;

        while (creator.opModeIsActive() && !(lfMotor.getCurrentPosition() > LfMotorMaximumTicks ||
                lrMotor.getCurrentPosition() > LrMotorMaximumTicks ||
                rfMotor.getCurrentPosition() > RfMotorMaximumTicks ||
                rrMotor.getCurrentPosition() > RrMotorMaximumTicks ||
                lastKnownPositionAndHeading.valueSource == VUFORIA ||
                proximitySensor.getDistance(DistanceUnit.INCH) < 6)) {

            debug("Loop started");
            priorDelta = adjustSpeed(maximumDistance, desiredPolarHeading, priorDelta);
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while driving:
            telemetryDashboard("Navigation Probe");

            if(!creator.opModeIsActive()) {
                throw new EmergencyStopException("Navigation Probe");
            }
        }
        // Stop the robot
        debug("End of loop");
        powerTheWheels(0, 0, 0, 0);

        if(!creator.opModeIsActive()) {
            throw new EmergencyStopException("Navigation Probe");
        }
    }

    private void telemetryDashboard(String method) {
        telemetry.addData(method, "SL: %.0f, TZ: %.0f, Prox: %.1f", startLine, targetZone,
                proximitySensor.getDistance(DistanceUnit.INCH));

        telemetry.addData("Heading", "Desired: %.0f, Current: %.0f, Delta: %.0f",
                getImuHeading(desiredPolarHeading), getImuHeading(), delta);

        telemetry.addData("Target", "LF: %d, LR: %d, RF: %d, RR: %d",
                lfMotor.getTargetPosition(), lrMotor.getTargetPosition(), rfMotor.getTargetPosition(), rrMotor.getTargetPosition());
        telemetry.addData("Position", "LF: %d, LR: %d, RF: %d, RR: %d",
                lfMotor.getCurrentPosition(), lrMotor.getCurrentPosition(), rfMotor.getCurrentPosition(), rrMotor.getCurrentPosition());
        telemetry.addData("Power", "LF: %.1f, LR: %.1f, RF: %.1f, RR: %.1f",
                lfMotor.getPower(), lrMotor.getPower(), rfMotor.getPower(), rrMotor.getPower());

        List<NavigationInfo> allVisibleTargets = ringDetector.getNavigationInfo();
        if (allVisibleTargets != null) {
            for (NavigationInfo visibleTarget : allVisibleTargets) {

                float xPosition = visibleTarget.translation.get(0);
                float yPosition = visibleTarget.translation.get(1);
                float zPosition = visibleTarget.translation.get(2);
                float vuforiaRoll = visibleTarget.rotation.firstAngle;
                float vuforiaPitch = visibleTarget.rotation.secondAngle;
                double vuforiaHeading = normalizeHeading(visibleTarget.rotation.thirdAngle);

                lastKnownPositionAndHeading = new PositionAndHeading(xPosition, yPosition, vuforiaHeading, VUFORIA);
                /*
                Position position = new Position(DistanceUnit.INCH, xPosition, yPosition, 0, System.nanoTime());
                //Tells the IMU to start paying attention because the IMU is the backup to Vuforia.
                imu.startAccelerationIntegration(position, null, 1);
                */

                telemetry.addData("Visible Target", visibleTarget.targetName);
                telemetry.addData("Vuforia Position, Heading", "(%.1f, %.1f), %.0f",
                        xPosition, yPosition, vuforiaHeading);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                //        vuforiaRoll, vuforiaPitch, vuforiaHeading);
            }
        } else {
            telemetry.addData("Visible Target", "none");
            /*
            //IMU takes over
            Position position = imu.getPosition().toUnit(DistanceUnit.INCH);
            */
            double imuHeading = getImuHeading();
            // Don't update X & Y; the IMU is too inaccurate
            lastKnownPositionAndHeading.heading = imuHeading;
            lastKnownPositionAndHeading.valueSource = IMU;
            /*
            telemetry.addData("IMU Position, Heading", "(%.1f, %.1f), %.0f", position.x, position.y,
                    heading);
             */
            telemetry.addData("IMU Heading", "%.0f", imuHeading);
        }
        telemetry.update();
    }

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

    private void powerTheWheels(double lfPower, double lrPower, double rfPower, double rrPower) {
        double leftMax = Math.max(Math.abs(lfPower), Math.abs(lrPower));
        double rightMax = Math.max(Math.abs(rfPower), Math.abs(rrPower));
        double max = Math.max (leftMax, rightMax);

        if(max > 1.0) {
            lfPower /= max;
            lrPower /= max;
            rfPower /= max;
            rrPower /= max;
        }

        if (lfMotor.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER)) {
            double lfVelocity = lfPower * maximumRobotTps;
            double lrVelocity = lrPower * maximumRobotTps;
            double rfVelocity = rfPower * maximumRobotTps;
            double rrVelocity = rrPower * maximumRobotTps;

            if (creator.opModeIsActive()) {

                lfMotor.setVelocity(lfVelocity);
                lrMotor.setVelocity(lrVelocity);
                rfMotor.setVelocity(rfVelocity);
                rrMotor.setVelocity(rrVelocity);
            }
            else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        }
        else {
            // We assume that we will be using RUN_TO_POSITION mode.
            if(creator.opModeIsActive()) {
                lfMotor.setPower(lfPower);
                lrMotor.setPower(lrPower);
                rfMotor.setPower(rfPower);
                rrMotor.setPower(rrPower);
            }
            else {
                throw new EmergencyStopException("PowerTheWheels");
            }
        }
    }

    /**
     * Return the robot's current heading, as an angle in degrees,
     * with 90 as the heading at the time of IMU initialization.
     * Angles are positive in a counter-clockwise direction.
     */
    private double getImuHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        // Add 90 degrees, because we want to match a polar coordinate system, and Vuforia
        return angles.firstAngle;
    }

    //ToDo: JavaDoc
    private double getImuHeading(double polarHeading) {
        return polarHeading - 90;
    }

    //ToDo: JavaDoc
    private double getPolarHeading() {
        return getImuHeading() + 90;
    }

    //ToDo: JavaDoc
    private double getPolarHeading(double imuHeading) {
        return imuHeading + 90;
    }

    private void debug(String text) {
        System.out.println("Debug " + text);
    }

}

