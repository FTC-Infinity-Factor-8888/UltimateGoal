package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.teamcode.PositionAndHeading.FIXED;
import static org.firstinspires.ftc.teamcode.PositionAndHeading.IMU;
import static org.firstinspires.ftc.teamcode.PositionAndHeading.VUFORIA;

public class Robot {
    private final static double MIN_ROBOT_SPEED = 0.3;
    private final static double MAX_ROBOT_SPEED = 1.0;
    private final static double SPEED_RANGE = MAX_ROBOT_SPEED - MIN_ROBOT_SPEED;
    private final static double HOLD_TIME = 1000;

    // Maximum amount of ticks/second.
    //Based off of PIDF measurements:
    private int maximumRobotTps = 2500;
    private double robotSpeed = 0.75;
    private double turnSpeed = SPEED_RANGE;
    private double holdSpeed = 0.1;
    private double speedAdjust = 0.08;
    private double correctionSpeed = 0.1;
    private double ticksPerMotorRev = 530.3;
    private int lfMotorMaxTps = 2655;
    private int rfMotorMaxTps = 2650;
    private int lrMotorMaxTps = 2610;
    private int rrMotorMaxTps = 2615;
    private double drivePositionPIDF = 5.0; // 4.5
    private double strafePositionPIDF = 6.5;
    private double wheelCircumferenceInInches = 10.0625;
    private double ticksPerInch = ticksPerMotorRev/ wheelCircumferenceInInches;
    private double leftSpeed;
    private double rightSpeed;
    private double driveAccelerationIncrement = 0.075;
    private double strafeAccelerationIncrement = 0.05;

    private UltimateGoalRobot creator;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx lfMotor;
    private DcMotorEx rfMotor;
    private DcMotorEx lrMotor;
    private DcMotorEx rrMotor;
    private BNO055IMU imu;
    private Rev2mDistanceSensor proximitySensor;
    private Servo intakeLift;
    private Servo dumpBed;
    private ObjectDetector ringDetector;

    // Instance variables so we can display them on the dashboard
    double desiredPolarHeading;
    double delta;
    double deltaThreshold = 1;

    int targetZone = 1; //if countTheRings doesn't see anything, the value is target zone 1

    private PositionAndHeading lastKnownPositionAndHeading = new PositionAndHeading();

    //CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT
    private float cameraForwardDisplacement;
    private float cameraLeftDisplacement;
    private float cameraVerticalDisplacement;

    /**
     * These variables are used to adjust the x and y position of the camera in relation to the robot.
     */
    private float cameraAdjustX;
    private float cameraAdjustY;

    // A robot without a creator makes no sense!
    private Robot() {}

    public Robot(UltimateGoalRobot creator) {
      this.creator = creator;
      this.telemetry = creator.telemetry;
    }

    public void init() {
        initializeMotors();
        initializeIMU();
        proximitySensor = hardwareMap.get(Rev2mDistanceSensor.class, "ProximitySensor");
        intakeLift = hardwareMap.get(Servo.class, "IntakeLift");
        dumpBed = hardwareMap.get(Servo.class, "Servo1");
        ringDetector = new ObjectDetector();

        try {
            System.out.println("Starting camera initialization");
            ringDetector.init(this);
        }
        catch (IllegalStateException e) {
            telemetry.addData("ERROR", e.getLocalizedMessage());
            telemetry.addData("Solution", "Diverting to TargetZone 1");
        }
        //Make robot legal-size by raising intake
        intakeLift.setPosition(1.0);
    }

    public Rev2mDistanceSensor getProximitySensor() {
        return proximitySensor;
    }

    public Servo getIntakeLift() {
        return intakeLift;
    }

    public Servo getDumpBed() {
        return dumpBed;
    }

    //TODO: JavaDoc
    public int findTargetZone() {
        // look for the rings
        List<Recognition> updatedRecognitions = ringDetector.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            targetZone = 1;
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                i += 1;
                String label = recognition.getLabel();

                if (ringDetector.QUAD.equals(label)) {
                    targetZone = 3;
                } else if (ringDetector.SINGLE.equals(label)) {
                    targetZone = 2;
                }

                telemetry.addData(String.format("label (%d)", i), label);
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
            }
        }
        // ringdetector., initOpenCv, if openCv Sees ring
        if (targetZone == 1) {
            ringDetector.initOpenCv();
            telemetry.addData("avg1", ringDetector.getAvg1());
            telemetry.addData("avg2", ringDetector.getAvg2());
            int boxSeen = ringDetector.whichBoxSeen();
            if (boxSeen != 0) {
                targetZone = 2;
            }
            telemetry.addData("TargetZone", targetZone);
            telemetry.update();
        }
        return targetZone;
    }

    //TODO: JavaDoc
    public int navigationSource() {
        return lastKnownPositionAndHeading.valueSource;
    }

    public double navigationX() {
        return lastKnownPositionAndHeading.xPosition;
    }

    public double navigationY() {
        return lastKnownPositionAndHeading.yPosition;
    }

    public void navigationProbe(final double maximumDistance){
        double speed = MIN_ROBOT_SPEED;
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

        leftSpeed = speed;
        rightSpeed = speed;
        powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        debug("Motors On");

        //double priorDelta = 0.0;

        while (creator.opModeIsActive() && !(lfMotor.getCurrentPosition() > LfMotorMaximumTicks ||
                lrMotor.getCurrentPosition() > LrMotorMaximumTicks ||
                rfMotor.getCurrentPosition() > RfMotorMaximumTicks ||
                rrMotor.getCurrentPosition() > RrMotorMaximumTicks ||
                lastKnownPositionAndHeading.valueSource == VUFORIA ||
                proximitySensor.getDistance(DistanceUnit.INCH) < 6)) {

            debug("Loop started");
            //priorDelta = adjustSpeed(maximumDistance, desiredPolarHeading, priorDelta);
            // TODO: Do same calculations that are in drive.
            leftSpeed = speed;
            rightSpeed = speed;
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while driving:
            telemetryDashboard("Navigation Probe");

            if (speed < robotSpeed) {
                speed += driveAccelerationIncrement;
            }

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

    /**
     * Drives to target.
     * Only goes forward, and turns slowly.
     * @param target The coordinates for where the robot should go.
     */
    public void cautiousDriving(PositionAndHeading target) {
        // What is the current location and heading? [DONE]

        double xDist = target.xPosition - lastKnownPositionAndHeading.xPosition;
        double yDist = target.yPosition - lastKnownPositionAndHeading.yPosition;

        double targetDist = Math.sqrt(xDist * xDist + yDist * yDist);
        double targetImuHeading = 0;

        // Based on heading [insert trigonometry here].
        if (xDist == 0 && yDist == 0) {
            // We are already there.
            return;
        }
        else if (xDist > 0 && yDist >= 0) {
            targetImuHeading = Math.toDegrees(Math.atan(yDist / xDist));
        }
        else if (xDist == 0 && yDist > 0) {
            targetImuHeading = 90;
        }
        else if (xDist < 0) {
            // it doesn't matter if y is less than, greater than, or equal to zero, the math is the same!
            targetImuHeading = Math.toDegrees(Math.atan(yDist / xDist)) + 180;
        }
        else if (xDist == 0 && yDist < 0) {
            targetImuHeading = 270;
        }
        else if (xDist > 0 && yDist < 0) {
            targetImuHeading = Math.toDegrees(Math.atan(yDist / xDist)) + 360;
        }

        // How far does the robot need to turn.
        targetImuHeading = normalizeHeading(targetImuHeading - getImuHeading());

        // turn "A" degrees
        turn(targetImuHeading);
        telemetryDashboard("Drive To");

        drive(targetDist);

        if (lastKnownPositionAndHeading.valueSource != VUFORIA) {
            // If we don't see a Vuforia target, assume we are at the coordinates we wanted to reach
            lastKnownPositionAndHeading = new PositionAndHeading(target.xPosition, target.yPosition, getImuHeading(), FIXED);
        }
        telemetryDashboard("Drive To");

    }


    /**
     * Drive in a straight line.
     * @param distance How far to move, in inches.
     */
    public void driveOg(double distance) {
        debug("Drive is called to go " + distance + " inches.");
        // Drive should be straight along the heading
        desiredPolarHeading = getPolarHeading();
        debug("PolarHeading " + desiredPolarHeading);
        setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});
        leftSpeed = robotSpeed;
        rightSpeed = robotSpeed;
        powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        telemetryDashboard("Drive(" + distance + ")");

        debug("Motors On");
        double priorDelta = 0.0;
        while (creator.opModeIsActive() && lfMotor.isBusy() && lrMotor.isBusy() && rfMotor.isBusy() &&
                rrMotor.isBusy()) {
            debug("Loop started");
            priorDelta = adjustSpeed(distance, desiredPolarHeading, priorDelta);
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while driving:
            telemetryDashboard("Drive(" + distance + ")");
        }
        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("Drive");
        }
        // Stop the robot
        debug("End of loop");
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
    }

    private boolean motorsShouldContinue(double distance) {
        boolean motorsAreBusy = lfMotor.isBusy() && rfMotor.isBusy() && lrMotor.isBusy() && rrMotor.isBusy();
        boolean aMotorHasPassedPosition = false;
        if (motorsAreBusy) {
            aMotorHasPassedPosition = checkMotorPosition(lfMotor, distance) || checkMotorPosition(rfMotor, distance) ||
                    checkMotorPosition(lrMotor, distance) || checkMotorPosition(rrMotor, distance);
        }
        return motorsAreBusy && !aMotorHasPassedPosition;
    }

    private boolean checkMotorPosition(DcMotorEx motor, double distance) {
        if (distance > 0) {
            return motor.getCurrentPosition() > motor.getTargetPosition();
        }
        return motor.getCurrentPosition() < motor.getTargetPosition();
    }

    public void drive(double distance) {
        double desiredHeading = getImuHeading();
        setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});
        double speed = MIN_ROBOT_SPEED;
        if (distance < 0) {
            speed *= -1;
        }
        leftSpeed = speed;
        rightSpeed = speed;
        powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        telemetryDashboard("Drive(" + distance + ")");
        while (creator.opModeIsActive() && motorsShouldContinue(distance)) {
            double imuHeading = getImuHeading();
            delta = normalizeHeading(desiredHeading - imuHeading);
            double adjustSpeed = 0;
            if (Math.abs(delta) > deltaThreshold) {
                adjustSpeed = correctionSpeed;
                if (delta > 0) {
                    adjustSpeed *= -1;
                }
            }
            leftSpeed = speed + adjustSpeed;
            rightSpeed = speed - adjustSpeed;
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            telemetryDashboard("Drive(" + distance + ")");

            if (speed < robotSpeed) {
                speed += driveAccelerationIncrement;
            }
        }
        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("Drive");
        }
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
    }

    /**
     * Turn turns the robot till it is facing imuHeading.
     * @param imuHeading Heading is in IMU coordinates (0* points to the tower goal).
     */
    public void turnOg(double imuHeading) {
        desiredPolarHeading = getPolarHeading(imuHeading);

        double priorDelta = 0;
        double currentPolarHeading = getPolarHeading();
        delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        while (creator.opModeIsActive() && Math.abs(delta) > deltaThreshold) {
            if (delta - priorDelta > 0) {
                turnSpeed -= speedAdjust;
                if (turnSpeed < MIN_ROBOT_SPEED) {
                    turnSpeed = MIN_ROBOT_SPEED;
                }
            }
            else {
                turnSpeed += speedAdjust;
                if (turnSpeed > SPEED_RANGE) {
                    turnSpeed = SPEED_RANGE;
                }
            }

            currentPolarHeading = getPolarHeading();
            priorDelta = delta;
            delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);

            double deltaPercentage =  powerPercentage(delta);
            double currentTurnSpeed = turnSpeed * deltaPercentage + MIN_ROBOT_SPEED;
            if (delta < 0) {
                currentTurnSpeed = -currentTurnSpeed;
            }
            leftSpeed = -currentTurnSpeed;
            rightSpeed = currentTurnSpeed;
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            telemetryDashboard("Turn(" + (int) imuHeading + ")");
        }

        if(!creator.opModeIsActive()) {
            throw new EmergencyStopException("Turn");
        }

        powerTheWheels(0, 0, 0, 0);
        hold(imuHeading);
    }

    public void turn(double desiredHeading) {
        double minTurnSpeed = 0.1;
        double currentHeading = getImuHeading();
        delta = normalizeHeading(desiredHeading - currentHeading);
        while (creator.opModeIsActive() && Math.abs(delta) > deltaThreshold) {
            currentHeading = getImuHeading();
            delta = normalizeHeading(desiredHeading - currentHeading);
            double deltaPercentage =  powerPercentage(delta);
            double currentTurnSpeed = turnSpeed * deltaPercentage + minTurnSpeed;
            if (delta < 0) {
                currentTurnSpeed = -currentTurnSpeed;
            }
            leftSpeed = -currentTurnSpeed;
            rightSpeed = currentTurnSpeed;
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            telemetryDashboard("Turn(" + (int) desiredHeading + ")");
        }
        if(!creator.opModeIsActive()) {
            throw new EmergencyStopException("Turn");
        }

        powerTheWheels(0, 0, 0, 0);
        hold(desiredHeading);
    }

    /**
     *
     * @param imuHeading Heading is in IMU coordinates (0* points to the tower goal).
     */
    public void holdOg(double imuHeading) {
        ElapsedTime timer;

        desiredPolarHeading = getPolarHeading(imuHeading);
        double currentPolarHeading = getPolarHeading();
        delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        while (creator.opModeIsActive() && Math.abs(delta) > 0.5 && timer.time() < HOLD_TIME) {
            if (delta > 0) {
                leftSpeed = -holdSpeed;
                rightSpeed = holdSpeed;
            } else {
                leftSpeed = holdSpeed;
                rightSpeed = -holdSpeed;
            }
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            creator.sleep(75);
            powerTheWheels(0, 0, 0, 0);
            telemetryDashboard("Hold(" + (int) imuHeading + ")");
            currentPolarHeading = getPolarHeading();
            delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        }

        if(!creator.opModeIsActive()) {
            throw new EmergencyStopException("Hold");
        }

        powerTheWheels(0, 0, 0, 0);
    }

    public void hold(double desiredHeading) {
        ElapsedTime timer;

        double currentHeading = getImuHeading();
        delta = normalizeHeading(desiredHeading - currentHeading);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        while (creator.opModeIsActive() && Math.abs(delta) > 0.5 && timer.time() < HOLD_TIME) {
            if (delta > 0) {
                leftSpeed = -holdSpeed;
                rightSpeed = holdSpeed;
            } else {
                leftSpeed = holdSpeed;
                rightSpeed = -holdSpeed;
            }
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            creator.sleep(75);
            powerTheWheels(0, 0, 0, 0);
            telemetryDashboard("Hold(" + (int) desiredHeading + ")");
            currentHeading = getImuHeading();
            delta = normalizeHeading(desiredHeading - currentHeading);
        }

        if(!creator.opModeIsActive()) {
            throw new EmergencyStopException("Hold");
        }

        powerTheWheels(0, 0, 0, 0);
    }

    /**
     * TODO: Javadoc
     * @param distance
     */
    public void strafeOg(double distance) {
        debug("strafe is called");
        desiredPolarHeading = getPolarHeading();
        setMotorDistanceToTravel(distance, new int[]{-1, 1, 1, -1});
        leftSpeed = robotSpeed;
        rightSpeed = robotSpeed;
        powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
        telemetryDashboard("Strafe(" + distance + ")");

        debug("Motors On");
        double priorDelta = 0.0;
        while (creator.opModeIsActive() && lfMotor.isBusy() && lrMotor.isBusy() && rfMotor.isBusy() &&
                rrMotor.isBusy()) {
            priorDelta = adjustSpeed(distance, desiredPolarHeading, priorDelta);
            powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
            // Show motor power while strafing:
            telemetryDashboard("Strafe(" + distance + ")");
        }

        if(!creator.opModeIsActive()) {
            throw new EmergencyStopException("Strafe");
        }

        debug("End of loop");
        powerTheWheels(0, 0, 0, 0);
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe(double distance) {
        double strafeSlippage = 1.1;
        double desiredHeading = getImuHeading();
        lfMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        rfMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        lrMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        rrMotor.setPositionPIDFCoefficients(strafePositionPIDF);
        setMotorDistanceToTravel(distance * strafeSlippage, new int[]{-1, 1, 1, -1});
        double speed = MIN_ROBOT_SPEED;
        leftSpeed = speed;
        rightSpeed = -speed;
        powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
        telemetryDashboard("Strafe(" + distance + ")");
        while (creator.opModeIsActive() && lfMotor.isBusy() && rfMotor.isBusy() && lrMotor.isBusy() && rrMotor.isBusy()) {
            double imuHeading = getImuHeading();
            delta = normalizeHeading(desiredHeading - imuHeading);
            double adjustSpeed = 0;
            if (Math.abs(delta) > deltaThreshold) {
                adjustSpeed = correctionSpeed;
                if (delta > 0) {
                    adjustSpeed *= -1;
                }
            }
            leftSpeed = speed + adjustSpeed;
            rightSpeed = -speed + adjustSpeed;
            powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
            telemetryDashboard("Strafe(" + distance + ")");

            if (speed < robotSpeed) {
                speed += driveAccelerationIncrement;
            }
        }
        if (!creator.opModeIsActive()) {
            throw new EmergencyStopException("Strafe");
        }
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
        lfMotor.setPositionPIDFCoefficients(drivePositionPIDF);
        rfMotor.setPositionPIDFCoefficients(drivePositionPIDF);
        lrMotor.setPositionPIDFCoefficients(drivePositionPIDF);
        rrMotor.setPositionPIDFCoefficients(drivePositionPIDF);
    }

    // TODO: JavaDoc
    public void telemetryDashboard(String method) {
        telemetry.addData(method, "SL: %d, TZ: %d, Prox: %.1f", creator.getStartLine(), targetZone,
                proximitySensor.getDistance(DistanceUnit.INCH));

        telemetry.addData("Heading", "Desired: %.0f, Current: %.0f, Delta: %.0f",
                getImuHeading(), getImuHeading(), delta);

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

    /**
     * Configure motor direction and modes.
     */
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

        System.out.print("LF: ");
        setPIDFValues(lfMotor, lfMotorMaxTps);
        System.out.print("RF: ");
        setPIDFValues(rfMotor, rfMotorMaxTps);
        System.out.print("LR: ");
        setPIDFValues(lrMotor, lrMotorMaxTps);
        System.out.print("RR: ");
        setPIDFValues(rrMotor, rrMotorMaxTps);
    }

    private void setPIDFValues(DcMotorEx motor, int tps) {
            double D = 0;
            double F = 32767.0 / tps;
            double P = 0.1 * F;
            double I = 0.1 * P;
            System.out.printf("Max %d, P %.4f, I %.4f, D %.0f, F %.4f\n", tps, P, I, D, F);
            motor.setVelocityPIDFCoefficients(P, I, D, F);
            motor.setPositionPIDFCoefficients(drivePositionPIDF);
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

    private void setMotorMode(DcMotor.RunMode mode) {
        lfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rfMotor.setMode(mode);
        rrMotor.setMode(mode);
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

    /**
     * Programs all four motors to run to position, based off of distance and direction.
     *
     * @param distance The distance you want to drive in inches
     * @param direction The direction each motor should turn. It is an array consisting of the
     *                  LfMotor, LrMotor, RfMotor, and RrMotor. The values can be -1 to move backwards,
     *                  1 to move forwards, or 0 to not move the motor at all.
     */
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

    /**
     * TODO: Add javadoc
     * @param distance
     * @param desiredPolarHeading is in Polar Coordinates (0* is along the positive x axis).
     * @param priorDelta
     * @return
     */
    private double adjustSpeed(double distance, double desiredPolarHeading, double priorDelta) {
        double currentPolarHeading = getPolarHeading();
        delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        if (Math.abs(delta) >= deltaThreshold) {
            if (Math.abs(delta) - priorDelta > 0) {
                robotSpeed -= speedAdjust;
                if (robotSpeed < MIN_ROBOT_SPEED) {
                    robotSpeed = MIN_ROBOT_SPEED;
                }
            }
            else if (Math.abs(delta) - priorDelta < 0) {
                robotSpeed += speedAdjust;
                if (robotSpeed > MAX_ROBOT_SPEED) {
                    robotSpeed = MAX_ROBOT_SPEED;
                }
            }

            if (delta > 0) {
                if (distance > 0) {
                    rightSpeed = robotSpeed + correctionSpeed;
                    leftSpeed = robotSpeed - correctionSpeed;
                }
                else {
                    rightSpeed = robotSpeed - correctionSpeed;
                    leftSpeed = robotSpeed + correctionSpeed;
                }
            }
            else {
                if (distance > 0) {
                    rightSpeed = robotSpeed - correctionSpeed;
                    leftSpeed = robotSpeed + correctionSpeed;
                } else {
                    rightSpeed = robotSpeed + correctionSpeed;
                    leftSpeed = robotSpeed - correctionSpeed;
                }
            }
        }
        return delta;
    }

    //TODO: JavaDoc
    private double powerPercentage(double delta) {
        double powerPercent = -0.000027 * Math.pow(Math.abs(delta) - 180, 2) + 1;
        if (powerPercent > 1 || powerPercent < 0) {
            System.out.println("*** WARNING! POWER PERCENT IS OUT OF RANGE: delta = " + delta + ", " +
                    "powerPercent = " + powerPercent + " ***");
        }

        return powerPercent;
    }

    //TODO: Add JavaDoc
    private void powerTheWheels(double lfPower, double lrPower, double rfPower, double rrPower) {
        double leftMax = Math.max(Math.abs(lfPower), Math.abs(lrPower));
        double rightMax = Math.max(Math.abs(rfPower), Math.abs(rrPower));
        double max = Math.max (leftMax, rightMax);

        if(max > MAX_ROBOT_SPEED) {
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

    //TODO: JavaDoc
    private double normalizeHeading(double heading) {
        while (heading >= 180.0 || heading < -180.0) {
            if (heading >= 180.0) {
                heading -= 360.0;
            }
            else if (heading < -180.0) {
                heading += 360.0;
            }
        }
        return heading;
    }

    private void debug(String text) {
        System.out.println("Debug " + text);
    }

}

