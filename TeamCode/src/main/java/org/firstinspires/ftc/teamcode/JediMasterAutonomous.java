package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.teamcode.PositionAndHeading.FIXED;
import static org.firstinspires.ftc.teamcode.PositionAndHeading.IMU;
import static org.firstinspires.ftc.teamcode.PositionAndHeading.VUFORIA;

@Autonomous(name = "JediMasterAutonomous (Blocks to Java)")
public class JediMasterAutonomous extends LinearOpMode {


    private final PositionAndHeading START_LINE_1 = new PositionAndHeading(-69.5, 47.75, 0,0);
    private final PositionAndHeading START_LINE_2 = new PositionAndHeading(-69.5, 21.5, 0,0);
    private final PositionAndHeading TARGET_ZONE_A = new PositionAndHeading(13, 58, 0,0);
    private final PositionAndHeading TARGET_ZONE_B = new PositionAndHeading(34, 35, 0,0);
    private final PositionAndHeading TARGET_ZONE_C = new PositionAndHeading(58, 58, 0,0);

    private Servo intakeLift;
    private Servo dumpBed;
    private DcMotorEx lfMotor;
    private DcMotorEx rfMotor;
    private DcMotorEx lrMotor;
    private DcMotorEx rrMotor;
    private BNO055IMU imu;
    private Rev2mDistanceSensor proximitySensor;

    private Robot robot;
    private ObjectDetector ringDetector;

    double startLine = 1; //default to the first start line
    double targetZone = 1; //if countTheRings doesn't see anything, the value is target zone 1
    PositionAndHeading startLineCoordinates = START_LINE_1;
    PositionAndHeading targetZoneCoordinates = TARGET_ZONE_A;

    // Instance variables so we can display them on the dashboard
    double desiredPolarHeading;
    double delta;
    double deltaThreshold;

    double leftFrontMotorPower;
    double leftRearMotorPower;
    double rightFrontMotorPower;
    double rightRearMotorPower;

    double distanceInTicks;
    int leftFrontTargetPosition;
    int leftRearTargetPosition;
    int rightFrontTargetPosition;
    int rightRearTargetPosition;
    double ticksPerInch;

    double robotSpeed;
    double leftSpeed;
    double rightSpeed;

    double turnSpeed;
    double correctionSpeed;
    double holdSpeed;
    double holdTime;

    // Maximum amount of ticks/second.
    private int maximumRobotTps = 2350;
    private double speedAdjust = 0.02;

    private PositionAndHeading lastKnownPositionAndHeading = new PositionAndHeading();
    private PositionAndHeading tower = new PositionAndHeading(69,36,0,0);

    private void AllSix() {
        //Looks like we are using this after all
        if (startLine == 1) {
            if (targetZone == 1) {
                //1a
                drive(53);
                turn(25);
                drive(17);
                drive(-17);
                turn(0);
                strafe(18);
                hold(0);
                drive(21);
                hold(0);
            } else if (targetZone == 2) {
                //1b
                drive(80);
                turn(-30);
                drive(20);
                drive(-20);
                turn(0);
                drive(-3);
                strafe(18);
                hold(0);
            } else {
                //1c
                drive(100);
                turn(25);
                drive(20);
                drive(-20);
                turn(0);
                strafe(18);
                hold(0);
                drive(-20);
                hold(0);
            }
        } else {
            if (targetZone == 1) {
                //2a
                drive(53);
                turn(60);
                drive(36);
                drive(-36);
                turn(0);
                drive(25);
                hold(0);
            } else if (targetZone == 2) {
                //2b
                drive(80);
                turn(30);
                drive(18);
                drive(-18);
                turn(0);
                drive(-5);
                hold(0);
            } else {
                //2c
                drive(100);
                turn(60);
                drive(36);
                drive(-36);
                turn(0);
                drive(-25);
                hold(0);
            }
        }
    }

    private void navigationProbe(final double maximumDistance){
        debug("navigationProbe is called");
        // Drive should be straight along the heading
        desiredPolarHeading = getImuHeading();
        debug("Heading " + desiredPolarHeading);

        // Makes sure we're in Encoder Mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double maximumDistanceInTicks = maximumDistance * ticksPerInch;

        int LfMotorMaximumTicks = (int) (lfMotor.getCurrentPosition() + maximumDistanceInTicks);
        int LrMotorMaximumTicks = (int) (lrMotor.getCurrentPosition() + maximumDistanceInTicks);
        int RfMotorMaximumTicks = (int) (rfMotor.getCurrentPosition() + maximumDistanceInTicks);
        int RrMotorMaximumTicks = (int) (rrMotor.getCurrentPosition() + maximumDistanceInTicks);

        leftSpeed = robotSpeed;
        rightSpeed = robotSpeed;
        powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        debug("Motors On");

        double priorDelta = 0.0;

        while (opModeIsActive() && !(lfMotor.getCurrentPosition() > LfMotorMaximumTicks ||
                lrMotor.getCurrentPosition() > LrMotorMaximumTicks ||
                rfMotor.getCurrentPosition() > RfMotorMaximumTicks ||
                rrMotor.getCurrentPosition() > RrMotorMaximumTicks ||
                lastKnownPositionAndHeading.valueSource == VUFORIA ||
                proximitySensor.getDistance(DistanceUnit.INCH) < 6)) {

            debug("Loop started");
            priorDelta = adjustSpeed(112, desiredPolarHeading, priorDelta);
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while driving:
            telemetryDashboard("Navigation Probe");

            if(!opModeIsActive()) {
                throw new EmergencyStopException("Navigation Probe");
            }
        }
        // Stop the robot
        debug("End of loop");
        powerTheWheels(0, 0, 0, 0);

        if(!opModeIsActive()) {
            throw new EmergencyStopException("Navigation Probe");
        }
    }

    /**
     * TODO: Add javadoc
     * @param distance
     * @param desiredPolarHeading is in Polar Corrdinates (0* is along the positive x axis).
     * @param priorDelta
     * @return
     */
    private double adjustSpeed(double distance, double desiredPolarHeading, double priorDelta) {
        double currentPolarHeading = getPolarHeading();
        delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        if (Math.abs(delta) >= deltaThreshold) {
            if (Math.abs(delta) - priorDelta > 0) {
                robotSpeed -= speedAdjust;
                if (robotSpeed < 0) {
                    robotSpeed = 0;
                }
            }
            else if (Math.abs(delta) - priorDelta < 0) {
                robotSpeed += speedAdjust;
                if (robotSpeed > 1.0) {
                    robotSpeed = 1.0;
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

    private void driveTo(PositionAndHeading target) {
        // What is the current location and heading? [DONE]

        double xDist = target.xPosition - lastKnownPositionAndHeading.xPosition;
        double yDist = target.yPosition - lastKnownPositionAndHeading.yPosition;

        double targetDist = Math.sqrt(xDist * xDist + yDist * yDist);
        double targetHeading = 0;

        // Based on heading [insert trigonometry here].
        if (xDist == 0 && yDist == 0) {
            // We are already there.
            return;
        }
        else if (xDist > 0 && yDist >= 0) {
            targetHeading = Math.toDegrees(Math.atan(yDist / xDist));
        }
        else if (xDist == 0 && yDist > 0) {
            targetHeading = 90;
        }
        else if (xDist < 0) {
            // it doesn't matter if y is less than, greater than, or equal to zero, the math is the same!
            targetHeading = Math.toDegrees(Math.atan(yDist / xDist)) + 180;
        }
        else if (xDist == 0 && yDist < 0) {
            targetHeading = 270;
        }
        else if (xDist > 0 && yDist < 0) {
            targetHeading = Math.toDegrees(Math.atan(yDist / xDist)) + 360;
        }

        // How far does the robot need to turn.
        targetHeading = normalizeHeading(targetHeading - getImuHeading());

        // turn "A" degrees
        turn(targetHeading);
        telemetryDashboard("Drive To");

        drive(targetDist);

        if (lastKnownPositionAndHeading.valueSource != VUFORIA) {
            // If we don't see a Vuforia target, assume we are at the coordinates we wanted to reach
            lastKnownPositionAndHeading = new PositionAndHeading(target.xPosition, target.yPosition, getImuHeading(), FIXED);
        }
        telemetryDashboard("Drive To");

    }

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

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.setHardwareMap(hardwareMap);
        robot.setCameraAdjustX(11.0f);
        robot.setCameraAdjustY(-4.5f);
        double ticksPerMotorRev;
        double WheelCircumferenceInInches;

        intakeLift = hardwareMap.get(Servo.class, "IntakeLift");
        dumpBed = hardwareMap.get(Servo.class, "Servo1");
        lfMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LR Motor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RR Motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        proximitySensor = hardwareMap.get(Rev2mDistanceSensor.class, "ProximitySensor");

        // Initialize variables
        ticksPerMotorRev = 530.3;
        // Convert 75mm wheel to inches
        WheelCircumferenceInInches = 9.6125;
        ticksPerInch = ticksPerMotorRev / WheelCircumferenceInInches;
        deltaThreshold = 1;
        correctionSpeed = 0.1;
        robotSpeed = 0.5;
        turnSpeed = 0.4;
        holdSpeed = 0.1;
        holdTime = 2000;
        initializeMotors();
        initializeIMU();
        ringDetector = new ObjectDetector();
        try {
            System.out.println("Starting camera initialization");
            ringDetector.init(robot);
        }
        catch (IllegalStateException e) {
            telemetry.addData("ERROR", e.getLocalizedMessage());
            telemetry.addData("Solution", "Diverting to TargetZone 1");
        }
        System.out.println("Vuforia initialized, NOT starting OpenCV next");

        //Make robot legal-size by raising intake
        intakeLift.setPosition(1.0);
        telemetry.addData("Status", "Ready to start - v1.3.5");
        telemetry.addData("ProximitySensor", proximitySensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            try {
                //background.start();   [Thread has other consequences]

                //TODO: make sure we check opModeIsActive
                countTheRings();
                // Put run blocks here.
                intakeLift.setPosition(0.9);
                sleep(1000);
                AllSix();

                // IMU starts to pay attention to where it's going, in case Vuforia doesn't pick up the target.
                Position position = new Position(DistanceUnit.INCH, startLineCoordinates.xPosition, startLineCoordinates.yPosition, 0, System.nanoTime());
                imu.startAccelerationIntegration(position, null, 1);
                navigationProbe(25);
                if (lastKnownPositionAndHeading.valueSource == VUFORIA) {
                    telemetryDashboard("runOpMode");
                    //where are we?
                    hold(0);
                    telemetryDashboard("Square to tower");
                    strafe(lastKnownPositionAndHeading.yPosition - tower.yPosition);
                    telemetryDashboard("Strafe in front");
                    hold(0);
                    telemetryDashboard("Square to tower");
                    drive(tower.xPosition - lastKnownPositionAndHeading.xPosition);
                    dumpBed.setPosition(0);
                    sleep(1000);
                    telemetryDashboard("Dumped rings");
                    drive(-19 - lastKnownPositionAndHeading.xPosition);
                    telemetryDashboard("Parked on line");
                }
                intakeLift.setPosition(0.0);
            }
            catch (EmergencyStopException e) {
                // FORCE QUIT THE PROGRAM RIGHT NOW!!!
                // SHUT IT DOWN!!!
                // OVERRIDE EVERYTHING!!!
                telemetry.addData("Emergency Stop", e.getMessage());
                telemetry.update();
            }
        }
    }

    /**
     * Configure motor direction and modes.
     */
    private void initializeMotors() {
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void debug(String text) {
        System.out.println("Debug" + text);
    }

    private void countTheRings() {
        // look for the rings
        List<Recognition> updatedRecognitions = ringDetector.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            targetZone = 1;
            targetZoneCoordinates = TARGET_ZONE_A;
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                i += 1;
                String label = recognition.getLabel();
                // TODO: Set the appropriate Target Zone

                if (ringDetector.QUAD.equals(label)) {
                    targetZone = 3;
                    targetZoneCoordinates = TARGET_ZONE_C;
                }
                else if (ringDetector.SINGLE.equals(label)) {
                    targetZone = 2;
                    targetZoneCoordinates = TARGET_ZONE_B;
                }

                telemetry.addData(String.format("label (%d)", i), label);
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
            }
        }
        // ringdetector., initOpenCv, if openCv Sees ring
        if(targetZone == 1) {
            ringDetector.initOpenCv();
            telemetry.addData("avg1", ringDetector.getAvg1());
            telemetry.addData("avg2", ringDetector.getAvg2());
            int boxSeen = ringDetector.whichBoxSeen();
            if(boxSeen != 0) {
                targetZone = 2;
                targetZoneCoordinates = TARGET_ZONE_B;
                if(boxSeen == 1) {
                    startLine = 2;
                    startLineCoordinates = START_LINE_2;
                }
                else if(boxSeen == 2) {
                    startLine = 1;
                    startLineCoordinates = START_LINE_1;
                }
            }
            telemetry.addData("StartLine", startLine);
            telemetry.addData("TargetZone", targetZone);
            telemetry.update();
        }
    }

    private void initializeIMU() {
        BNO055IMU.Parameters imuParameters;

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);
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

    private double getPolarHeading() {
         return getImuHeading() + 90;
    }

    private double getPolarHeading(double imuHeading) {
        return imuHeading + 90;
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        lfMotor.setMode(mode);
        lrMotor.setMode(mode);
        rfMotor.setMode(mode);
        rrMotor.setMode(mode);
    }

    private void telemetryDashboard(String method) {
        telemetry.addData(method, "SL: %.0f, TZ: %.0f, Prox: %.1f", startLine, targetZone,
                proximitySensor.getDistance(DistanceUnit.INCH));

        telemetry.addData("Heading", "Desired: %.0f, Current: %.0f, Delta: %.0f",
                desiredPolarHeading, getImuHeading(), delta);

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
                Position position = new Position(DistanceUnit.INCH, xPosition, yPosition, 0, System.nanoTime());
                //Tells the IMU to start paying attention because the IMU is the backup to Vuforia.
                imu.startAccelerationIntegration(position, null, 1);

                telemetry.addData("Visible Target", visibleTarget.targetName);
                telemetry.addData("Vuforia Position, Heading", "(%.1f, %.1f), %.0f",
                        xPosition, yPosition, vuforiaHeading);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                //        vuforiaRoll, vuforiaPitch, vuforiaHeading);
            }
        } else {
            telemetry.addData("Visible Target", "none");
            //IMU takes over
            Position position = imu.getPosition().toUnit(DistanceUnit.INCH);
            double heading = getImuHeading();
            lastKnownPositionAndHeading = new PositionAndHeading(position.x, position.y, heading, IMU);
            telemetry.addData("IMU Position, Heading", "(%.1f, %.1f), %.0f", position.x, position.y,
                    heading);
        }
        telemetry.update();
    }

    /**
     * Drive in a straight line.
     * @param distance How far to move, in inches.
     */
    private void drive(double distance) {
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
        while (opModeIsActive() && lfMotor.isBusy() && lrMotor.isBusy() && rfMotor.isBusy() &&
                rrMotor.isBusy()) {
            debug("Loop started");
            priorDelta = adjustSpeed(distance, desiredPolarHeading, priorDelta);
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while driving:
            telemetryDashboard("Drive(" + distance + ")");
        }

        if(!opModeIsActive()) {
            throw new EmergencyStopException("Drive");
        }

        // Stop the robot
        debug("End of loop");
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        powerTheWheels(0, 0, 0, 0);
    }

    /**
     * Turn turns the robot till it is facing imuHeading.
     * @param imuHeading Heading is in IMU coordinates (0* points to the tower goal).
     */
    private void turn(double imuHeading) {
        desiredPolarHeading = getPolarHeading(imuHeading);

        double currentPolarHeading = getPolarHeading();
        delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        while (opModeIsActive() && Math.abs(delta) > deltaThreshold) {
            if (delta > 0) {
                leftFrontMotorPower = -turnSpeed;
                leftRearMotorPower = -turnSpeed;
                rightFrontMotorPower = turnSpeed;
                rightRearMotorPower = turnSpeed;
            }
            else {
                leftFrontMotorPower = turnSpeed;
                leftRearMotorPower = turnSpeed;
                rightFrontMotorPower = -turnSpeed;
                rightRearMotorPower = -turnSpeed;
            }
            powerTheWheels(leftFrontMotorPower, leftRearMotorPower, rightFrontMotorPower, rightRearMotorPower);
            telemetryDashboard("Turn(" + (int) imuHeading + ")");
            currentPolarHeading = getPolarHeading();
            delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        }

        if(!opModeIsActive()) {
            throw new EmergencyStopException("Turn");
        }

        powerTheWheels(0, 0, 0, 0);
        hold(imuHeading);
    }

    /**
     *
     * @param imuHeading Heading is in IMU coordinates (0* points to the tower goal).
     */
    private void hold(double imuHeading) {
        ElapsedTime Timer;

        desiredPolarHeading = getPolarHeading(imuHeading);
        double currentPolarHeading = getPolarHeading();
        delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer.reset();
        while (opModeIsActive() && Math.abs(delta) > 0.5 && Timer.time() < holdTime) {
            if (delta > 0) {
                leftSpeed = -holdSpeed;
                rightSpeed = holdSpeed;
            } else {
                leftSpeed = holdSpeed;
                rightSpeed = -holdSpeed;
            }
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            sleep(75);
            powerTheWheels(0, 0, 0, 0);
            telemetryDashboard("Hold(" + (int) imuHeading + ")");
            currentPolarHeading = getPolarHeading();
            delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        }

        if(!opModeIsActive()) {
            throw new EmergencyStopException("Hold");
        }

        powerTheWheels(0, 0, 0, 0);
    }

    /**
     * TODO: Javadoc
     * @param distance
     */
    private void strafe(double distance) {
        debug("strafe is called");
        desiredPolarHeading = getPolarHeading();
        setMotorDistanceToTravel(distance, new int[]{-1, 1, 1, -1});
        leftFrontMotorPower = -robotSpeed;
        leftRearMotorPower = robotSpeed;
        rightFrontMotorPower = robotSpeed;
        rightRearMotorPower = -robotSpeed;
        powerTheWheels(leftFrontMotorPower, leftRearMotorPower, rightFrontMotorPower, rightRearMotorPower);
        debug("Motors On");
        double priorDelta = 0.0;
        while (!(isStopRequested() || !lfMotor.isBusy() || !lrMotor.isBusy() || !rfMotor.isBusy()
                || !rrMotor.isBusy())) {
            priorDelta = adjustSpeed(distance, desiredPolarHeading, priorDelta);
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while strafing:
            telemetryDashboard("Strafe(" + distance + ")");
        }

        if(!opModeIsActive()) {
            throw new EmergencyStopException("Hold");
        }

        debug("End of loop");
        powerTheWheels(0, 0, 0, 0);
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        distanceInTicks = distance * ticksPerInch;
        leftFrontTargetPosition = (int) (lfMotor.getCurrentPosition() + distanceInTicks);
        leftRearTargetPosition = (int) (lrMotor.getCurrentPosition() + distanceInTicks);
        rightFrontTargetPosition = (int) (rfMotor.getCurrentPosition() + distanceInTicks);
        rightRearTargetPosition = (int) (rrMotor.getCurrentPosition() + distanceInTicks);

        lfMotor.setTargetPosition(direction[0] * leftFrontTargetPosition);
        lrMotor.setTargetPosition(direction[1] * leftRearTargetPosition);
        rfMotor.setTargetPosition(direction[2] * rightFrontTargetPosition);
        rrMotor.setTargetPosition(direction[3] * rightRearTargetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
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

            lfMotor.setVelocity(lfVelocity);
            lrMotor.setVelocity(lrVelocity);
            rfMotor.setVelocity(rfVelocity);
            rrMotor.setVelocity(rrVelocity);
        }
        else {
            // We assume that we will be using RUN_TO_POSITION mode.

            lfMotor.setPower(lfPower);
            lrMotor.setPower(lrPower);
            rfMotor.setPower(rfPower);
            rrMotor.setPower(rrPower);
        }
    }
}
