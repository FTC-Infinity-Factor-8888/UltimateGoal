package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "JediMasterAutonomous (Blocks to Java)")
public class JediMasterAutonomous extends LinearOpMode {


    private final PositionAndHeading TARGET_ZONE_A = new PositionAndHeading(13, 58, 0);
    private final PositionAndHeading TARGET_ZONE_B = new PositionAndHeading(34, 35, 0);
    private final PositionAndHeading TARGET_ZONE_C = new PositionAndHeading(58, 58, 0);

    private Servo intakeLift;
    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LRMotor;
    private DcMotor RRMotor;
    private BNO055IMU imu;

    private Robot robot;
    private ObjectDetector ringDetector;

    boolean robotCanKeepGoing;
    double startLine = 1; //default to the first start line
    double targetZone = 1; //if countTheRings doesn't see anything, the value is target zone 1
    PositionAndHeading targetZoneCoordinates = TARGET_ZONE_A;

    float currentHeading;
    double desiredHeading;
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

    private List<NavigationInfo> lastKnownTargets;
    private PositionAndHeading lastKnownPositionAndHeading;

    /**
     * Describe this function...
     */
    private void AllSix() {
        if (startLine == 1) {
            if (targetZone == 1) {
                //1a
                drive(53);
                turn(25);
                drive(17);
                drive(-17);
                turn(0);
                Strafe(18);
                Hold(0);
                drive(21);
                Hold(0);
            } else if (targetZone == 2) {
                //1b
                drive(80);
                turn(-30);
                drive(20);
                drive(-20);
                turn(0);
                drive(-3);
                Hold(0);
            } else {
                //1c
                drive(100);
                turn(25);
                drive(20);
                drive(-20);
                turn(0);
                Strafe(18);
                Hold(0);
                drive(-20);
                Hold(0);
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
                Hold(0);
            } else if (targetZone == 2) {
                //2b
                drive(80);
                turn(30);
                drive(18);
                drive(-18);
                turn(0);
                drive(-5);
                Hold(0);
            } else {
                //2c
                drive(100);
                turn(60);
                drive(36);
                drive(-36);
                turn(0);
                drive(-25);
                Hold(0);
            }
        }
    }

    private void navigationProbe(final double maximumDistance){
        debug("navigationProbe is called");
        // Drive should be straight along the heading
        double desiredHeading = getHeading();
        debug("Heading " + desiredHeading);

        // Makes sure we're in Encoder Mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double maximumDistanceInTicks = maximumDistance * ticksPerInch;

        int LfMotorMaximumTicks = (int) (LFMotor.getCurrentPosition() + maximumDistanceInTicks);
        int LrMotorMaximumTicks = (int) (LRMotor.getCurrentPosition() + maximumDistanceInTicks);
        int RfMotorMaximumTicks = (int) (RFMotor.getCurrentPosition() + maximumDistanceInTicks);
        int RrMotorMaximumTicks = (int) (RRMotor.getCurrentPosition() + maximumDistanceInTicks);

        leftSpeed = robotSpeed;
        rightSpeed = robotSpeed;
        LFMotor.setPower(leftSpeed);
        LRMotor.setPower(leftSpeed);
        RFMotor.setPower(rightSpeed);
        RRMotor.setPower(rightSpeed);
        debug("Motors On");

        while (!(isStopRequested() || LFMotor.getCurrentPosition() > LfMotorMaximumTicks ||
                LRMotor.getCurrentPosition() > LrMotorMaximumTicks ||
                RFMotor.getCurrentPosition() > RfMotorMaximumTicks ||
                RRMotor.getCurrentPosition() > RrMotorMaximumTicks || lastKnownTargets != null)) {

            debug("Loop started");
            currentHeading = getHeading();
            delta = desiredHeading - currentHeading;
            if (Math.abs(delta) >= deltaThreshold) {
                if (delta > 0) {
                    rightSpeed = robotSpeed + correctionSpeed;
                    leftSpeed = robotSpeed - correctionSpeed;
                }
                else {
                    rightSpeed = robotSpeed - correctionSpeed;
                    leftSpeed = robotSpeed + correctionSpeed;
                }

            }
            PowerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while driving:
            telemetryDashboard();
        }
        // Stop the robot
        debug("End of loop");
        PowerTheWheels(0, 0, 0, 0);

        if(!opModeIsActive()) {
            requestOpModeStop();
        }
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
        targetHeading = normalizeHeading(targetHeading - currentHeading);

        // turn "A" degrees
        turn(targetHeading);

        // How far does the robot need to drive + heading correction?
        if (targetDist <  30) {
            drive(targetDist);
        }
        else {
            drive(targetDist / 2);
            driveTo(target);
        }

        // Drive(distance, heading);
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
        LFMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        RFMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        LRMotor = hardwareMap.get(DcMotor.class, "LR Motor");
        RRMotor = hardwareMap.get(DcMotor.class, "RR Motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize variables
        robotCanKeepGoing = true;
        ticksPerMotorRev = 530.3;
        // Convert 75mm wheel to inches
        WheelCircumferenceInInches = 9.6125;
        ticksPerInch = ticksPerMotorRev / WheelCircumferenceInInches;
        deltaThreshold = 1;
        correctionSpeed = 0.1;
        robotSpeed = 0.5;
        turnSpeed = 0.3;
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
        //ringDetector.initOpenCv();

        //Make robot legal-size by raising intake
        intakeLift.setPosition(1.0);
        telemetry.addData("Status", "Ready to start - v1.3.5");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //background.start();   [Thread has other consequences]

            //TODO: make sure we check opModeIsActive
            countTheRings();
            // Put run blocks here.
            intakeLift.setPosition(0.9);
            sleep(1000);
            //AllSix();
            navigationProbe(112);
            driveTo(targetZoneCoordinates);
            intakeLift.setPosition(0.0);
        }
        currentHeading = getHeading();
        telemetry.addData("Current Heading", currentHeading);
        telemetry.update();
        sleep(5000);
    }

    /**
     * Configure motor direction and modes.
     */
    private void initializeMotors() {
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
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
                }
                else if(boxSeen == 2) {
                    startLine = 1;
                }
            }
            telemetry.addData("StartLine", startLine);
            telemetry.addData("TargetZone", targetZone);
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
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
     * with 0 as the heading at the time of IMU initialization.
     * Angles are positive in a counter-clockwise direction.
     */

    private float getHeading() {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Drive in a straight line.
     * @param distance How far to move, in inches.
     */
    private void drive(double distance) {
        debug("Drive is called");
        // Drive should be straight along the heading
        double desiredHeading = getHeading();
        debug("Heading " + desiredHeading);
        setMotorDistanceToTravel(distance, new int[]{1, 1, 1, 1});
        leftSpeed = robotSpeed;
        rightSpeed = robotSpeed;
        LFMotor.setPower(leftSpeed);
        LRMotor.setPower(leftSpeed);
        RFMotor.setPower(rightSpeed);
        RRMotor.setPower(rightSpeed);
        debug("Motors On");
        telemetry.update();
        while (!(isStopRequested() || !LFMotor.isBusy() || !LRMotor.isBusy() || !RFMotor.isBusy() || !RRMotor.isBusy())) {
            debug("Loop started");
            currentHeading = getHeading();
            delta = desiredHeading - currentHeading;
            if (Math.abs(delta) >= deltaThreshold) {
                if (delta > 0) {
                    if (distance > 0) {
                        rightSpeed = robotSpeed + correctionSpeed;
                        leftSpeed = robotSpeed - correctionSpeed;
                    } else {
                        rightSpeed = robotSpeed - correctionSpeed;
                        leftSpeed = robotSpeed + correctionSpeed;
                    }
                } else {
                    if (distance > 0) {
                        rightSpeed = robotSpeed - correctionSpeed;
                        leftSpeed = robotSpeed + correctionSpeed;
                    } else {
                        rightSpeed = robotSpeed + correctionSpeed;
                        leftSpeed = robotSpeed - correctionSpeed;
                    }
                }
            }
            PowerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while driving:
            telemetryDashboard();
        }
        // Stop the robot
        debug("End of loop");
        PowerTheWheels(0, 0, 0, 0);
        // Reset motor mode
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(!opModeIsActive()) {
            requestOpModeStop();
        }
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        LFMotor.setMode(mode);
        LRMotor.setMode(mode);
        RFMotor.setMode(mode);
        RRMotor.setMode(mode);
    }

    private void telemetryDashboard() {
        telemetry.addData("Start Line", startLine);
        telemetry.addData("Target Zone", targetZone);

        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Delta", delta);
        telemetry.addData("TurnSpeed", turnSpeed);

        telemetry.addData("LFPower", LFMotor.getPower());
        telemetry.addData("LRPower", LRMotor.getPower());
        telemetry.addData("RFPower", RFMotor.getPower());
        telemetry.addData("RRPower", RRMotor.getPower());

        List<NavigationInfo> allVisibleTargets = ringDetector.getNavigationInfo();
        if (allVisibleTargets != null) {
            lastKnownTargets = allVisibleTargets;
        }
        if (lastKnownTargets != null) {
            for (NavigationInfo visibleTarget : lastKnownTargets) {

                lastKnownPositionAndHeading.xPosition =  visibleTarget.translation.get(0);
                lastKnownPositionAndHeading.yPosition = visibleTarget.translation.get(1);
                float zPosition = visibleTarget.translation.get(2);
                float vuforiaRoll = visibleTarget.rotation.firstAngle;
                float vuforiaPitch = visibleTarget.rotation.secondAngle;
                /**
                 * Vuforia is off by 90 degrees compared to the IMU
                 * The code below matches the IMU value
                 */
                lastKnownPositionAndHeading.vuforiaHeading = normalizeHeading(visibleTarget.rotation.thirdAngle - 90);

                telemetry.addData("Visible Target", visibleTarget.targetName);
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        lastKnownPositionAndHeading.xPosition,
                        lastKnownPositionAndHeading.yPosition, zPosition);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                        vuforiaRoll, vuforiaPitch, lastKnownPositionAndHeading.vuforiaHeading);
            }
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void turn(double Heading) {
        desiredHeading = Heading;

        currentHeading = getHeading();
        delta = desiredHeading - currentHeading;
        while (!(isStopRequested() || Math.abs(delta) <= deltaThreshold)) {
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
            PowerTheWheels(leftFrontMotorPower, leftRearMotorPower, rightFrontMotorPower, rightRearMotorPower);
            telemetryDashboard();
            currentHeading = getHeading();
            delta = desiredHeading - currentHeading;
        }
        PowerTheWheels(0, 0, 0, 0);
        Hold(Heading);
    }

    /**
     * Describe this function...
     */
    private void Hold(double Heading) {
        ElapsedTime Timer;

        desiredHeading = Heading;
        currentHeading = getHeading();
        delta = desiredHeading - currentHeading;
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
            PowerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            sleep(75);
            PowerTheWheels(0, 0, 0, 0);
            currentHeading = getHeading();
            delta = desiredHeading - currentHeading;
            telemetry.addData("Desired Heading", desiredHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Delta", delta);
            telemetry.update();
        }
        PowerTheWheels(0, 0, 0, 0);
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Delta", delta);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void Strafe(double distance) {
        debug("Strafe is called");
        desiredHeading = getHeading();
        setMotorDistanceToTravel(distance, new int[]{-1, 1, 1, -1});
        leftFrontMotorPower = -robotSpeed;
        leftRearMotorPower = robotSpeed;
        rightFrontMotorPower = robotSpeed;
        rightRearMotorPower = -robotSpeed;
        PowerTheWheels(leftFrontMotorPower, leftRearMotorPower, rightFrontMotorPower, rightRearMotorPower);
        debug("Motors On");
        telemetry.addData("LFPower ", LFMotor.getPower());
        telemetry.addData("LRPower ", LRMotor.getTargetPosition());
        telemetry.addData("RFPower ", RFMotor.getTargetPosition());
        telemetry.addData("RRPower ", RRMotor.getPower());
        telemetry.update();
        while (!(isStopRequested() || !LFMotor.isBusy() || !LRMotor.isBusy() || !RFMotor.isBusy() || !RRMotor.isBusy())) {
            currentHeading = getHeading();
            delta = desiredHeading - currentHeading;
            if (Math.abs(delta) >= deltaThreshold) {
                if (delta > 0) {
                    if (distance > 0) {
                        rightSpeed = robotSpeed + correctionSpeed;
                        leftSpeed = robotSpeed - correctionSpeed;
                    } else {
                        rightSpeed = robotSpeed - correctionSpeed;
                        leftSpeed = robotSpeed + correctionSpeed;
                    }
                } else {
                    if (distance > 0) {
                        rightSpeed = robotSpeed - correctionSpeed;
                        leftSpeed = robotSpeed + correctionSpeed;
                    } else {
                        rightSpeed = robotSpeed + correctionSpeed;
                        leftSpeed = robotSpeed - correctionSpeed;
                    }
                }
            }
            PowerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            // Show motor power while strafing:
            telemetryDashboard();
        }
        // Stop the robot
        debug("End of loop");
        PowerTheWheels(0, 0, 0, 0);
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
        leftFrontTargetPosition = (int) (LFMotor.getCurrentPosition() + distanceInTicks);
        leftRearTargetPosition = (int) (LRMotor.getCurrentPosition() + distanceInTicks);
        rightFrontTargetPosition = (int) (RFMotor.getCurrentPosition() + distanceInTicks);
        rightRearTargetPosition = (int) (RRMotor.getCurrentPosition() + distanceInTicks);

        LFMotor.setTargetPosition(direction[0] * leftFrontTargetPosition);
        LRMotor.setTargetPosition(direction[1] * leftRearTargetPosition);
        RFMotor.setTargetPosition(direction[2] * rightFrontTargetPosition);
        RRMotor.setTargetPosition(direction[3] * rightRearTargetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void PowerTheWheels(double LFPower, double LRPower, double RFPower, double RRPower) {
        LFMotor.setPower(LFPower);
        LRMotor.setPower(LRPower);
        RFMotor.setPower(RFPower);
        RRMotor.setPower(RRPower);
    }
}
