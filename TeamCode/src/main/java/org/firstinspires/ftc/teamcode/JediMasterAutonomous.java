package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.EmergencyStopException;
import org.firstinspires.ftc.teamcode.PositionAndHeading;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.UltimateGoalRobot;

import java.util.List;

import static org.firstinspires.ftc.teamcode.PositionAndHeading.FIXED;
import static org.firstinspires.ftc.teamcode.PositionAndHeading.VUFORIA;

@Autonomous(name = "JediMasterAutonomous (Blocks to Java)")
public class JediMasterAutonomous extends UltimateGoalRobot {


    private final PositionAndHeading START_LINE_1 = new PositionAndHeading(-51.5, 47.75, 0,0);
    private final PositionAndHeading START_LINE_2 = new PositionAndHeading(-51.5, 21.5, 0,0);
    private final PositionAndHeading TARGET_ZONE_A = new PositionAndHeading(13, 58, 0,0);
    private final PositionAndHeading TARGET_ZONE_B = new PositionAndHeading(34, 35, 0,0);
    private final PositionAndHeading TARGET_ZONE_C = new PositionAndHeading(58, 58, 0,0);

    private Servo intakeLift;
    private Servo dumpBed;




    private Robot robot;


    double startLine = 1; //default to the first start line
    PositionAndHeading startLineCoordinates = START_LINE_1;
    PositionAndHeading targetZoneCoordinates = TARGET_ZONE_A;



    double leftFrontMotorPower;
    double leftRearMotorPower;
    double rightFrontMotorPower;
    double rightRearMotorPower;

    double distanceInTicks;
    int leftFrontTargetPosition;
    int leftRearTargetPosition;
    int rightFrontTargetPosition;
    int rightRearTargetPosition;


    double robotSpeed;
    double leftSpeed;
    double rightSpeed;

    double turnSpeed;
    double correctionSpeed;
    double holdSpeed;
    double holdTime;
    static double MIN_ROBOT_POWER = 0.2;
    static double MAX_ROBOT_POWER = 0.8;
    static double POWER_RANGE = MAX_ROBOT_POWER - MIN_ROBOT_POWER;



    private PositionAndHeading lastKnownPositionAndHeading = new PositionAndHeading();
    private PositionAndHeading tower = new PositionAndHeading(69,36,0,0);

    public int getStartLine() {
        return (int) startLine;
    }

    private void allSix() {
        //Looks like we are using this after all
        if (startLine == 1) {
            if (targetZone == 1) {
                //1a
                drive(51);
                turn(45);
                drive(14);
                drive(-14);
                turn(0);
                strafe(18);
                turn(0);
            } else if (targetZone == 2) {
                //1b
                drive(72);
                turn(-50);
                drive(17.5);
                drive(-17.5);
                turn(0);
                strafe(18);
                turn(0);
            } else {
                //1c
                drive(98);
                turn(45);
                drive(14);
                drive(-14);
                turn(0);
                strafe(18);
                turn(0);
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
                if (robotSpeed < minimumRobotSpeed) {
                    robotSpeed = minimumRobotSpeed;
                }
            }
            else if (Math.abs(delta) - priorDelta < 0) {
                robotSpeed += speedAdjust;
                if (robotSpeed > maximumRobotSpeed) {
                    robotSpeed = maximumRobotSpeed;
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
        robot = new Robot(this);
        robot.setHardwareMap(hardwareMap);
        robot.setCameraAdjustX(11.0f);
        robot.setCameraAdjustY(-4.5f);


        intakeLift = hardwareMap.get(Servo.class, "IntakeLift");
        dumpBed = hardwareMap.get(Servo.class, "Servo1");




        deltaThreshold = 1;
        correctionSpeed = 0.1;
        robotSpeed = 0.5;
        turnSpeed = POWER_RANGE;
        holdSpeed = 0.1;
        holdTime = 1000;
        robot.initializeMotors();
        robot.initializeIMU();

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
        telemetry.addData("Status", "Ready to start - v1.9.9");
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
                allSix();

                // IMU starts to pay attention to where it's going, in case Vuforia doesn't pick up the target.
                /*
                Position position = new Position(DistanceUnit.INCH, startLineCoordinates.xPosition, startLineCoordinates.yPosition, 0, System.nanoTime());
                imu.startAccelerationIntegration(position, null, 1);
                */
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
                    drive(19 - lastKnownPositionAndHeading.xPosition);
                    telemetryDashboard("Parked on line");
                }
                intakeLift.setPosition(0.0);
                dumpBed.setPosition(1);
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

        double priorDelta = 0;
        double currentPolarHeading = getPolarHeading();
        delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);
        while (opModeIsActive() && Math.abs(delta) > deltaThreshold) {
            if (delta - priorDelta > 0) {
                turnSpeed -= speedAdjust;
                if (turnSpeed < 0) {
                    turnSpeed = 0;
                }
            }
            else {
                turnSpeed += speedAdjust;
                if (turnSpeed > POWER_RANGE) {
                    turnSpeed = POWER_RANGE;
                }
            }

            currentPolarHeading = getPolarHeading();
            priorDelta = delta;
            delta = normalizeHeading(desiredPolarHeading - currentPolarHeading);

            double deltaPercentage =  powerPercentage(delta);
            double currentTurnSpeed = turnSpeed * deltaPercentage + MIN_ROBOT_POWER;
            if (delta < 0) {
                currentTurnSpeed = -currentTurnSpeed;
            }
            leftSpeed = -currentTurnSpeed;
            rightSpeed = currentTurnSpeed;
            powerTheWheels(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
            telemetryDashboard("Turn(" + (int) imuHeading + ")");
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
        leftSpeed = robotSpeed;
        rightSpeed = robotSpeed;
        powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
        telemetryDashboard("Strafe(" + distance + ")");

        debug("Motors On");
        double priorDelta = 0.0;
        while (opModeIsActive() && lfMotor.isBusy() && lrMotor.isBusy() && rfMotor.isBusy() &&
                rrMotor.isBusy()) {
            priorDelta = adjustSpeed(distance, desiredPolarHeading, priorDelta);
            powerTheWheels(rightSpeed, leftSpeed, leftSpeed, rightSpeed);
            // Show motor power while strafing:
            telemetryDashboard("Strafe(" + distance + ")");
        }

        if(!opModeIsActive()) {
            throw new EmergencyStopException("Strafe");
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



    private double powerPercentage(double delta) {
        double powerPercent = -0.00002 * Math.pow(Math.abs(delta) - 180, 2) + 1;
        if (powerPercent > 1 || powerPercent < 0) {
            System.out.println("*** WARNING! POWER PERCENT IS OUT OF RANGE: delta = " + delta + ", " +
                    "powerPercent = " + powerPercent + " ***");
        }

        return powerPercent;
    }
}
