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

    private boolean stopThread = false;

    /**
     * Describe this function...
     */
    private void AllSix() {
        if (startLine == 1) {
            if (targetZone == 1) {
                //1a
                drive(53);
                Turn(25);
                drive(17);
                drive(-17);
                Turn(0);
                Strafe(18);
                Hold(0);
                drive(21);
                Hold(0);
            } else if (targetZone == 2) {
                //1b
                drive(80);
                Turn(-30);
                drive(20);
                drive(-20);
                Turn(0);
                drive(-3);
                Hold(0);
            } else {
                //1c
                drive(100);
                Turn(25);
                drive(20);
                drive(-20);
                Turn(0);
                Strafe(18);
                Hold(0);
                drive(-20);
                Hold(0);
            }
        } else {
            if (targetZone == 1) {
                //2a
                drive(53);
                Turn(60);
                drive(36);
                drive(-36);
                Turn(0);
                drive(25);
                Hold(0);
            } else if (targetZone == 2) {
                //2b
                drive(80);
                Turn(30);
                drive(18);
                drive(-18);
                Turn(0);
                drive(-5);
                Hold(0);
            } else {
                //2c
                drive(100);
                Turn(60);
                drive(36);
                drive(-36);
                Turn(0);
                drive(-25);
                Hold(0);
            }
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.setHardwareMap(hardwareMap);
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
            ringDetector.init(robot);
        }
        catch (IllegalStateException e) {
            telemetry.addData("ERROR", e.getLocalizedMessage());
            telemetry.addData("Solution", "Diverting to TargetZone 1");
        }
        //Make robot legal-size by raising intake
        intakeLift.setPosition(1.0);
        telemetry.addData("Status", "Ready to start - v1.2.1");
        telemetry.update();

        waitForStart();

        Runnable lookForStop = new Runnable() {
            @Override
            public void run() {
                //Check to see if the opMode is running. Aborts when not running.
                while (opModeIsActive() || stopThread) {
                    idle();
                }
                requestOpModeStop();
            }
        };
        Thread background = new Thread(lookForStop);

        if (opModeIsActive()) {
            //background.start();   [Thread has other consequences]

            //TODO: make sure we check opModeIsActive
            countTheRings();
            // Put run blocks here.
            intakeLift.setPosition(0.9);
            sleep(1000);
            AllSix();
            intakeLift.setPosition(0.0);
        }
        currentHeading = getHeading();
        telemetry.addData("Current Heading", currentHeading);
        telemetry.update();
        sleep(5000);
        stopThread = true;
    }

    /**
     * Configure motor direction and modes.
     */
    private void initializeMotors() {
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
    private void debug(String text) {
        telemetry.addData("Debug", text);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void MatchSpecificUI() {
        boolean UIDone;

        startLine = 1;
        targetZone = 1;
        UIDone = false;
        while (!(UIDone || opModeIsActive())) {
            if (gamepad1.dpad_left) {
                startLine = 1;
            } else if (gamepad1.dpad_right) {
                startLine = 2;
            }
            if (gamepad1.a) {
                targetZone = 1;
            } else if (gamepad1.b) {
                targetZone = 2;
            } else if (gamepad1.y) {
                targetZone = 3;
            }
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                UIDone = true;
            }
            telemetry.update();
            telemetry.addData("Start Line", startLine);
            telemetry.addData("Target Zone", targetZone);
        }
    }

    private void countTheRings() {
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
                // TODO: Set the appropriate Target Zone

                if (ringDetector.QUAD.equals(label)) {
                    targetZone = 3;
                }
                else if (ringDetector.SINGLE.equals(label)) {
                    targetZone = 2;
                }

                telemetry.addData(String.format("label (%d)", i), label);
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
            }

            if(targetZone == 1) {
                //Check OpenCV to see if it sees orange

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
        distanceInTicks = distance * ticksPerInch;
        leftFrontTargetPosition = (int) (LFMotor.getCurrentPosition() + distanceInTicks);
        leftRearTargetPosition = (int) (LRMotor.getCurrentPosition() + distanceInTicks);
        rightFrontTargetPosition = (int) (RFMotor.getCurrentPosition() + distanceInTicks);
        rightRearTargetPosition = (int) (RRMotor.getCurrentPosition() + distanceInTicks);
        LFMotor.setTargetPosition(leftFrontTargetPosition);
        LRMotor.setTargetPosition(leftRearTargetPosition);
        RFMotor.setTargetPosition(rightFrontTargetPosition);
        RRMotor.setTargetPosition(rightRearTargetPosition);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            telemetry.addData("LFPower", LFMotor.getPower());
            telemetry.addData("LRPower", LRMotor.getPower());
            telemetry.addData("RFPower", RFMotor.getPower());
            telemetry.addData("RRPower", RRMotor.getPower());
            telemetry.update();
        }
        // Stop the robot
        debug("End of loop");
        PowerTheWheels(0, 0, 0, 0);
        // Reset motor mode
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(!opModeIsActive()) {
            requestOpModeStop();
        }
    }

    /**
     * Describe this function...
     */
    private void Turn(double Heading) {
        desiredHeading = Heading;
        currentHeading = getHeading();
        delta = desiredHeading - currentHeading;
        while (!(isStopRequested() || Math.abs(delta) <= deltaThreshold)) {
            if (delta > 0) {
                leftFrontMotorPower = -turnSpeed;
                leftRearMotorPower = -turnSpeed;
                rightFrontMotorPower = turnSpeed;
                rightRearMotorPower = turnSpeed;
            } else {
                leftFrontMotorPower = turnSpeed;
                leftRearMotorPower = turnSpeed;
                rightFrontMotorPower = -turnSpeed;
                rightRearMotorPower = -turnSpeed;
            }
            PowerTheWheels(leftFrontMotorPower, leftRearMotorPower, rightFrontMotorPower, rightRearMotorPower);
            telemetry.addData("Desired Heading", desiredHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Delta", delta);
            telemetry.addData("TurnSpeed", turnSpeed);
            telemetry.addData("LFPower", LFMotor.getPower());
            telemetry.addData("LRPower", LRMotor.getPower());
            telemetry.addData("RFPower", RFMotor.getPower());
            telemetry.addData("RRPower", RRMotor.getPower());
            telemetry.update();
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
        distanceInTicks = distance * ticksPerInch;
        leftFrontTargetPosition = (int) (LFMotor.getCurrentPosition() + distanceInTicks);
        leftRearTargetPosition = (int) (LRMotor.getCurrentPosition() + distanceInTicks);
        rightFrontTargetPosition = (int) (RFMotor.getCurrentPosition() + distanceInTicks);
        rightRearTargetPosition = (int) (RRMotor.getCurrentPosition() + distanceInTicks);
        LFMotor.setTargetPosition(-leftFrontTargetPosition);
        LRMotor.setTargetPosition(leftRearTargetPosition);
        RFMotor.setTargetPosition(rightFrontTargetPosition);
        RRMotor.setTargetPosition(-rightRearTargetPosition);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            telemetry.addData("LFPower", LFMotor.getPower());
            telemetry.addData("LRPower", LRMotor.getPower());
            telemetry.addData("RFPower", RFMotor.getPower());
            telemetry.addData("RRPower", RRMotor.getPower());
            telemetry.update();
        }
        // Stop the robot
        debug("End of loop");
        PowerTheWheels(0, 0, 0, 0);
        // Reset motor mode
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
