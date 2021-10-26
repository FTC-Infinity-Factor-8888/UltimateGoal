package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "JediMasterTeleOp (Blocks to Java)")
public class JediMasterTeleOp extends LinearOpMode {

    private DcMotor lfMotor;
    private DcMotor rfMotor;
    private DcMotor lrMotor;
    private DcMotor rrMotor;
    private DcMotor intakeWheels;
    private DcMotor ringShooter;
    //private DcMotor shooter;

    private BNO055IMU imu;
    private Servo dumpBed;
    private Servo intakeLift;
    private CRServo sweeperMotor;
    //private Servo sweeper;
    
    double leftFrontMotorVelocity;
    double leftRearMotorVelocity;
    double rightFrontMotorVelocity;
    double rightRearMotorVelocity;

    float currentHeading;
    float desiredHeading;
    double delta;
    double deltaThreshhold;

    double distanceInTicks;
    double ticksPerInch;

    double leftSpeed;
    double rightSpeed;
    double holdSpeed;
    double correctionSpeed;

    //Maximum amount of ticks/second, and also compensate for joystick direction.
    private int maximumRobotTps = -2350;
    private int maximumHalfTps = maximumRobotTps /2;

    private float intakeStepAmount = 0.05f;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        boolean sweeperButtonIsPressed;
        double intakeHeight;
        double TurnSpeed;
        boolean robotCanKeepGoing;
        double ticksPerMotorRev;
        double WheelCircumferenceInInches;
        double RobotSpeed;
        double StrafeSpeed;
        float StrafeHeading;

        lfMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        lrMotor = hardwareMap.get(DcMotor.class, "LR Motor");
        rrMotor = hardwareMap.get(DcMotor.class, "RR Motor");
        intakeWheels = hardwareMap.get(DcMotor.class, "IntakeWheels");
        ringShooter = hardwareMap.get(DcMotor.class, "RingShooter");
        //shooter = hardwareMap.get(DcMotor.class, "RingShooter");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        dumpBed = hardwareMap.get(Servo.class, "Servo1");
        intakeLift = hardwareMap.get(Servo.class, "IntakeLift");
        sweeperMotor = hardwareMap.get(CRServo.class, "SweeperMotor");
        //sweeper = hardwareMap.get(Servo.class, "SweeperMotor");

        ringShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        ringShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sweeperButtonIsPressed = false;

        // Initialize variables
        robotCanKeepGoing = true;
        ticksPerMotorRev = 530.3;
        // Convert 75mm wheel to inches
        WheelCircumferenceInInches = 9.6125;
        ticksPerInch = ticksPerMotorRev / WheelCircumferenceInInches;
        deltaThreshhold = 1;
        correctionSpeed = 0.1;
        RobotSpeed = 0.5;
        TurnSpeed = 0.5;
        holdSpeed = 0.1;
        StrafeSpeed = 0.5;
        StrafeHeading = currentHeading;
        initializeMotors();
        initializeIMU();
        telemetry.addData("Status", "Ready to start - v.6.0.1");
        telemetry.update();
        waitForStart();

        double currentRobotTps;
        double intakePosition = 0;

        if (opModeIsActive()) {
            //boolean sweeperMoving = false;
            //boolean sweeperButtonPushed = false;
            while (opModeIsActive()) {

                double forwardInput = gamepad1.left_stick_y;
                double strafeInput = gamepad1.left_stick_x;
                double turnInput = gamepad1.right_stick_x;
                double accelerator = gamepad1.left_trigger;

                //right trigger is to accelerate past the maximumHalfSpeed
                currentRobotTps = maximumHalfTps + maximumHalfTps * accelerator;
                //Strafe and Turn:
                leftFrontMotorVelocity = (forwardInput + strafeInput - turnInput);
                leftRearMotorVelocity = (forwardInput - strafeInput - turnInput);
                rightFrontMotorVelocity = (forwardInput - strafeInput + turnInput);
                rightRearMotorVelocity = (forwardInput + strafeInput + turnInput);

                double leftMax = Math.max(Math.abs(leftFrontMotorVelocity), Math.abs(leftRearMotorVelocity));
                double rightMax = Math.max(Math.abs(rightFrontMotorVelocity), Math.abs(rightRearMotorVelocity));
                double max = Math.max (leftMax, rightMax);

                if(max > 1.0) {
                    leftFrontMotorVelocity /= max;
                    leftRearMotorVelocity /= max;
                    rightFrontMotorVelocity /= max;
                    rightRearMotorVelocity /= max;
                }

                ((DcMotorEx) lfMotor).setVelocity(leftFrontMotorVelocity * currentRobotTps);
                ((DcMotorEx) lrMotor).setVelocity(leftRearMotorVelocity * currentRobotTps);
                ((DcMotorEx) rfMotor).setVelocity(rightFrontMotorVelocity * currentRobotTps);
                ((DcMotorEx) rrMotor).setVelocity(rightRearMotorVelocity * currentRobotTps);

                if (gamepad1.dpad_down) {
                    dumpBed.setPosition(0);
                } else if (gamepad1.dpad_up) {
                    dumpBed.setPosition(1);
                }

                if (gamepad1.dpad_left) {
                    intakePosition -= intakeStepAmount;
                    if (intakePosition < 0) {
                        intakePosition = 0;
                    }
                } else if (gamepad1.dpad_right) {
                    intakePosition += intakeStepAmount;
                    if (intakePosition > 1) {
                        intakePosition = 1;
                    }
                }
                intakeLift.setPosition(intakePosition);

                // y button is intake forward
                // x button is intake stop
                // a button is intake reverse
                if (gamepad1.y) {
                    intakeWheels.setPower(1);
                }
                else if (gamepad1.x) {
                    intakeWheels.setPower(0);
                }
                else if (gamepad1.a) {
                    intakeWheels.setPower(-1);
                }

                if (gamepad1.right_bumper) {
                    if (!sweeperButtonIsPressed) {
                        sweeperButtonIsPressed = true;
                        if (sweeperMotor.getPower() > 0) {
                            ((DcMotorEx) ringShooter).setVelocity(0);
                            sweeperMotor.setPower(0);
                        } else {
                            ((DcMotorEx) ringShooter).setVelocity(2300);
                            sweeperMotor.setPower(1);
                        }
                    }
                } else {
                    if (sweeperButtonIsPressed) {
                        sweeperButtonIsPressed = false;
                    }
                }

                telemetry.addData("Shooter", ((DcMotorEx) ringShooter).getVelocity());
                telemetry.addData("ServoPosition", dumpBed.getPosition());
                telemetry.addData("LF Velocity", ((DcMotorEx) lfMotor).getVelocity());
                telemetry.addData("RF Velocity", ((DcMotorEx) rfMotor).getVelocity());
                telemetry.addData("LR Velocity", ((DcMotorEx) lrMotor).getVelocity());
                telemetry.addData("RR Velocity", ((DcMotorEx) rrMotor).getVelocity());
                telemetry.addData("Accelerator", accelerator);
                telemetry.update();
            }

        }
        currentHeading = getHeading();
        delta = desiredHeading - currentHeading;
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("Delta", delta);
        telemetry.update();
        sleep(5000);
    }

    /**
     * Configure motor direction and modes.
     */
    private void initializeMotors() {
        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeWheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void debug(String text) {
        telemetry.addData("Debug", text);
        telemetry.update();
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
     * with 0 as the heading at the time of IMU initialization.
     * Angles are positive in a counter-clockwise direction.
     */
    private float getHeading() {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private void PowerTheWheels(double LFPower, double LRPower, double RFPower, double RRPower) {
        lfMotor.setPower(LFPower);
        lrMotor.setPower(LRPower);
        rfMotor.setPower(RFPower);
        rrMotor.setPower(RRPower);
    }
}
