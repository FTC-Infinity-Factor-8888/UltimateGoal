package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LRMotor;
    private DcMotor RRMotor;
    private BNO055IMU imu;
    private Servo Servo1;
    private Servo intakeLift;

    double leftFrontMotorVelocity;
    double leftRearMotorVelocity;
    double rightFrontMotorVelocity;
    double rightRearMotorVelocity;
    float CurrentHeading;
    float desiredHeading;
    double Delta;
    double distanceInTicks;
    int leftFrontTargetPosition;
    double ticksPerInch;
    int LeftRearTargetPosition;
    int RightFrontTargetPosition;
    double DeltaThreshhold;
    int RightRearTargetPosition;
    double CorrectionSpeed;
    double LeftSpeed;
    double RightSpeed;
    double HoldSpeed;
    //Maximum amount of ticks/second, and also compensate for joystick direction.
    private int maximumRobotSpeed = -2350;
    private int maximumHalfSpeed = maximumRobotSpeed/2;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double TurnSpeed;
        boolean robotCanKeepGoing;
        double ticksPerMotorRev;
        double WheelCircumferenceInInches;
        double RobotSpeed;
        double StrafeSpeed;
        float StrafeHeading;

        LFMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        RFMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        LRMotor = hardwareMap.get(DcMotor.class, "LR Motor");
        RRMotor = hardwareMap.get(DcMotor.class, "RR Motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        intakeLift = hardwareMap.get(Servo.class, "IntakeLift");

        // Initialize variables
        robotCanKeepGoing = true;
        ticksPerMotorRev = 530.3;
        // Convert 75mm wheel to inches
        WheelCircumferenceInInches = 9.6125;
        ticksPerInch = ticksPerMotorRev / WheelCircumferenceInInches;
        DeltaThreshhold = 1;
        CorrectionSpeed = 0.1;
        RobotSpeed = 0.5;
        TurnSpeed = 0.5;
        HoldSpeed = 0.1;
        StrafeSpeed = 0.5;
        StrafeHeading = CurrentHeading;
        initializeMotors();
        initializeIMU();
        telemetry.addData("Status", "Ready to start - v5");
        telemetry.update();
        waitForStart();

        double currentRobotSpeed;
        double forwardAmount;
        double strafeAmount;
        double turnAmount;

        if (opModeIsActive()) {
            //TODO: Do stuff
            while (opModeIsActive()) {
                //right trigger is to accelerate past the maximumHalfSpeed
                currentRobotSpeed = maximumHalfSpeed + maximumHalfSpeed * gamepad1.left_trigger;
                forwardAmount = gamepad1.left_stick_y * currentRobotSpeed;
                strafeAmount = gamepad1.left_stick_x * currentRobotSpeed;
                turnAmount = gamepad1.right_stick_x * currentRobotSpeed;
                //Strafe and Turn:
                leftFrontMotorVelocity = (forwardAmount + strafeAmount - turnAmount) / 3;
                leftRearMotorVelocity = (forwardAmount - strafeAmount - turnAmount) / 3;
                rightFrontMotorVelocity = (forwardAmount - strafeAmount + turnAmount) / 3;
                rightRearMotorVelocity = (forwardAmount + strafeAmount + turnAmount) / 3;
                ((DcMotorEx) LFMotor).setVelocity(leftFrontMotorVelocity);
                ((DcMotorEx) LRMotor).setVelocity(leftRearMotorVelocity);
                ((DcMotorEx) RFMotor).setVelocity(rightFrontMotorVelocity);
                ((DcMotorEx) RRMotor).setVelocity(rightRearMotorVelocity);


                if (gamepad1.dpad_down) {
                    Servo1.setPosition(0);
                } else if (gamepad1.dpad_up) {
                    Servo1.setPosition(1);
                }

                if (gamepad1.dpad_left) {
                    intakeLift.setPosition(0);
                } else if (gamepad1.dpad_right) {
                    intakeLift.setPosition(1);
                }

                telemetry.addData("ServoPosition", Servo1.getPosition());
                telemetry.addData("LF Velocity", ((DcMotorEx) LFMotor).getVelocity());
                telemetry.addData("RF Velocity", ((DcMotorEx) RFMotor).getVelocity());
                telemetry.addData("LR Velocity", ((DcMotorEx) LRMotor).getVelocity());
                telemetry.addData("RR Velocity", ((DcMotorEx) RRMotor).getVelocity());
                telemetry.addData("RightTrigger", gamepad1.right_trigger);
                telemetry.update();
            }

        }
        CurrentHeading = getHeading();
        Delta = desiredHeading - CurrentHeading;
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Current Heading", CurrentHeading);
        telemetry.addData("Delta", Delta);
        telemetry.update();
        sleep(5000);
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
     * Describe this function...
     */
    private void Turn(double TurnSpeed, float Heading) {
        desiredHeading = Heading;
        CurrentHeading = getHeading();
        Delta = desiredHeading - CurrentHeading;
        while (!(isStopRequested() || Math.abs(Delta) <= DeltaThreshhold)) {
            if (Delta > 0) {
                leftFrontMotorVelocity = -TurnSpeed;
                leftRearMotorVelocity = -TurnSpeed;
                rightFrontMotorVelocity = TurnSpeed;
                rightRearMotorVelocity = TurnSpeed;
            } else {
                leftFrontMotorVelocity = TurnSpeed;
                leftRearMotorVelocity = TurnSpeed;
                rightFrontMotorVelocity = -TurnSpeed;
                rightRearMotorVelocity = -TurnSpeed;
            }
            PowerTheWheels(leftFrontMotorVelocity, leftRearMotorVelocity, rightFrontMotorVelocity, rightRearMotorVelocity);
            telemetry.addData("Desired Heading", desiredHeading);
            telemetry.addData("Current Heading", CurrentHeading);
            telemetry.addData("Delta", Delta);
            telemetry.addData("TurnSpeed", TurnSpeed);
            telemetry.addData("LFPower", LFMotor.getPower());
            telemetry.addData("LRPower", LRMotor.getPower());
            telemetry.addData("RFPower", RFMotor.getPower());
            telemetry.addData("RRPower", RRMotor.getPower());
            telemetry.update();
            CurrentHeading = getHeading();
            Delta = desiredHeading - CurrentHeading;
        }
        PowerTheWheels(0, 0, 0, 0);
    }

    /**
     * Describe this function...
     */

    /**
     * Drive in a straight line.
     * @param speed Motor speed, -1 to 1. Negative number moves robot backward.
     * @param distance How far to move, in inches.
     */
    private void drive(double speed, double distance) {
        debug("Drive is called");
        // Drive should be straight along the heading
        desiredHeading = getHeading();
        debug("Heading " + desiredHeading);
        distanceInTicks = distance * ticksPerInch;
        leftFrontTargetPosition = (int) (LFMotor.getCurrentPosition() + distanceInTicks);
        LeftRearTargetPosition = (int) (LRMotor.getCurrentPosition() + distanceInTicks);
        RightFrontTargetPosition = (int) (RFMotor.getCurrentPosition() + distanceInTicks);
        RightRearTargetPosition = (int) (RRMotor.getCurrentPosition() + distanceInTicks);
        LFMotor.setTargetPosition(leftFrontTargetPosition);
        LRMotor.setTargetPosition(LeftRearTargetPosition);
        RFMotor.setTargetPosition(RightFrontTargetPosition);
        RRMotor.setTargetPosition(RightRearTargetPosition);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftSpeed = speed;
        RightSpeed = speed;
        LFMotor.setPower(LeftSpeed);
        LRMotor.setPower(LeftSpeed);
        RFMotor.setPower(RightSpeed);
        RRMotor.setPower(RightSpeed);
        debug("Motors On");
        telemetry.addData("LFPower", LFMotor.getPower());
        telemetry.addData("LRPower", LRMotor.getCurrentPosition());
        telemetry.addData("RFPower", RFMotor.getCurrentPosition());
        telemetry.addData("RRPower", RRMotor.getPower());
        telemetry.update();
        if (isStopRequested()) {
            debug("Stop is requested");
        } else {
            if (!LFMotor.isBusy()) {
                debug("LF motor is not busy");
            } else {
                if (!LRMotor.isBusy()) {
                    debug("LR motor is not busy");
                } else {
                    if (!RFMotor.isBusy()) {
                        debug("RF motor is not busy");
                    } else {
                        if (!RRMotor.isBusy()) {
                            debug("RR motor is not busy");
                        } else {
                            debug("Loop starting");
                        }
                    }
                }
            }
        }
        sleep(1000);
        while (!(isStopRequested() || !LFMotor.isBusy() || !LRMotor.isBusy() || !RFMotor.isBusy() || !RRMotor.isBusy())) {
            debug("Loop started");
            CurrentHeading = getHeading();
            Delta = desiredHeading - CurrentHeading;
            if (Math.abs(Delta) >= DeltaThreshhold) {
                if (Delta > 0) {
                    if (distance > 0) {
                        RightSpeed = speed + CorrectionSpeed;
                        LeftSpeed = speed - CorrectionSpeed;
                    } else {
                        RightSpeed = speed - CorrectionSpeed;
                        LeftSpeed = speed + CorrectionSpeed;
                    }
                } else {
                    if (distance > 0) {
                        RightSpeed = speed - CorrectionSpeed;
                        LeftSpeed = speed + CorrectionSpeed;
                    } else {
                        RightSpeed = speed + CorrectionSpeed;
                        LeftSpeed = speed - CorrectionSpeed;
                    }
                }
            }
            PowerTheWheels(LeftSpeed, LeftSpeed, RightSpeed, RightSpeed);
            // TODO: Add telemetry information
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
    private void Strafe(double speed, double distance) {
        debug("Strafe is called");
        desiredHeading = getHeading();
        distanceInTicks = distance * ticksPerInch;
        leftFrontTargetPosition = (int) (LFMotor.getCurrentPosition() + distanceInTicks);
        LeftRearTargetPosition = (int) (LRMotor.getCurrentPosition() + distanceInTicks);
        RightFrontTargetPosition = (int) (RFMotor.getCurrentPosition() + distanceInTicks);
        RightRearTargetPosition = (int) (RRMotor.getCurrentPosition() + distanceInTicks);
        LFMotor.setTargetPosition(-leftFrontTargetPosition);
        LRMotor.setTargetPosition(LeftRearTargetPosition);
        RFMotor.setTargetPosition(RightFrontTargetPosition);
        RRMotor.setTargetPosition(-RightRearTargetPosition);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotorVelocity = -speed;
        leftRearMotorVelocity = speed;
        rightFrontMotorVelocity = speed;
        rightRearMotorVelocity = -speed;
        PowerTheWheels(leftFrontMotorVelocity, leftRearMotorVelocity, rightFrontMotorVelocity, rightRearMotorVelocity);
        debug("Motors On");
        telemetry.addData("LFPower ", LFMotor.getPower());
        telemetry.addData("LRPower ", LRMotor.getTargetPosition());
        telemetry.addData("RFPower ", RFMotor.getTargetPosition());
        telemetry.addData("RRPower ", RRMotor.getPower());
        telemetry.update();
        while (!(isStopRequested() || !LFMotor.isBusy() || !LRMotor.isBusy() || !RFMotor.isBusy() || !RRMotor.isBusy())) {
            debug("Loop started");
            CurrentHeading = getHeading();
            Delta = desiredHeading - CurrentHeading;
            if (Math.abs(Delta) >= DeltaThreshhold) {
                if (Delta > 0) {
                    if (distance > 0) {
                        RightSpeed = speed + CorrectionSpeed;
                        LeftSpeed = speed - CorrectionSpeed;
                    } else {
                        RightSpeed = speed - CorrectionSpeed;
                        LeftSpeed = speed + CorrectionSpeed;
                    }
                } else {
                    if (distance > 0) {
                        RightSpeed = speed - CorrectionSpeed;
                        LeftSpeed = speed + CorrectionSpeed;
                    } else {
                        RightSpeed = speed + CorrectionSpeed;
                        LeftSpeed = speed - CorrectionSpeed;
                    }
                }
            }
            PowerTheWheels(LeftSpeed, LeftSpeed, RightSpeed, RightSpeed);
            // TODO: Add telemetry information
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
