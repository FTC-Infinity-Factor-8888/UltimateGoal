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

@Autonomous(name = "JediMasterAutonomous (Blocks to Java)")
public class JediMasterAutonomous extends LinearOpMode {

    private Servo IntakeLift;
    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LRMotor;
    private DcMotor RRMotor;
    private BNO055IMU imu;

    boolean robotCanKeepGoing;
    double leftRearMotorPower;
    double StartLine;
    double desiredHeading;
    double leftFrontMotorPower;
    double rightFrontMotorPower;
    double TargetZone;
    float CurrentHeading;
    double rightRearMotorPower;
    double Delta;
    double distanceInTicks;
    int leftFrontTargetPosition;
    double ticksPerInch;
    int LeftRearTargetPosition;
    int RightFrontTargetPosition;
    double DeltaThreshhold;
    double TurnSpeed;
    int RightRearTargetPosition;
    double CorrectionSpeed;
    double LeftSpeed;
    double RobotSpeed;
    double RightSpeed;
    double HoldSpeed;
    double HoldTime;

    /**
     * Describe this function...
     */
    private void AllSix() {
        if (StartLine == 1) {
            if (TargetZone == 1) {
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
            } else if (TargetZone == 2) {
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
            if (TargetZone == 1) {
                //2a
                drive(53);
                Turn(60);
                drive(36);
                drive(-36);
                Turn(0);
                drive(25);
                Hold(0);
            } else if (TargetZone == 2) {
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
        double ticksPerMotorRev;
        double WheelCircumferenceInInches;

        IntakeLift = hardwareMap.get(Servo.class, "IntakeLift");
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
        DeltaThreshhold = 1;
        CorrectionSpeed = 0.1;
        RobotSpeed = 0.5;
        TurnSpeed = 0.3;
        HoldSpeed = 0.1;
        HoldTime = 2000;
        initializeMotors();
        initializeIMU();
        //Testing only. Remove before Competition!!!
        IntakeLift.setPosition(1.0);
        MatchSpecificUI();
        telemetry.addData("Status", "Ready to start - v1.1.5");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            IntakeLift.setPosition(0.9);
            sleep(1000);
            AllSix();
            IntakeLift.setPosition(0.0);
        }
        CurrentHeading = getHeading();
        telemetry.addData("Current Heading", CurrentHeading);
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
    private void MatchSpecificUI() {
        boolean UIDone;

        StartLine = 1;
        TargetZone = 1;
        UIDone = false;
        while (!(UIDone || opModeIsActive())) {
            if (gamepad1.dpad_left) {
                StartLine = 1;
            } else if (gamepad1.dpad_right) {
                StartLine = 2;
            }
            if (gamepad1.a) {
                TargetZone = 1;
            } else if (gamepad1.b) {
                TargetZone = 2;
            } else if (gamepad1.y) {
                TargetZone = 3;
            }
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                UIDone = true;
            }
            telemetry.update();
            telemetry.addData("Start Line", StartLine);
            telemetry.addData("Target Zone", TargetZone);
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
        LeftSpeed = RobotSpeed;
        RightSpeed = RobotSpeed;
        LFMotor.setPower(LeftSpeed);
        LRMotor.setPower(LeftSpeed);
        RFMotor.setPower(RightSpeed);
        RRMotor.setPower(RightSpeed);
        debug("Motors On");
        telemetry.update();
        while (!(isStopRequested() || !LFMotor.isBusy() || !LRMotor.isBusy() || !RFMotor.isBusy() || !RRMotor.isBusy())) {
            debug("Loop started");
            CurrentHeading = getHeading();
            Delta = desiredHeading - CurrentHeading;
            if (Math.abs(Delta) >= DeltaThreshhold) {
                if (Delta > 0) {
                    if (distance > 0) {
                        RightSpeed = RobotSpeed + CorrectionSpeed;
                        LeftSpeed = RobotSpeed - CorrectionSpeed;
                    } else {
                        RightSpeed = RobotSpeed - CorrectionSpeed;
                        LeftSpeed = RobotSpeed + CorrectionSpeed;
                    }
                } else {
                    if (distance > 0) {
                        RightSpeed = RobotSpeed - CorrectionSpeed;
                        LeftSpeed = RobotSpeed + CorrectionSpeed;
                    } else {
                        RightSpeed = RobotSpeed + CorrectionSpeed;
                        LeftSpeed = RobotSpeed - CorrectionSpeed;
                    }
                }
            }
            PowerTheWheels(LeftSpeed, LeftSpeed, RightSpeed, RightSpeed);
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
    }

    /**
     * Describe this function...
     */
    private void Turn(double Heading) {
        desiredHeading = Heading;
        CurrentHeading = getHeading();
        Delta = desiredHeading - CurrentHeading;
        while (!(isStopRequested() || Math.abs(Delta) <= DeltaThreshhold)) {
            if (Delta > 0) {
                leftFrontMotorPower = -TurnSpeed;
                leftRearMotorPower = -TurnSpeed;
                rightFrontMotorPower = TurnSpeed;
                rightRearMotorPower = TurnSpeed;
            } else {
                leftFrontMotorPower = TurnSpeed;
                leftRearMotorPower = TurnSpeed;
                rightFrontMotorPower = -TurnSpeed;
                rightRearMotorPower = -TurnSpeed;
            }
            PowerTheWheels(leftFrontMotorPower, leftRearMotorPower, rightFrontMotorPower, rightRearMotorPower);
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
        Hold(Heading);
    }

    /**
     * Describe this function...
     */
    private void Hold(double Heading) {
        ElapsedTime Timer;

        desiredHeading = Heading;
        CurrentHeading = getHeading();
        Delta = desiredHeading - CurrentHeading;
        Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        Timer.reset();
        while (opModeIsActive() && Math.abs(Delta) > 0.5 && Timer.time() < HoldTime) {
            if (Delta > 0) {
                LeftSpeed = -HoldSpeed;
                RightSpeed = HoldSpeed;
            } else {
                LeftSpeed = HoldSpeed;
                RightSpeed = -HoldSpeed;
            }
            PowerTheWheels(LeftSpeed, LeftSpeed, RightSpeed, RightSpeed);
            sleep(75);
            PowerTheWheels(0, 0, 0, 0);
            CurrentHeading = getHeading();
            Delta = desiredHeading - CurrentHeading;
            telemetry.addData("Desired Heading", desiredHeading);
            telemetry.addData("Current Heading", CurrentHeading);
            telemetry.addData("Delta", Delta);
            telemetry.update();
        }
        PowerTheWheels(0, 0, 0, 0);
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Current Heading", CurrentHeading);
        telemetry.addData("Delta", Delta);
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
        leftFrontMotorPower = -RobotSpeed;
        leftRearMotorPower = RobotSpeed;
        rightFrontMotorPower = RobotSpeed;
        rightRearMotorPower = -RobotSpeed;
        PowerTheWheels(leftFrontMotorPower, leftRearMotorPower, rightFrontMotorPower, rightRearMotorPower);
        debug("Motors On");
        telemetry.addData("LFPower ", LFMotor.getPower());
        telemetry.addData("LRPower ", LRMotor.getTargetPosition());
        telemetry.addData("RFPower ", RFMotor.getTargetPosition());
        telemetry.addData("RRPower ", RRMotor.getPower());
        telemetry.update();
        while (!(isStopRequested() || !LFMotor.isBusy() || !LRMotor.isBusy() || !RFMotor.isBusy() || !RRMotor.isBusy())) {
            CurrentHeading = getHeading();
            Delta = desiredHeading - CurrentHeading;
            if (Math.abs(Delta) >= DeltaThreshhold) {
                if (Delta > 0) {
                    if (distance > 0) {
                        RightSpeed = RobotSpeed + CorrectionSpeed;
                        LeftSpeed = RobotSpeed - CorrectionSpeed;
                    } else {
                        RightSpeed = RobotSpeed - CorrectionSpeed;
                        LeftSpeed = RobotSpeed + CorrectionSpeed;
                    }
                } else {
                    if (distance > 0) {
                        RightSpeed = RobotSpeed - CorrectionSpeed;
                        LeftSpeed = RobotSpeed + CorrectionSpeed;
                    } else {
                        RightSpeed = RobotSpeed + CorrectionSpeed;
                        LeftSpeed = RobotSpeed - CorrectionSpeed;
                    }
                }
            }
            PowerTheWheels(LeftSpeed, LeftSpeed, RightSpeed, RightSpeed);
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
