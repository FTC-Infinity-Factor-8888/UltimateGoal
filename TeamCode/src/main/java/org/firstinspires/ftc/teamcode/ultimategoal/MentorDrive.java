package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MentorDrive", group = "")
public class MentorDrive extends LinearOpMode {

    private static final double MAX_VELOCITY = 2350.0; // Maximum we want the motors to run

    private DcMotorEx LFMotor;
    private DcMotorEx LRMotor;
    private DcMotorEx RFMotor;
    private DcMotorEx RRMotor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double leftStickY;
        double leftStickX;
        double rightStickX;

        double accelerator;

        double lfMotorVelocity;
        double lrMotorVelocity;
        double rfMotorVelocity;
        double rrMotorVelocity;

        LFMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LRMotor = hardwareMap.get(DcMotorEx.class, "LR Motor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        RRMotor = hardwareMap.get(DcMotorEx.class, "RR Motor");

        // Put initialization blocks here.

        // Ensure the robot is stationary, then reset the encoders
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                leftStickY = -gamepad1.left_stick_y; // Invert the input so up is positive
                leftStickX = -gamepad1.left_stick_x; // Invert the input so right is positive
                rightStickX = -gamepad1.right_stick_x; // Invert the input so right is positive

                accelerator = gamepad1.left_trigger;

                // Calculate power for each motor
                setMotorVelocities(leftStickY, leftStickX, rightStickX, accelerator);

                // Add debugging information
                telemetry.addData("LF Velocity", LFMotor.getVelocity());
                telemetry.addData("RF Velocity", RFMotor.getVelocity());
                telemetry.addData("LR Velocity", LRMotor.getVelocity());
                telemetry.addData("RR Velocity", RRMotor.getVelocity());
                telemetry.addData("Accelerator", accelerator);
                telemetry.update();
            }
        }
    }

    private void setMotorVelocities(double forward, double strafe, double rotate, double accel) {
        double maxVelocity = MAX_VELOCITY / 2.0;
        maxVelocity += maxVelocity * accel;
        double lfPower = forward + strafe - rotate;
        double rfPower = forward - strafe + rotate;
        double lrPower = forward - strafe - rotate;
        double rrPower = forward + strafe + rotate;
        double leftMax = Math.max(Math.abs(lfPower), Math.abs(lrPower));
        double rightMax = Math.max(Math.abs(rfPower), Math.abs(rrPower));
        double max = Math.max(leftMax, rightMax);
        if (max > 1.0) {
            lfPower /= max;
            rfPower /= max;
            lrPower /= max;
            rrPower /= max;
        }
        LFMotor.setVelocity(lfPower * maxVelocity);
        RFMotor.setVelocity(rfPower * maxVelocity);
        LRMotor.setVelocity(lrPower * maxVelocity);
        RRMotor.setVelocity(rrPower * maxVelocity);
    }
}
