package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpFF (Blocks to Java)")
class TeleOpFF extends LinearOpMode {

    private DcMotor LFMotor;
    private DcMotor LRMotor;
    private DcMotor RFMotor;
    private DcMotor RRMotor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() { //voids mean no anwer is returned from method
        double rDrive; //originally float (point variable (blocks)) Decimal values
        double lDrive;

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LRMotor = hardwareMap.get(DcMotor.class, "LRMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RRMotor = hardwareMap.get(DcMotor.class, "RRMotor");

        // Put initialization blocks here.
        LFMotor.setDirection(DcMotorSimple.Direction.REVERSE); //we want the wheels to go the right way
        LRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                rDrive = gamepad1.right_stick_y; //when the right moves up+down, the right wheels go
                lDrive = gamepad1.left_stick_y; //when the left moves up+down, the left wheels go
                LFMotor.setPower(lDrive);
                LRMotor.setPower(lDrive);
                RFMotor.setPower(rDrive);
                RRMotor.setPower(rDrive);
                telemetry.addData("Left stick", lDrive); //
                telemetry.addData("Right stick", rDrive); //
                telemetry.update(); // Updated telemetry based on how much the robot moved
            }
        }
    }
}