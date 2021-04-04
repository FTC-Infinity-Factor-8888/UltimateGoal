package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "WheelTest")
public class WheelTest extends LinearOpMode {
    private DcMotorEx lfMotor;
    private DcMotorEx rfMotor;
    private DcMotorEx lrMotor;
    private DcMotorEx rrMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        lfMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LR Motor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RR Motor");

        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            testAWheel(lfMotor, "LeftFront");
            testAWheel(lrMotor, "LeftRear");
            testAWheel(rfMotor, "RightFront");
            testAWheel(rrMotor, "RightRear");
        }
    }

    public void testAWheel (DcMotorEx motor, String motorName) {
        if (opModeIsActive()) {
            motor.setPower(1.0);
            telemetryDashboard(motor, motorName);
            motor.setPower(-1.0);
            telemetryDashboard(motor, motorName);
            motor.setPower(0.0);
        }
    }

    public void telemetryDashboard(DcMotorEx motor, String motorName) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 2000 && opModeIsActive()) {
            telemetry.addData("Motor", motorName);
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("Encoder", motor.getVelocity());
            telemetry.update();
            idle();
        }
    }
}
