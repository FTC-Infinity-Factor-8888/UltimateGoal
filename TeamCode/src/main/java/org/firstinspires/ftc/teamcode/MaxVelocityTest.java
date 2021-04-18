package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx lfMotor;
    DcMotorEx rfMotor;
    DcMotorEx lrMotor;
    DcMotorEx rrMotor;
    double currentVelocity;
    double lfMaxVelocity = 0.0;
    double rfMaxVelocity = 0.0;
    double lrMaxVelocity = 0.0;
    double rrMaxVelocity = 0.0;



    @Override
    public void runOpMode() {
        lfMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LR Motor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RR Motor");
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            lfMotor.setPower(1.0);
            rfMotor.setPower(1.0);
            lrMotor.setPower(1.0);
            rrMotor.setPower(1.0);

            while (opModeIsActive()) {
                currentVelocity = lfMotor.getVelocity();
                if (currentVelocity > lfMaxVelocity) {
                    lfMaxVelocity = currentVelocity;
                }

                currentVelocity = rfMotor.getVelocity();

                if (currentVelocity > rfMaxVelocity) {
                    rfMaxVelocity = currentVelocity;
                }
                currentVelocity = lrMotor.getVelocity();
                if (currentVelocity > lrMaxVelocity) {
                    lrMaxVelocity = currentVelocity;
                }

                currentVelocity = rrMotor.getVelocity();
                if (currentVelocity > rrMaxVelocity) {
                    rrMaxVelocity = currentVelocity;
                }

                telemetry.addData("LF maximum velocity", lfMaxVelocity);

                telemetry.addData("Rf maximum velocity", rfMaxVelocity);

                telemetry.addData("LR maximum velocity", lrMaxVelocity);

                telemetry.addData("RR maximum velocity", rrMaxVelocity);
                telemetry.update();
            }
        }
    }
}

