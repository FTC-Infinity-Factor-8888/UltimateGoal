package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="MaxVelocityTest")
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
        rfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rrMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

                double P;
                double I;
                double D = 0;
                double F;

                F = 32767 / lfMaxVelocity;
                P = 0.1 * F;
                I = 0.1 * P;

                telemetry.addData("LF", "P (%.2f) I (%.3f) D (%.3f) F (%.1f)", P, I, D, F);
                System.out.printf("LF - P (%.2f) I (%.3f) D (%.3f) F (%.1f) TPS (%.0f)\n", P, I, D, F, lfMaxVelocity);

                F = 32767 / rfMaxVelocity;
                P = 0.1 * F;
                I = 0.1 * P;

                telemetry.addData("RF", "P (%.2f) I (%.3f) D (%.3f) F (%.1f)", P, I, D, F);
                System.out.printf("RF - P (%.2f) I (%.3f) D (%.3f) F (%.1f) TPS (%.0f)\n", P, I, D, F, rfMaxVelocity);


                F = 32767 / lrMaxVelocity;
                P = 0.1 * F;
                I = 0.1 * P;

                telemetry.addData("LR", "P (%.2f) I (%.3f) D (%.3f) F (%.1f)", P, I, D, F);
                System.out.printf("LR - P (%.2f) I (%.3f) D (%.3f) F (%.1f) TPS (%.0f)\n", P, I, D, F, lrMaxVelocity);

                F = 32767 / rrMaxVelocity;
                P = 0.1 * F;
                I = 0.1 * P;

                telemetry.addData("RR", "P (%.2f) I (%.3f) D (%.3f) F (%.1f)", P, I, D, F);
                System.out.printf("RR - P (%.2f) I (%.3f) D (%.3f) F (%.1f) TPS (%.0f)\n", P, I, D, F, rrMaxVelocity);

                telemetry.update();
            }
        }
    }
}

