package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="DrivingAccuracyTest")
public class DrivingAccuracyTest extends LinearOpMode {
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

        lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfMotor.setVelocityPIDFCoefficients(1.2342, 0.1234, 0, 12.3416);
        rfMotor.setVelocityPIDFCoefficients(1.2365, 0.1236, 0, 12.3649);
        lrMotor.setVelocityPIDFCoefficients(1.2554, 0.1255, 0, 12.5544);
        rrMotor.setVelocityPIDFCoefficients(1.2530, 0.1253, 0, 12.5304);

        // Change these to optimize the robot

        double distance = 100;
        double positionPIDF = 4.5;
        double power = 0.75;
        double WheelCircumferenceInInches = 10.0625;

        lfMotor.setPositionPIDFCoefficients(positionPIDF);
        rfMotor.setPositionPIDFCoefficients(positionPIDF);
        lrMotor.setPositionPIDFCoefficients(positionPIDF);
        rrMotor.setPositionPIDFCoefficients(positionPIDF);

        waitForStart();

        double ticksPerMotorRev = 560.0;
        double ticksPerInch = ticksPerMotorRev/ WheelCircumferenceInInches;
        int distanceToTravel = (int) (distance * ticksPerInch);

        lfMotor.setTargetPosition(distanceToTravel);
        rfMotor.setTargetPosition(distanceToTravel);
        lrMotor.setTargetPosition(distanceToTravel);
        rrMotor.setTargetPosition(distanceToTravel);

        lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (opModeIsActive()) {

            lfMotor.setPower(power);
            rfMotor.setPower(power);
            lrMotor.setPower(power);
            rrMotor.setPower(power);

            while (opModeIsActive() && lfMotor.isBusy() && rfMotor.isBusy() && lrMotor.isBusy() && rrMotor.isBusy()) {

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

                telemetry.addData("LF", "TPS (%.0f)", lfMaxVelocity);
                System.out.printf("LF - TPS (%.0f)\n", lfMaxVelocity);

                telemetry.addData("RF", "TPS (%.0f)", rfMaxVelocity);
                System.out.printf("RF - TPS (%.0f)\n", rfMaxVelocity);

                telemetry.addData("LR", "TPS (%.0f)", lrMaxVelocity);
                System.out.printf("LR - TPS (%.0f)\n", lrMaxVelocity);

                telemetry.addData("RR", "TPS (%.0f)", rrMaxVelocity);
                System.out.printf("RR - TPS (%.0f)\n", rrMaxVelocity);

                telemetry.update();
            }
            lfMotor.setPower(0);
            rfMotor.setPower(0);
            lrMotor.setPower(0);
            rrMotor.setPower(0);

            System.out.println("Robot has stopped");
        }
    }
}

