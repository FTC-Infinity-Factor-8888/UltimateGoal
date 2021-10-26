package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "MentorNav", group = "")
public class MentorNav extends LinearOpMode {

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private DcMotorEx LFMotor;
    private DcMotorEx LRMotor;
    private DcMotorEx RFMotor;
    private DcMotorEx RRMotor;

    static final double COUNTS_PER_MOTOR_REV = 28;   // REV HD Hex Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 7.5;     // For figuring circumference
    static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_CM * 0.393701;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.141593);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double MAX_SPEED = 0.8;   // Maximum speed for the robot because of traction
    static final double DRIVE_SPEED = 0.4; // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.25; // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1; // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;    // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.08;  // Larger is more responsive, but also less stable

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        // Put initialization blocks here.
        status("Initializing...");

        LFMotor = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LRMotor = hardwareMap.get(DcMotorEx.class, "LR Motor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RF Motor");
        RRMotor = hardwareMap.get(DcMotorEx.class, "RR Motor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Ensure the robot is stationary, then reset the encoders and calibrate the IMU.
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        status("Calibrating IMU...");

        // Make sure the imu accelerometers are calibrated before continuing
        if (!imu.initialize(parameters)) {
            status("IMU failed to initialize");
            return;
        }

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display IMU value), and reset gyro before we move..
        while (!isStarted()) {
            status("Waiting for start", false);
            telemetry.addData("IMU calib status", imu.getCalibrationStatus());
            telemetry.addData("IMU Heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }

        // Change the display status before starting the loop
        status("Running");

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        drive(DRIVE_SPEED, 48.0, 0.0); // Drive FWD 48 inches
        turn(TURN_SPEED, -45.0); // Turn  CW to -45 Degrees
        holdHeading(TURN_SPEED, -45.0, 0.5); // Hold -45 Deg heading for a 1/2 second
        drive(DRIVE_SPEED, 12.0, -45.0); // Drive FWD 12 inches at -45 degrees
        turn(TURN_SPEED,  45.0); // Turn  CCW  to  45 Degrees
        holdHeading(TURN_SPEED,  45.0, 0.5); // Hold 45 Deg heading for a 1/2 second
        drive(DRIVE_SPEED, 12.0, 45.0); // Drive FWD 12 inches at 45 degrees
        turn(TURN_SPEED,   0.0); // Turn  CW  to 0 Degrees
        holdHeading(TURN_SPEED,   0.0, 1.0); // Hold 0 Deg heading for a 1 second
        drive(DRIVE_SPEED, -64.97, 0.0); // Return to start, including two detours

        status("Complete");
    }

    private void status(String statusLine) {
        status(statusLine, true);
    }

    private void status(String statusLine, boolean autoUpdate) {
        telemetry.addData("Status", statusLine);
        if (autoUpdate) {
            telemetry.update();
        }
    }

    private void showDriveData(int newLfTarget, int newRfTarget, double leftSpeed, double rightSpeed, double error, double steer) {
        showUnifiedData(newLfTarget, newRfTarget, null, leftSpeed, rightSpeed, error, steer);
    }

    private void showHeadingData(double newAngle, double leftSpeed, double rightSpeed, double error, double steer) {
        showUnifiedData(null, null, newAngle, leftSpeed, rightSpeed, error, steer);
    }

    /**
     * Combined method to display either target distance or target angle. Intended to be called by the specific show* methods.
     *
     * @param newLfTarget  Left side target position
     * @param newRfTarget  Right side target position
     * @param newAngle     Target angle
     * @param leftSpeed    Left side motor speed
     * @param rightSpeed   Right side motor speed
     * @param error        Angle deviation from target
     * @param steer        Steering force being applied to fix deviation
     */
    private void showUnifiedData(Integer newLfTarget, Integer newRfTarget, Double newAngle, double leftSpeed, double rightSpeed, double error, double steer) {
        // Display drive status for the driver.
        if (newLfTarget != null && newRfTarget != null) {
            telemetry.addData("Target", "%7d:%7d", newLfTarget, newRfTarget);
        }
        else if (newAngle != null) {
            telemetry.addData("Target", "%5.2f deg", newAngle);
        }
        else {
            telemetry.addData("Target", "N/A");
        }
        telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
        telemetry.addData("Actual",  "%7d:%7d %5.2f deg",
                LFMotor.getCurrentPosition(), RFMotor.getCurrentPosition(), lastAngles.firstAngle);
        telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     *  Drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for +/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void drive (double speed, double distance, double angle) {
        int newLfTarget;
        int newLrTarget;
        int newRfTarget;
        int newRrTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLfTarget = LFMotor.getCurrentPosition() + moveCounts;
            newLrTarget = LRMotor.getCurrentPosition() + moveCounts;
            newRfTarget = RFMotor.getCurrentPosition() + moveCounts;
            newRrTarget = RRMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            LFMotor.setTargetPosition(newLfTarget);
            LRMotor.setTargetPosition(newLrTarget);
            RFMotor.setTargetPosition(newRfTarget);
            RRMotor.setTargetPosition(newRrTarget);

            LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            LFMotor.setPower(speed);
            LRMotor.setPower(speed);
            RFMotor.setPower(speed);
            RRMotor.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (LFMotor.isBusy() && LRMotor.isBusy() && RFMotor.isBusy() && RRMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- MAX_SPEED;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > MAX_SPEED) {
                    leftSpeed = leftSpeed / max * MAX_SPEED;
                    rightSpeed = rightSpeed / max * MAX_SPEED;
                }

                LFMotor.setPower(leftSpeed);
                LRMotor.setPower(leftSpeed);
                RFMotor.setPower(rightSpeed);
                RRMotor.setPower(rightSpeed);

                showDriveData(newLfTarget, newRfTarget, leftSpeed, rightSpeed, error, steer);
            }

            // Stop all motion
            LFMotor.setPower(0);
            LRMotor.setPower(0);
            RFMotor.setPower(0);
            RRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     */
    public void turn(double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Allow time for other processes to run.
            idle();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            onHeading(speed, angle, P_TURN_COEFF);
            // Allow time for other processes to run.
            idle();
        }

        // Stop all motion;
        LFMotor.setPower(0);
        LRMotor.setPower(0);
        RFMotor.setPower(0);
        RRMotor.setPower(0);
    }

    /**
     * Determine the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last IMU reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - lastAngles.firstAngle;
        while (robotError > 180) {
            robotError -= 360;
        }
        while (robotError <= -180) {
            robotError += 360;
        }
        return robotError;
    }

    /**
     * Calculate steering force
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return        Desired steering force.  +/- 1 range.  +ve = steer left
     */
    public double getSteer(double error, double PCoeff) {
        return error * PCoeff;
    }

    /**
     * Perform one cycle of closed loop heading control. This must be called within a loop which
     * checks
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return          True if current angle is within threshold of the target angle
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = Range.clip(speed * steer, -MAX_SPEED, MAX_SPEED);
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        LFMotor.setPower(leftSpeed);
        LRMotor.setPower(leftSpeed);
        RFMotor.setPower(rightSpeed);
        RRMotor.setPower(rightSpeed);

        showHeadingData(angle, leftSpeed, rightSpeed, error, steer);

        return onTarget;
    }
}
