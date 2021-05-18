package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "FLL Drive Test")
public class FLLDriveTest extends UltimateGoalRobot {

    private static final PositionAndHeading START_LINE_1 = new PositionAndHeading(-51.5, 47.75, 0,0);
    private static final PositionAndHeading START_LINE_2 = new PositionAndHeading(-51.5, 21.5, 0,0);
    private static final PositionAndHeading TARGET_ZONE_A = new PositionAndHeading(13, 58, 0,0);
    private static final PositionAndHeading TARGET_ZONE_B = new PositionAndHeading(34, 35, 0,0);
    private static final PositionAndHeading TARGET_ZONE_C = new PositionAndHeading(58, 58, 0,0);
    private static final PositionAndHeading BLUE_TOWER_GOAL = new PositionAndHeading(-69, 47, 0, 0);

    private static final double START_LINE = 1; // This autonomous program is for start line one.

    private Robot robot;
    private FakeFLLDrive fll;
    private int targetZone;

    private PositionAndHeading tower = new PositionAndHeading(69,36,0,0);

    public int getStartLine() {
        return (int) START_LINE;
    }

    private void allThree(double initialDrive) {
        if (targetZone == 1) {
            // 1a, going to target zone number 1 (a)
            robot.drive(53 - initialDrive);
            robot.turn(25);
            robot.drive(17);
            robot.drive(-17);
            robot.turn(0);
            robot.strafe(18);
            robot.turn(0);
            robot.drive(20);
        }
        else if (targetZone == 2) {
            // 1b, going to target zone number 2 (b)
            robot.drive(72 - initialDrive);
            robot.turn(-30);
            robot.drive(18);
            robot.drive(-18);
            robot.turn(0);
            //robot.strafe(28);
            //robot.turn(0);
        }
        else {
            // 1c, going to target zone number 3 (c)
            robot.drive(98 - initialDrive);
            robot.turn(35);
            robot.drive(17);
            robot.drive(-17);
            robot.turn(0);
            robot.drive(-29);

        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double detectionDrive = 0;
        robot = new Robot(this);
        robot.setHardwareMap(hardwareMap);
        robot.setCameraAdjustX(11.0f);
        robot.setCameraAdjustY(-4.5f);
        robot.init();

        telemetry.addData("Status", "Ready to start - v1.0");
        telemetry.addData("ProximitySensor", robot.getProximitySensor().getDistance(DistanceUnit.INCH));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            try {
                robot.getIntakeLift().setPosition(0.9);
                sleep(500);
                fll.drive(24);
                robot.getIntakeLift().setPosition(0.0);
            }
            catch (EmergencyStopException e) {
                // FORCE QUIT THE PROGRAM RIGHT NOW!!!
                // SHUT IT DOWN!!!
                // OVERRIDE EVERYTHING!!!
                telemetry.addData("Emergency Stop", e.getMessage());
                telemetry.update();
            }
            catch (NullPointerException e) {
                telemetry.addData("FLL Drive Won't Work", e.getMessage());
            }
        }
    }
}
