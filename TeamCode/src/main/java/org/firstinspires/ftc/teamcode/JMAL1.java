package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.PositionAndHeading.VUFORIA;

@Autonomous(name = "JMA Line 1")
public class JMAL1 extends UltimateGoalRobot {

    private static final PositionAndHeading START_LINE_1 = new PositionAndHeading(-51.5, 47.75, 0,0);
    private static final PositionAndHeading START_LINE_2 = new PositionAndHeading(-51.5, 21.5, 0,0);
    private static final PositionAndHeading TARGET_ZONE_A = new PositionAndHeading(13, 58, 0,0);
    private static final PositionAndHeading TARGET_ZONE_B = new PositionAndHeading(34, 35, 0,0);
    private static final PositionAndHeading TARGET_ZONE_C = new PositionAndHeading(58, 58, 0,0);

    private static final double START_LINE = 1; // This autonomous program is for start line one.

    private Robot robot;

    private int targetZone;

    private PositionAndHeading tower = new PositionAndHeading(69,36,0,0);

    public int getStartLine() {
        return (int) START_LINE;
    }

    private void allThree() {
        if (targetZone == 1) {
            // 1a, going to target zone number 1 (a)
            robot.drive(51);
            robot.turn(45);
            robot.drive(14);
            robot.drive(-14);
            robot.turn(0);
            robot.strafe(18);
            robot.turn(0);
        }
        else if (targetZone == 2) {
            // 1b, going to target zone number 2 (b)
            robot.drive(72);
            robot.turn(-50);
            robot.drive(17.5);
            robot.drive(-17.5);
            robot.turn(0);
            robot.strafe(18);
            robot.turn(0);
        }
        else {
            // 1c, going to target zone number 3 (c)
            robot.drive(98);
            robot.turn(45);
            robot.drive(14);
            robot.drive(-14);
            robot.turn(0);
            robot.strafe(18);
            robot.turn(0);
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.setHardwareMap(hardwareMap);
        robot.setCameraAdjustX(11.0f);
        robot.setCameraAdjustY(-4.5f);
        robot.init();

        telemetry.addData("Status", "Ready to start - v1.9.9");
        telemetry.addData("ProximitySensor", robot.getProximitySensor().getDistance(DistanceUnit.INCH));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            try {
                targetZone = robot.findTargetZone();
                // Put run blocks here.
                robot.getIntakeLift().setPosition(0.9);
                allThree();
                // This is for the other version of autonomous
                // allThreeLine2();

                robot.navigationProbe(25);
                if (robot.navigationSource() == VUFORIA) {
                    robot.telemetryDashboard("runOpMode");
                    robot.hold(0);
                    robot.telemetryDashboard("Square to tower");
                    robot.strafe(robot.navigationY() - tower.yPosition);
                    robot.telemetryDashboard("Strafe in front");
                    robot.hold(0);
                    robot.telemetryDashboard("Square to tower");
                    robot.drive(tower.xPosition - robot.navigationX());
                    robot.getDumpBed().setPosition(0);
                    sleep(1000);
                    robot.telemetryDashboard("Dumped rings");
                    robot.drive(19 - robot.navigationX());
                    robot.telemetryDashboard("Parked on line");
                }
                robot.getIntakeLift().setPosition(0.0);
                robot.getDumpBed().setPosition(1);
            }
            catch (EmergencyStopException e) {
                // FORCE QUIT THE PROGRAM RIGHT NOW!!!
                // SHUT IT DOWN!!!
                // OVERRIDE EVERYTHING!!!
                telemetry.addData("Emergency Stop", e.getMessage());
                telemetry.update();
            }
        }
    }
}
