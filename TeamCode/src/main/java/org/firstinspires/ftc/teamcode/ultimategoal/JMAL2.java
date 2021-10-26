package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.EmergencyStopException;
import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.ultimategoal.PositionAndHeading.VUFORIA;

@Autonomous(name = "JMA Line 2")
public class JMAL2 extends UltimateGoalRobot {

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
            // 2a, going to target zone number 1 (a)
            robot.drive(53);
            robot.turn(60);
            robot.drive(36);
            robot.drive(-36);
            robot.turn(0);
            robot.drive(25);
            robot.hold(0);
        }
        else if (targetZone == 2) {
            // 2b, going to target zone number 2 (b)
            robot.drive(80);
            robot.turn(30);
            robot.drive(18);
            robot.drive(-18);
            robot.turn(0);
            robot.drive(-5);
            robot.hold(0);
        }
        else {
            // 2c, going to target zone number 3 (c)
            robot.drive(100);
            robot.turn(60);
            robot.drive(36);
            robot.drive(-36);
            robot.turn(0);
            robot.drive(-25);
            robot.hold(0);
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
                // allThree();

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
