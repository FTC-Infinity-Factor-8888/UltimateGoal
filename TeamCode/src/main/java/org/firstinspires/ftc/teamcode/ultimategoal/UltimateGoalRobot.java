package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class UltimateGoalRobot extends LinearOpMode {
    /**
     * Returns the robot starting line
     *
     * @return 1 if on start line one, 2 if on start line two
     */
    public abstract int getStartLine();

}
