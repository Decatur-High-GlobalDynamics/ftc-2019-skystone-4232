package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.scheduler.BaseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.scheduler.EndableAction;
import org.firstinspires.ftc.teamcode.scheduler.TeamImu;

@Autonomous(name = "FoundationRight", group="Autonomous")
public class PushFoundationRight extends BaseAutonomousOpMode<TeamRobot> {
    public PushFoundationRight() {
        super(new TeamRobot());
    }

    @Override
    public void teamInit() {
        super.teamInit();
    }

    @Override
    public void teamRun() {
        try {
            robot.startInchMoveBack(25, -0.5).waitUntilFinished();
            /*robot.startInchMove(5, 0.5).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startTurningRight(90).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startInchMove(54, 0.75).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startTurningLeft(90).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startInchMove(58, 0.5).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startTurningRight(45).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startInchMove(20, 0.75).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startInchMoveBack(20, -0.5).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startTurningRight(45).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startInchMove(26.5, 0.5).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startTurningRight(90).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startInchMove(55, 0.75).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startTurningRight(90).waitUntilFinished();
            robot.startStopping().waitUntilFinished();
            robot.startInchMove(50.5, 0.75).waitUntilFinished();
            robot.startStopping().waitUntilFinished();*/
        } catch (Exception e) {}
    }
}
