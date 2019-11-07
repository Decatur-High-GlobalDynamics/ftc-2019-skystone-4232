package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Schooch", group="Linear op mode")
public class SchoochAuto extends LinearOpMode {
    TeamRobot robot = new TeamRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.leftDrive.setPower(.25);
        robot.rightDrive.setPower(.35);
        try
        {
            Thread.sleep(500);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }
}
