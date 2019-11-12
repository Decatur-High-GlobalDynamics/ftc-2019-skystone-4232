package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.scheduler.BaseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.scheduler.TeamImu;

@Autonomous(name = "ParkingLeft", group="Autonomous")
public class AutoParkLeft extends BaseAutonomousOpMode {
    TeamRobot robot = new TeamRobot();

    @Override
    public void teamInit() {
        super.teamInit();
        robot.init(telemetry, hardwareMap, this);
    }

    @Override
    public void teamRun() {

    }
}
