package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.scheduler.BaseAutonomousOpMode;
import org.firstinspires.ftc.teamcode.scheduler.TeamImu;

@Autonomous(name = "ParkingLeft", group="Autonomous")
public class AutoParkLeft extends BaseAutonomousOpMode<TeamRobot> {
    public AutoParkLeft() {
        super(new TeamRobot());
    }

    @Override
    public void teamInit() {
        super.teamInit();
    }

    @Override
    public void teamRun() {

    }
}
