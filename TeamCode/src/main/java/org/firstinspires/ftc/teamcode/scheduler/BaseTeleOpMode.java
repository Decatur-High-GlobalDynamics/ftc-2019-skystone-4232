package org.firstinspires.ftc.teamcode.scheduler;

import org.firstinspires.ftc.robotcore.external.Func;

public abstract class BaseTeleOpMode<RobotClass extends Robot> extends BaseLinearOpMode<RobotClass>
{
    public BaseTeleOpMode(RobotClass robot) {
        super(robot);
    }


    // Called over and over until stop button is pressed
    protected abstract void teleOpLoop();

    @Override
    public void teamInit() {
        telemetry.addLine("GP1:")
                .addData("", new Func<String>() {
                    @Override public String value() {
                        return robot.saveTelemetryData("GP1", gamepad1.toString());
                    }});

        telemetry.addLine("GP2:")
                .addData("", new Func<String>() {
                    @Override public String value() {
                        return robot.saveTelemetryData("GP2", gamepad1.toString());
                    }});
    }


    // First TeleOp thing that happens after play button is pressed
    public void teleOpStart()
    {
    }

    // Convert LinearOpMode into OpMode
    @Override
    protected void teamRun()
    {
        teleOpStart();

        while (shouldOpModeKeepRunning())
        {
            teleOpLoop();
        }
    }

}
