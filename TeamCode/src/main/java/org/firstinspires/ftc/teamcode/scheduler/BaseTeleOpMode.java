package org.firstinspires.ftc.teamcode.scheduler;

import org.firstinspires.ftc.robotcore.external.Func;

public abstract class BaseTeleOpMode<RobotClass extends Robot> extends BaseLinearOpMode<RobotClass>
{
    public EventGamepad gp1, gp2;

    public BaseTeleOpMode(RobotClass robot) {
        super(robot);
    }


    // Called over and over until stop button is pressed
    protected abstract void teleOpLoop() throws InterruptedException;

    @Override
    public void teamInit() {
        gp1 = new EventGamepad("GP1", super.gamepad1);
        gp2 = new EventGamepad("GP2", super.gamepad2);


        telemetry.addLine("GP1:")
                .addData("", new Func<String>() {
                    @Override public String value() {
                        return robot.saveTelemetryData("GP1", gp1.toString());
                    }});

        telemetry.addLine("GP2:")
                .addData("", new Func<String>() {
                    @Override public String value() {
                        return robot.saveTelemetryData("GP2", gp1.toString());
                    }});
    }


    // First TeleOp thing that happens after play button is pressed
    protected abstract void teleOpStart();

    // Convert LinearOpMode into OpMode
    @Override
    final protected void teamRun() throws InterruptedException
    {
        teleOpStart();

        while (runSchedulerAndRobot())
        {
            teleOpLoop();
        }
    }

}
