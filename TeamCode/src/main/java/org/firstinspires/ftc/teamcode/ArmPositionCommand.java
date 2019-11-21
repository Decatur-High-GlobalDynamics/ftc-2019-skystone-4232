package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.scheduler.OngoingAction;
import org.firstinspires.ftc.teamcode.scheduler.Utils;

public class ArmPositionCommand extends OngoingAction {
    static double UP_SPEED = 1.0;
    static double DOWN_SPEED = -0.3;
    static double P = 1/10;
    static double HOLD_SPEED = 0.3;
    static double D = 0;
    int previousError = 0;
    TeamRobot robot;

    public ArmPositionCommand(String label, TeamRobot robot) {super(label); this.robot = robot;}

    @Override
    protected void loop() {
        int targetPos = robot.armRaiseMotor.getTargetPosition();
        int currentPos = robot.armRaiseMotor.getCurrentPosition();

        if (Math.abs(currentPos - targetPos) < 5) {
            if (Math.abs(currentPos - robot.startArmPos) > 10) {
                robot.armRaiseMotor.setPower(HOLD_SPEED);
            }
        } else {
            int error = targetPos - currentPos;
            double deriv = error - previousError;
            robot.armRaiseMotor.setPower(Utils.clipValue("arm power", error * P + deriv * D, DOWN_SPEED, UP_SPEED));
        }
    }
}
