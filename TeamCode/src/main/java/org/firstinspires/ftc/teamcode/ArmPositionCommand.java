package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.scheduler.OngoingAction;
import org.firstinspires.ftc.teamcode.scheduler.Utils;

public class ArmPositionCommand extends OngoingAction {
    final double UP_SPEED = 1.0;
    static public double upSpeedTest = 0;
    static public double downSpeedTest = 0;
    final double DOWN_SPEED = 0.5;
    static double P = 1/10;
    static double HOLD_SPEED = 0.2;
    //TODO: Test this empirically by setting power to 1 when in run_using_encoder mode
    static double MOTOR_MAX_SPEED = 1120;
    TeamRobot robot;

    public ArmPositionCommand(String label, TeamRobot robot) {super(label); this.robot = robot;}

    @Override
    protected void loop() {
        int targetPos = robot.armRaiseMotor.getTargetPosition();
        int currentPos = robot.armRaiseMotor.getCurrentPosition();
        int error = targetPos - currentPos;

        if (robot.armTouch.isPressed() && targetPos < 5){
            robot.armRaiseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.startArmPos = robot.armRaiseMotor.getCurrentPosition();
            robot.armRaiseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (Math.abs(currentPos - targetPos) < 5) {
            robot.armRaiseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.armRaiseMotor.setPower(HOLD_SPEED);
        } else if (currentPos < targetPos) {
            robot.armRaiseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armRaiseMotor.setPower(upSpeedTest);
        } else if (targetPos > currentPos) {
            robot.armRaiseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armRaiseMotor.setPower(downSpeedTest);
        }
    }
}
