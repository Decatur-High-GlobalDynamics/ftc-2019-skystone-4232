package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.scheduler.OngoingAction;
import org.firstinspires.ftc.teamcode.scheduler.Utils;

public class ArmPositionCommand extends OngoingAction {
    final double UP_SPEED = 0.2;
    final double DOWN_SPEED = 0.01;
    static double P = 1/10;
    static double HOLD_SPEED = 0.2;
    //TODO: Test this empirically by setting power to 1 when in run_using_encoder mode
    static double MOTOR_MAX_SPEED = 1120;
    TeamRobot robot;

    public ArmPositionCommand(String label, final TeamRobot robot) {
        super(label);
        this.robot = robot;
        robot.telemetry.addLine("ArmControl: ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.saveTelemetryData("ArmControl",
                                "TargetPos: %d|CurrentPos: %d|Mode: %s|Pow: %.1f|DownPow: %.1f|HoldPow: %.1f|UpPow: %.1f",
                                robot.armRaiseMotor.getTargetPosition(),
                                robot.armRaiseMotor.getCurrentPosition(),
                                robot.armRaiseMotor.getMode().toString().toLowerCase(),
                                robot.armRaiseMotor.getPower(),
                                UP_SPEED, HOLD_SPEED, DOWN_SPEED
                                );
                    }
                });

    }

    @Override
    protected void loop() {
        int targetPos = robot.armRaiseMotor.getTargetPosition();
        int currentPos = robot.armRaiseMotor.getCurrentPosition();

        if (robot.armTouch.isPressed() && targetPos < robot.startArmPos + 20){
            robot.armRaiseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.startArmPos = robot.armRaiseMotor.getCurrentPosition();
            robot.armRaiseMotor.setTargetPosition(robot.startArmPos);
            robot.armRaiseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
        if (Math.abs(currentPos - targetPos) < 5 && !robot.armTouch.isPressed()) {
            robot.armRaiseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.armRaiseMotor.setPower(HOLD_SPEED);
        } else if (currentPos < targetPos) {
            robot.armRaiseMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.armRaiseMotor.setPower(UP_SPEED);
        } else if (targetPos < currentPos) {
            robot.armRaiseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.armRaiseMotor.setPower(DOWN_SPEED);
        }}
    }
}
