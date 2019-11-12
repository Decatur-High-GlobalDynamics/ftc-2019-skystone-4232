package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Main Teleop", group="Iterative Opmode")

public class TeleopMode extends OpMode {
    TeamRobot robot = new TeamRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "initialized");
    }

    @Override
    public void loop() {
        double leftPwr;
        double rightPwr;

        leftPwr = gamepad1.left_stick_y;
        rightPwr = gamepad1.right_stick_y;

        robot.leftDrive.setPower(leftPwr);
        robot.rightDrive.setPower(rightPwr);

        if (gamepad2.a) {
            robot.foundationServo.setPower(1);
        } else if (gamepad2.b) {
            robot.foundationServo.setPower(-1);
        } else {
            robot.foundationServo.setPower(0);
        }

        if (gamepad2.right_bumper) {
            robot.wheelServoLeft.setPower(1);
            robot.wheelServoRight.setPower(1);
        } else if (gamepad2.left_bumper)
        {
            robot.wheelServoLeft.setPower(-1);
            robot.wheelServoRight.setPower(-1);
        } else {
            robot.wheelServoLeft.setPower(0);
            robot.wheelServoRight.setPower(0);
        }
    }
}
