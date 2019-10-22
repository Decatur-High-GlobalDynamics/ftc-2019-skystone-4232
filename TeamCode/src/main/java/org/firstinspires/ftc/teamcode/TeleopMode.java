package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Main Teleop", group="Iterative Opmode")

public class TeleopMode extends OpMode {
    Hardware robot = new Hardware();

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

        if (gamepad1.right_bumper) {
            robot.gateServo.setPower(1);
        } else if (gamepad1.left_bumper)
        {
            robot.gateServo.setPower(0);
        }
    }
}
