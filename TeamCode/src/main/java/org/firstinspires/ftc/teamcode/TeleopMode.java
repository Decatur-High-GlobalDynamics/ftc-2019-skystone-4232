package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

        leftPwr = -gamepad1.left_stick_y;
        rightPwr = -gamepad1.right_stick_y;

        robot.leftDrive.setPower(leftPwr);
        robot.rightDrive.setPower(rightPwr);
    }
}
