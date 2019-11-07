package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Push Foundation Left", group="Linear op mode")
public class PushFoundationAutoLeft extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 4.0/2.5 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     INCHES_PER_DEGREE       = 5.0/90.0;
    TeamRobot robot = new TeamRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        moveInches(5, 5, .25);
        moveInches(90 * INCHES_PER_DEGREE, -90 * INCHES_PER_DEGREE, 0.5);
        moveInches(48 * 2, 48 * 2, 0.75);
        moveInches(-90 * INCHES_PER_DEGREE, 90 * INCHES_PER_DEGREE, 0.5);
        moveInches((6 * 12) - 5, (6 * 12) - 5, 0.75);
        moveInches(75 * INCHES_PER_DEGREE, -75 * INCHES_PER_DEGREE, 0.5);
        moveInches(40, 20, 0.5);
        moveInches(-40, -20, 0.5);
        moveInches(-75 * INCHES_PER_DEGREE, 75 * INCHES_PER_DEGREE, 0.5);
        moveInches(10, 10, 0.5);
        moveInches(90 * INCHES_PER_DEGREE, -90 * INCHES_PER_DEGREE, 0.5);
        moveInches(26.75, 26.75, 0.5);
        moveInches(90 * INCHES_PER_DEGREE, -90 * INCHES_PER_DEGREE, 0.5);
        moveInches((48 * 2) + 10, (48 * 2) + 10, 0.75);
        moveInches(-10, -10, 0.5);
        moveInches(90 * INCHES_PER_DEGREE, -90 * INCHES_PER_DEGREE, 0.5);
        moveInches(74.75, 74.75, 0.25);
    }

    public void moveInches(double rightInches, double leftInches, double speed) {
        int leftTarget;
        int rightTarget;
        //Calculate Target
        leftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        rightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        //Set targets to Drives
        robot.leftDrive.setTargetPosition(leftTarget);
        robot.rightDrive.setTargetPosition(rightTarget);
        //Set Drives to run to position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Reset time and run motion
        robot.leftDrive.setPower(Range.clip(speed, 1.0, -1.0));
        robot.rightDrive.setPower(Range.clip(speed, 1.0, -1.0));
        //Loop until done or at position
        while (robot.leftDrive.isBusy() || robot.rightDrive.isBusy()) {
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
