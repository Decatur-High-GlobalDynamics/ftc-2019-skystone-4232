package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="ForwardTest", group="Linear")
public class TurnToTest extends LinearOpMode {

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 4.0/2.5 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = 1;
    static final double     INCHES_PER_DEGREE       = 5.0/90.0;
    TeamRobot robot = new TeamRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        //Encoder ticks to test
        moveInches(500, 500, 0.5, 0.5);
    }
    public void moveInches(double rightInches, double leftInches, double speed, double powerPerSecond) {
        runtime.reset();
        int leftTarget;
        int rightTarget;
        double rightDirection;
        double leftDirection;
        double rightMiddle = rightInches/2;
        double leftMiddle = leftInches/2;
        //Calculate Target
        if (rightInches > 0) {
            rightDirection = -1;
        } else {
            rightDirection = 1;
        }
        if (leftInches > 0) {
            leftDirection = -1;
        } else {
            leftDirection = 1;
        }
        leftTarget = robot.leftDrive.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
        rightTarget = robot.rightDrive.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
        //Set targets to Drives
        robot.leftDrive.setTargetPosition(leftTarget);
        robot.rightDrive.setTargetPosition(rightTarget);
        //Set Drives to run to position
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Reset time and run motion
        double currentRightPower = powerPerSecond * (runtime.milliseconds()/1000);
        double currentLeftPower = powerPerSecond * (runtime.milliseconds()/1000);
        double previousTime = runtime.milliseconds()/1000;
        robot.leftDrive.setPower(leftDirection * Range.clip(currentLeftPower, 0,Range.clip(Math.abs(speed), 1.0, -1.0)));
        robot.rightDrive.setPower(rightDirection * Range.clip(currentRightPower, 0,Range.clip(Math.abs(speed), 1.0, -1.0)));
        //Loop until done or at position
        while (robot.leftDrive.isBusy() || robot.rightDrive.isBusy()) {
            if (Math.abs(rightTarget - robot.rightDrive.getCurrentPosition()) < rightMiddle) {
                currentRightPower += rightDirection * ((runtime.milliseconds()/1000) - (previousTime)) * powerPerSecond;
            } else {
                currentRightPower -= rightDirection * ((runtime.milliseconds()/1000) - (previousTime)) * powerPerSecond;
            }
            robot.rightDrive.setPower(rightDirection * Range.clip(currentRightPower, 0,Range.clip(Math.abs(speed), 1.0, -1.0)));
            if (Math.abs(leftTarget - robot.leftDrive.getCurrentPosition()) < leftMiddle) {
                currentLeftPower += leftDirection * ((runtime.milliseconds()/1000) - (previousTime)) * powerPerSecond;
            } else {
                currentLeftPower -= leftDirection * ((runtime.milliseconds()/1000) - (previousTime)) * powerPerSecond;
            }
            robot.leftDrive.setPower(leftDirection * Range.clip(currentLeftPower, 0,Range.clip(Math.abs(speed), 1.0, -1.0)));
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
