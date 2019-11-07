package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Hardware {
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public CRServo foundationServo;
    public CRServo wheelServoLeft;
    public CRServo wheelServoRight;

    HardwareMap hwMap;

    public Hardware(){
    }

    public void init(HardwareMap hw) {
        hwMap = hw;
        leftDrive = hwMap.dcMotor.get("left_drive");
        rightDrive = hwMap.dcMotor.get("right_drive");
        foundationServo = hwMap.crservo.get("foundation");
        wheelServoLeft = hwMap.crservo.get("wheel_left");
        wheelServoRight = hwMap.crservo.get("wheel_right");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        wheelServoLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelServoRight.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}