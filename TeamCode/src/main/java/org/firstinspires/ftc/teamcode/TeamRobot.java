package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.scheduler.BaseLinearOpMode;
import org.firstinspires.ftc.teamcode.scheduler.Robot;

public class TeamRobot extends Robot
{
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public CRServo gateServo;

    public static TeamRobot get(){
        return (TeamRobot) Robot.sharedInstance;
    }

    public TeamRobot(){
    }

    public void init() {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        gateServo = hardwareMap.crservo.get("gate");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        stopDrivingWheels_raw();
        setDrivingMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public DcMotor getLeftMotor()
    {
        return leftDrive;
    }

    @Override
    public DcMotor getRightMotor()
    {
        return rightDrive;
    }


    @Override
    public double getDriveWheelEncoderClicksPerInch()
    {
        // TODO: TBD
        return 50;
    }

    @Override
    public String getRobotTag()
    {
        return "Team4232";
    }

}