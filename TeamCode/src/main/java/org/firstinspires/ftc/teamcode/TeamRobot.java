package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.scheduler.Robot;
import org.firstinspires.ftc.teamcode.scheduler.TeamImu;

public class TeamRobot extends Robot
{
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public CRServo gateServo;
    public CRServo wheelServoLeft;
    public CRServo wheelServoRight;

    double previousGatePosition;
    double previousIntakeLeftPostition;
    double previousIntakeRightPosition;

    public static TeamRobot get(){
        return (TeamRobot) Robot.sharedInstance;
    }

    public TeamRobot(){
    }

    public void init(Telemetry telemetry, HardwareMap hardwareMap, BaseLinearOpMode baseLinearOpMode) {
        super.init(telemetry, hardwareMap, baseLinearOpMode);
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        gateServo = hardwareMap.crservo.get("gate");
        wheelServoLeft = hardwareMap.crservo.get("wheel_left");
        wheelServoRight = hardwareMap.crservo.get("wheel_right");
        previousLeftWheelPosition = leftDrive.getCurrentPosition();
        previousRightWheelPosition = rightDrive.getCurrentPosition();
        previousGatePosition = gateServo.getController().getServoPosition(0);
        previousIntakeLeftPostition = wheelServoLeft.getController().getServoPosition(2);
        previousIntakeRightPosition = wheelServoRight.getController().getServoPosition(1);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        wheelServoLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelServoRight.setDirection(DcMotor.Direction.FORWARD);
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

    @Override
    public void protectRobot() {
        if (gateServo.getPower() > 0.05 && ((Math.abs(gateServo.getController().getServoPosition(0) - previousGatePosition) < 0.001) || (Math.abs(gateServo.getController().getServoPosition(0))) > 0.99)) {
            gateServo.setPower(0);
            telemetry.addData("Gate Servo: ", "stopped");
        }
        if (wheelServoLeft.getPower() > 0.05 && (Math.abs()))
    }
}