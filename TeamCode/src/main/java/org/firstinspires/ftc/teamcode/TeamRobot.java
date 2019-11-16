package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.scheduler.Robot;
import org.firstinspires.ftc.teamcode.scheduler.Utils;

public class TeamRobot extends Robot
{
    // TODO: These need to be determined by experiment
    static final double GATE_ARM_EXTENDED_POSITION=0.82;
    static final double GATE_ARM_FOLDED_POSITION=0.0;
    static final double GATE_ARM_HORIZONTAL_POSITION=0.7;

    static final double ARM_SWING_FOLDED_POSITION=0;
    static final double ARM_SWING_EXTENDED_POSITION=1.0;

    static final double GRABBER_GRAB_POWER=0.10;
    static final double GRABBER_RELEASE_POWER=-0.10;

    public static enum ARM_POSITION {DOWN, MID, TOP};

    int startArmPos;
    static final int MEDIUM_POS = 120;
    static final int TOP_POS = 257;

    public ARM_POSITION currentPos = ARM_POSITION.DOWN;

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armRaiseMotor;

    public Servo armSwingServo;
    public CRServo blockGrabberServo;


    public Servo gateServo;
    public CRServo intakeServoLeft;
    public CRServo intakeServoRight;

    double previousIntakeLeftPostition, previousIntakeRightPosition;
    double intakeLeftSpeed, intakeRightSpeed;

    public static TeamRobot get(){
        return (TeamRobot) Robot.sharedInstance;
    }

    public TeamRobot(){
    }

    /**
     * Set the intake wheels spinning. Positive power is intake
     * @param power
     */
    public void setIntakePower(double power){
        intakeServoLeft.setPower(power);
        intakeServoRight.setPower(power);
        documentComponentStatus("Intake Servos","SetIntakePowers(%.2f,%s)",
                power, power>0?"in":"out");
    }

    // This is called by BaseLinearOpMode
    protected void teamInit() {
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        setDrivingMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stopDrivingWheels_raw();

        gateServo = hardwareMap.servo.get("gate");
        intakeServoLeft = hardwareMap.crservo.get("intake_left");
        intakeServoRight = hardwareMap.crservo.get("intake_right");
        intakeServoLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeServoRight.setDirection(DcMotor.Direction.FORWARD);

        blockGrabberServo = hardwareMap.crservo.get("block_grabber");
        armSwingServo = hardwareMap.servo.get("arm_swing");
        armRaiseMotor = hardwareMap.dcMotor.get("arm");
        armRaiseMotor.setPower(0);
        //Reset encoder
        armRaiseMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Get position just in case
        startArmPos = armRaiseMotor.getCurrentPosition();
        //Run to position is super cool. It ensures that you always go to position
        armRaiseMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRaiseMotor.setTargetPosition(startArmPos);
        //This power isn't actually used unless you aren't at position
        armRaiseMotor.setPower(1);

        // Make sure gate arm is out of the way
        setGateServoPosition(GATE_ARM_FOLDED_POSITION);
        setArmSwingServoPosition(ARM_SWING_FOLDED_POSITION);
        releaseBlock();
    }

    @Override
    protected void setupRobotTelemetry(Telemetry telemetry)
    {
        super.setupRobotTelemetry(telemetry);
        telemetry.addLine("Intake: ")
                .addData("", new Func<String>() {
                    @Override
                    public String value()
                    {
                        return saveTelemetryData("Intake", "|left-pwr=%.1f/spd=%.2f|right-pwr==%.1f/spd=%.2f",
                                intakeServoLeft.getPower(), intakeLeftSpeed,
                                intakeServoRight.getPower(), intakeRightSpeed);
                    }});

        telemetry.addLine("Gate: ")
                .addData("", new Func<String>() {
                    @Override
                    public String value()
                    {
                        return saveTelemetryData("Gate", "|pos=%.2f",
                                gateServo.getPosition());
                    }});
        telemetry.addLine("Arm: ")
                .addData("", new Func<String>() {
                    @Override
                    public String value()
                    {
                        return saveTelemetryData("Arm", "|SwingPos=%.2f|GrabberPow=%.2f(%s)|LiftPos=%d|LiftPow=%.1f",
                                armSwingServo.getPosition(), blockGrabberServo.getPower(),
                                blockGrabberServo.getPower()>0.5 ? "release" : "grab",
                                armRaiseMotor.getCurrentPosition(),
                                armRaiseMotor.getPower());
                    }});
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
        return 79.9446494465;
    }

    @Override
    public String getRobotTag()
    {
        return "Team4232";
    }


    @Override
    protected void trackMovementSpeeds()
    {
        super.trackMovementSpeeds();
        double currentLeftIntakeServoPosition = getCRServoPosition(intakeServoLeft);
        double currentRightIntakeServoPosition = getCRServoPosition(intakeServoRight);
        intakeLeftSpeed = Utils.subtractWithWraparound("LeftIntakeServo", currentLeftIntakeServoPosition, previousIntakeLeftPostition, 0, 1);
        intakeRightSpeed = Utils.subtractWithWraparound("RightIntakeServo", currentRightIntakeServoPosition, previousIntakeRightPosition, 0, 1);
    }

    @Override
    public void protectRobot() {
        if (armRaiseMotor.getCurrentPosition() < 20 && armRaiseMotor.getPower() < 0) {
            alert("Stopping arm motor because it is too low");
            armRaiseMotor.setPower(0);
        }
//        if (gateServo.getPower() > 0.05 && Math.abs(gateServoSpeed) < 0.001) {
//            alert("GateServo is stuck: power=%.2f, speed=%.3f", gateServo.getPower(), gateServoSpeed);
//            setGateServoPosition(0);
//        }
    }

    public void setGateServoPosition(double position)
    {
        position= Utils.clipValue("GateServoPosition", position, GATE_ARM_FOLDED_POSITION, GATE_ARM_EXTENDED_POSITION);
        documentComponentStatus("GateServo", "Setting Gate position to %.2f", position);
        gateServo.setPosition(position);
    }

    public void setArmSwingServoPosition(double position)
    {
        position= Utils.clipValue("ArmSwingPosition", position, ARM_SWING_FOLDED_POSITION, ARM_SWING_EXTENDED_POSITION);
        documentComponentStatus("ArmSwingServo", "Setting ArmSwing position to %.2f", position);
        armSwingServo.setPosition(position);
    }

    public void grabBlock()
    {
        setBlockGrabberServoPower(GRABBER_GRAB_POWER);
    }

    public void releaseBlock()
    {
        setBlockGrabberServoPower(GRABBER_RELEASE_POWER);
    }

    public void setBlockGrabberServoPower(double power)
    {
        power= Utils.clipValue("BlockGrabberPower", power, -1.0, 1.0);
        documentComponentStatus("BlockGrabberServo", "Setting BlockGrabber power to %.2f", power);
        blockGrabberServo.setPower(power);
    }

    public void setArmPower(double power) {
        documentComponentStatus("ArmRaiseMotor", "SettingArmRaiseMotor power to %.2f", power);
        armRaiseMotor.setPower(power);
    }

    public void moveArmUp() {
        if (currentPos == ARM_POSITION.DOWN) {
            currentPos = ARM_POSITION.MID;
            armRaiseMotor.setTargetPosition(startArmPos + MEDIUM_POS);
        } else if (currentPos == ARM_POSITION.MID) {
            currentPos = ARM_POSITION.TOP;
            armRaiseMotor.setTargetPosition(startArmPos + TOP_POS);
        }
    }
    public void moveArmDown() {
        if (currentPos == ARM_POSITION.TOP) {
            currentPos = ARM_POSITION.MID;
            armRaiseMotor.setTargetPosition(startArmPos + MEDIUM_POS);
        } else if (currentPos == ARM_POSITION.MID) {
            currentPos = ARM_POSITION.DOWN;
            armRaiseMotor.setTargetPosition(startArmPos);
        }
    }
}