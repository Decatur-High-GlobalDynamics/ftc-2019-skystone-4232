package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.scheduler.Robot;
import org.firstinspires.ftc.teamcode.scheduler.Scheduler;
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

    public static enum GRAB_POSITION {RELEASE, GRAB};

    int startArmPos;
    static final int MEDIUM_POS = 140;
    static final int TOP_POS = 329;

    static final int GRAB_POS = 40;

    public ARM_POSITION currentPos = ARM_POSITION.DOWN;

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armRaiseMotor;

    public Servo armSwingServo;
    public CRServo blockGrabberServo;

    public TouchSensor armTouch;

    public DcMotor foundGrab;
    public CRServo intakeServoLeft;
    public CRServo intakeServoRight;

    public Servo holdServo;

    public Servo sweepLeft;
    public Servo sweepRight;

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

        foundGrab = hardwareMap.dcMotor.get("tray_grabber");
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
        armRaiseMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRaiseMotor.setTargetPosition(startArmPos);
        //This power isn't actually used unless you aren't at position
        //armRaiseMotor.setPower(1);

        armTouch = hardwareMap.touchSensor.get("arm_touch");

        //holdServo = hardwareMap.servo.get("hold");
        //setHoldServoPosition(0.5);

        sweepLeft = hardwareMap.servo.get("sweep_left");
        sweepLeft.setPosition(-1);
        sweepRight = hardwareMap.servo.get("sweep_right");
        sweepRight.setPosition(1);

        // Make sure gate arm is out of the way
        foundGrab.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        foundGrab.setPower(-0.3);
        int prevLoc = foundGrab.getCurrentPosition();
        while (true) {
            try {
                Scheduler.get().sleep(20, "Trying to setup found grabber");
            } catch (Exception e) {

            }
            if (Math.abs(foundGrab.getCurrentPosition() - prevLoc) < 2) {
                break;
            }
            prevLoc = foundGrab.getCurrentPosition();
        }
        foundGrab.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foundGrab.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //foundGrab.setTargetPosition(0);
        //foundGrab.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        foundGrab.setPower(0);
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

        telemetry.addLine("Arm: ")
                .addData("", new Func<String>() {
                    @Override
                    public String value()
                    {
                        return saveTelemetryData("Arm", "|SwingPos=%.2f|GrabberPow=%.2f(%s)|LiftPos=%d|LiftPow=%.2f|Touch Sensor Status= %s",
                                armSwingServo.getPosition(), blockGrabberServo.getPower(),
                                blockGrabberServo.getPower()>0.5 ? "release" : "grab",
                                armRaiseMotor.getCurrentPosition(),
                                armRaiseMotor.getPower(),
                                armTouch.isPressed()?"Touch sensor pressed":"Touch sensor not pressed");
                    }});

        telemetry.addLine("Foundation: ")
                .addData("", new Func<String>() {
                    @Override
                    public String value()
                    {
                        return saveTelemetryData("Arm", "|currentpos=%d|Power=%.2f|targetpos=%d",
                                foundGrab.getCurrentPosition(),
                                foundGrab.getPower(),
                                foundGrab.getTargetPosition());
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
        if (armTouch.isPressed() && armRaiseMotor.getPower() < 0) {
            alert("Stopping arm motor because it is too low");
            armRaiseMotor.setPower(0);
        }
        /*if (armRaiseMotor.getTargetPosition() > startArmPos + MEDIUM_POS && armSwingServo.getPosition() < 0.5) {
            alert("Stopping arm motor because the block is not out");
            armRaiseMotor.setPower(0);
            armRaiseMotor.setTargetPosition(startArmPos + MEDIUM_POS);
        }*/
//        if (gateServo.getPower() > 0.05 && Math.abs(gateServoSpeed) < 0.001) {
//            alert("GateServo is stuck: power=%.2f, speed=%.3f", gateServo.getPower(), gateServoSpeed);
//            setGateServoPosition(0);
//        }
    }

    public void setTrayGrabberPosition(GRAB_POSITION pos)
    {
        if (pos == GRAB_POSITION.GRAB) {
            foundGrab.setTargetPosition(GRAB_POS);
            foundGrab.setPower(1);
        } else {
            foundGrab.setTargetPosition(0);
            foundGrab.setPower(-1);
        }
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

    public void setHoldServoPosition(double position) {
        documentComponentStatus("Hold Servo", "Setting Hold Servo position to %.2f", position);
        //holdServo.setPosition(position);
    }
}