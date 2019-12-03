package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.scheduler.BaseTeleOpMode;
import org.firstinspires.ftc.teamcode.scheduler.Utils;

@TeleOp(name="TeleOp1")
public class TeleOP1 extends BaseTeleOpMode <TeamRobot>
{
    public TeleOP1(){
        super(new TeamRobot());
    }


    @Override
    protected void teleOpLoop() throws InterruptedException {
        robot.setDrivingPowers_teleop(gp1.left_stick_y, gp1.right_stick_y);
        if (gp2.left_bumper.isPressed) {
            robot.setIntakePower(1);
            robot.setHoldServoPosition(0);
        } else if (gp2.right_bumper.isPressed) {
            robot.setIntakePower(-1);
            robot.setHoldServoPosition(0);
        } else if (gp2.right_trigger > 0.5) {
            robot.setHoldServoPosition(1);
            robot.setIntakePower(0);
        } else {
            if (robot.blockGrabberServo.getPower() > 0) robot.setHoldServoPosition(0.5);
            else robot.setHoldServoPosition(0);
            robot.setIntakePower(0);
        }



        //if(gp2.left_bumper.isPressed){
            robot.setGateServoPosition(-gp2.left_stick_y);
        //}
        if (gp2.a.onPress) {
            robot.currentPos = TeamRobot.ARM_POSITION.DOWN;
            robot.armRaiseMotor.setTargetPosition(robot.startArmPos);
        } else if (gp2.b.onPress) {
            robot.currentPos = TeamRobot.ARM_POSITION.MID;
            robot.armRaiseMotor.setTargetPosition(robot.startArmPos + TeamRobot.MEDIUM_POS);
        } else if (gp2.y.onPress) {
            robot.currentPos = TeamRobot.ARM_POSITION.TOP;
            robot.armRaiseMotor.setTargetPosition(robot.startArmPos + TeamRobot.TOP_POS);
        }

        if (gp1.a.onPress) {
            ArmPositionCommand.upSpeedTest += 0.05;
        } else if (gp1.y.onPress) {
            ArmPositionCommand.upSpeedTest += 0.05;
        }

        if (gp1.b.onPress) {
            ArmPositionCommand.downSpeedTest -= 0.05;
        } else if (gp1.x.onPress) {
            ArmPositionCommand.downSpeedTest += 0.05;
        }

        robot.armRaiseMotor.setTargetPosition(robot.armRaiseMotor.getTargetPosition() + (int)(gp2.right_stick_y * 15));

        // GP2 - DPAD-UP/DOWN: grabber

        if (gp2.dpad_down.isPressed)
            robot.grabBlock();
        else if (gp2.dpad_up.isPressed)
            robot.releaseBlock();

        if (gp2.a.onPress) {
            robot.moveArmDown();
        } else if (gp2.y.onPress) {
            robot.moveArmUp();
        }

        // GP2 - DPAD-LEFT/RIGHT: armSwing
        if (gp2.dpad_left.isPressed)
            robot.setArmSwingServoPosition(robot.armSwingServo.getPosition()-0.05);
        if (gp2.dpad_right.isPressed)
            robot.setArmSwingServoPosition(robot.armSwingServo.getPosition()+0.05);



    }

    @Override
    protected void teleOpStart()
    {
        new ArmPositionCommand("Arm position setter", robot).start();
    }
}
