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
            robot.armRaiseMotor.setPower(Utils.clipValue("arm raise power", robot.armRaiseMotor.getPower()-0.01, -1, 1));
        } else if (gp2.y.onPress) {
            robot.armRaiseMotor.setPower(Utils.clipValue("arm raise power", robot.armRaiseMotor.getPower()+0.01, -1, 1));
        }


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

    }
}
