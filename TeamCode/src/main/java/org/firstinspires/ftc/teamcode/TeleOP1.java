package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.scheduler.BaseTeleOpMode;

@TeleOp(name="TeleOp1")
public class TeleOP1 extends BaseTeleOpMode <TeamRobot>
{
    public TeleOP1(){
        super(new TeamRobot());
    }

    @Override
    protected void teleOpLoop() throws InterruptedException
    {
        robot.setDrivingPowers_teleop(gp1.left_stick_y, gp1.right_stick_y);
        robot.setIntakePower(gp2.left_stick_y);

        if(gp2.left_bumper.isPressed){
            robot.setGateServoPosition(gp2.right_stick_x);
        }

        // GP2 - DPAD-UP/DOWN: grabber
        if (gp2.dpad_down.isPressed)
            robot.grabBlock();
        else if (gp2.dpad_up.isPressed)
            robot.releaseBlock();

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