package org.firstinspires.ftc.teamcode.scheduler;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.scheduler.Utils.log;
import static org.firstinspires.ftc.teamcode.scheduler.Utils.safeStringFormat;

public abstract class Robot
{
    public static enum TURN_TYPE {SPIN, PIVOT};

    // Found via experiment
    protected double TURN_POWER = 0.35;
    protected double TURN_SLOWDOWN_POWER = 0.05;
    protected int TURN_SLOWDOWN_DEGREES = 10;

    // When to start stopping turns
    protected double TURN_APPROXIMATION = 3;

    protected static Robot sharedInstance=null;

    protected HardwareMap hardwareMap;
    public BaseLinearOpMode opMode;

    private RobotVision robotVision;
    public Telemetry.Line visionLine;

    protected TeamImu teamImu;
    double correctHeading;

    // Used to measure robot motor movements/speeds/changes
        protected int previousLeftWheelPosition;
        protected int previousRightWheelPosition;
        int leftWheelSpeed;
        int rightWheelSpeed;
        double previousLeftMotorPower;
        double previousRightMotorPower;

    ////
    // Telemetry
        public Telemetry telemetry;
        private static final int MAX_ALERTS=2;

        // Used to log telemetry to RobotLog (in addition to driver station)
        Map<String, String> latestTelemetryData = new LinkedHashMap<>();
        long lastTelemetryLoggingTime_ms = 0;
        long lastTelemetryUpdateTime_ms = 0;
        private LinkedList<String> alerts = new LinkedList<>();
        private int alertCounter = 0;
        // What was logged most recently for each component... used to avoid duplicate messages
        private Map<String, String> component2MostRecentStatus = new HashMap<>();

    protected Robot() {
        sharedInstance = this;
    }

    /**
     * This is called by BaseLinearOpMode to get the robot wired into the scheduler
     * and to initialize itself. Specific motors, devices, etc should be initialized
     * in teamInit() which this method calls.
     * @param telemetry
     * @param hardwareMap
     * @param baseLinearOpMode
     */
    void init(Telemetry telemetry, HardwareMap hardwareMap, BaseLinearOpMode baseLinearOpMode)
    {
        this.telemetry   = telemetry;
        this.hardwareMap = hardwareMap;
        opMode = baseLinearOpMode;
        teamImu = new TeamImu(hardwareMap, telemetry);
        correctHeading = teamImu.getTotalDegreesTurned();
        teamInit();
    }

    // TODO... these are assuming a tank drive... We'll deal with this when we have something better
    public abstract DcMotor getLeftMotor();
    public abstract DcMotor getRightMotor();

    public abstract double getDriveWheelEncoderClicksPerInch();

    public abstract String getRobotTag();

    protected abstract void teamInit();

    /**
     *
     * @return how many encoder clicks the drive motors have moved. Positive ==> Forward, as
     * defined by the robot's orientation
     */

    public int getLeftWheelPosition() {
        return getLeftMotor().getCurrentPosition();
    }

    public int getRightWheelPosition() {
        return getRightMotor().getCurrentPosition();
    }


    // Something to use when we don't care which wheel we're measuring
    public int getWheelPosition() {
        return getLeftWheelPosition();
    }
    
    public void alert(String alertFormat, Object...alertArgs)
    {
        String alert = safeStringFormat("%d: %s", ++alertCounter, safeStringFormat(alertFormat, alertArgs));
        alerts.add(alert);
        log("ALERT: %s", alert);

        // Remove first message if there are too many
        while ( alerts.size() > MAX_ALERTS )
            alerts.remove(0);
    }

    public String saveTelemetryData(String key, String valueFormat, Object... valueArgs)
    {
        String value = safeStringFormat(valueFormat, valueArgs);
        String oldValue = latestTelemetryData.get(key);

        if ( oldValue == null || !value.equals(oldValue))
        {
            latestTelemetryData.put(key, value);
            lastTelemetryUpdateTime_ms = System.currentTimeMillis();
        }
        return value;
    }

    public void removeTelemetryData(String key)
    {
        latestTelemetryData.remove(key);
        lastTelemetryUpdateTime_ms = System.currentTimeMillis();
    }

    protected void setupRobotTelemetry(Telemetry telemetry)
    {
        telemetry.addLine("Alert1")
                .addData("", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return saveTelemetryData("Alert1",
                                alerts.size()<1 ? "none" : alerts.get(0));
                    }
                });
        telemetry.addLine("Alert2")
                .addData("", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return saveTelemetryData("Alert2",
                                alerts.size()<2 ? "none" : alerts.get(1));
                    }
                });

        visionLine = telemetry.addLine("Vision");

        telemetry.addLine("Heading: ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return saveTelemetryData("Heading","|TotDeg=%.1f|Correct=%.1f|Error=%.1f",
                                teamImu.getTotalDegreesTurned(),
                                correctHeading,
                                getHeadingError());
                    }});

        telemetry.addLine("Left: ")
                .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return saveTelemetryData("Left", "|p=%+.1f|s=+%d|Loc=%+d|Tgt=%+d",
                                getLeftMotor().getPower(),
                                leftWheelSpeed,
                                getLeftMotor().getCurrentPosition(),
                                getLeftMotor().getTargetPosition());
                    }});
        telemetry.addLine("Right: ")
                    .addData("", new Func<String>() {
                    @Override
                    public String value() {
                        return saveTelemetryData("Right", "|p=%+.1f|s=+%d|Loc=%+d|Tgt=%+d",
                                getRightMotor().getPower(),
                                rightWheelSpeed,
                                getRightMotor().getCurrentPosition(),
                                getRightMotor().getTargetPosition());
                    }});
    }

    /**
     * Note that a command was sent to a component. Call this every time you command a component
     * and do not worry about duplicate log entries; this method will automatically avoid such
     * duplicates.
     *
     * However, it helps a lot to round your values to tenths so micro changes are not logged.
     * In other words, use %.1f in your statusFormat instead of %f.
     *
     * @param component
     * @param statusFormat
     * @param args
     */
    public void documentComponentStatus(String component, String statusFormat, Object... args) {
        String currentStatus = safeStringFormat(statusFormat, args);

        String previousStatus = component2MostRecentStatus.get(component);

        if ( previousStatus==null || !previousStatus.equals(currentStatus) )
        {
            log("TeamRobot change: %s: %s (was %s)",
                    component, currentStatus,
                    previousStatus==null ? "unknown" : previousStatus);
            component2MostRecentStatus.put(component, currentStatus);
        }
    }

    public void stopDrivingWheels_raw()
    {
        documentComponentStatus("Driving", "Stopping");
        setRightPower_raw(0);
        setLeftPower_raw(0);
    }

    public void stopDrivingWheels_teleop()
    {
        if (Scheduler.get().isDcMotorBusy(getLeftMotor()) || Scheduler.get().isDcMotorBusy(getRightMotor()))
        {
            alert("Ignoring stopDrivingWheels_teleop while motor(s) are busy");
            return;
        }

        stopDrivingWheels_raw();
    }

    public EndableAction startStopping()
    {
        EndableAction brakingAction = new EndableAction( "Stop")
        {
            DcMotor.ZeroPowerBehavior originalBehavior = getLeftMotor().getZeroPowerBehavior();
            @Override
            public EndableAction start()
            {
                super.start();
                setDrivingZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setRightPower_raw(0);
                setLeftPower_raw(0);
                return this;
            }

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                int maxSpeed = Math.max(Math.abs(leftWheelSpeed), Math.abs(rightWheelSpeed));
                if (maxSpeed > 5)
                {
                    statusMessage.append(safeStringFormat("Waiting for L/R to stop: speeds are %d and %d", leftWheelSpeed, rightWheelSpeed ));
                    return false;
                }
                else
                {
                    statusMessage.append(safeStringFormat("TeamRobot wheels have stopped... Lspeed=%d|Rspeed=%d", leftWheelSpeed, rightWheelSpeed ));
                    return true;
                }
            }

            @Override
            protected void cleanup(boolean actionWasCompletedSuccessfully)
            {
                setDrivingZeroPowerBehavior(originalBehavior);
                super.cleanup(actionWasCompletedSuccessfully);
            }
        }.start();

        return brakingAction;
    }

    public void driveStraight_teleop(double power)
    {
        if (Scheduler.get().isDcMotorBusy(getLeftMotor()) || Scheduler.get().isDcMotorBusy(getRightMotor()))
        {
            if (Math.abs(power)>0.05)
                alert("Ignoring driveStraight_teleop while motor(s) are busy");
            return;
        }

        driveStraight_raw(power);
    }

    public void driveStraight_raw(double power)
    {
        documentComponentStatus("Driving", "Straight(pow=%.2f)", power);

        setLeftPower_raw(power);
        setRightPower_raw(power);
    }

    /**
     * Turn robot with opposite power to wheels
     *
     * @param power (-1..1) negative means to the left
     */
    public void spin_raw(double power)
    {
        documentComponentStatus("Driving", "Spin%s(%.2f)", power > 0 ? "Right" : "Left", power);
        setLeftPower_raw(power);
        setRightPower_raw(-power);
    }

    public void spin_teleop(double power)
    {
        if (Scheduler.get().isDcMotorBusy(getLeftMotor()) || Scheduler.get().isDcMotorBusy(getRightMotor()))
        {
            if (Math.abs(power)>0.05)
                alert("Ignoring spin_teleop while motor(s) are busy");
            return;
        }

        spin_raw(power);
    }

    public void setDrivingPowers_raw(double leftPower, double rightPower)
    {
        documentComponentStatus("Driving","SetDrivingPowers(%.2f,%.2f)", leftPower, rightPower);
        setLeftPower_raw(leftPower);
        setRightPower_raw(rightPower);
    }

    public void setDrivingPowers_teleop(double leftPower, double rightPower)
    {
        if (Scheduler.get().isDcMotorBusy(getLeftMotor()) || Scheduler.get().isDcMotorBusy(getRightMotor()))
        {
            if (Math.abs(leftPower)>0.05 || Math.abs(rightPower)>0.05)
                alert("Ignoring setDrivingPowers_teleop while motor(s) are busy");
            return;
        }

        setDrivingPowers_raw(leftPower, rightPower);
    }

    public DcMotor.ZeroPowerBehavior setDrivingZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)
    {
        DcMotor.ZeroPowerBehavior originalZeroPowerBehavior = getLeftMotor().getZeroPowerBehavior();

        documentComponentStatus("DrivingMotorMode", "setDrivingZeroPowerBehavior(%s)", behavior);
        getLeftMotor().setZeroPowerBehavior(behavior);
        getRightMotor().setZeroPowerBehavior(behavior);

        return originalZeroPowerBehavior;
    }


    /**
     *
     * @return How many degrees the robot has turned. Negative: To the right, Positive: to the Left
     */

    public double getTotalDegreesTurned() {
        return teamImu.getTotalDegreesTurned();
    }

    private void setLeftPower_raw(double leftPower)
    {
        documentComponentStatus(safeStringFormat("Left(Port%d)MotorPower", getLeftMotor().getPortNumber()),
            "Power(%.2f)", leftPower);

        getLeftMotor().setPower(leftPower);
    }

    public void setLeftPower_teleop(double power)
    {
        if (Scheduler.get().isDcMotorBusy(getLeftMotor()))
        {
            if (Math.abs(power)>0.05)
                alert("Ignoring setLeftPower_raw while motor(s) are busy");
            return;
        }

        setLeftPower_raw(power);
    }

    private void setRightPower_raw(double rightPower)
    {
        documentComponentStatus(safeStringFormat("Right(Port%d)MotorPower", getRightMotor().getPortNumber()),
                "Power(%.2f)", rightPower);

        getRightMotor().setPower(rightPower);
    }

    public void setRightPower_teleop(double power)
    {
        if (Scheduler.get().isDcMotorBusy(getRightMotor()))
        {
            if (Math.abs(power)>0.05)
                alert("Ignoring setRightPower_raw while motor(s) are busy");
            return;
        }

        setRightPower_raw(power);
    }

    /**
     * @param power    Positive power is forward, negative power is backwards (between -1, 1)
     * @param steering Positive steering is to the right, negative steering is to the left (between -1, 1)
     */
    void setPowerSteering_raw(double power, double steering)
    {
        documentComponentStatus("Driving", "Steer(pow=%.2f, steering=%.2f)", power, steering);

        double powerRight, powerLeft;

        if (steering > 0) {
            // Turning to right: powerLeft should be more than powerRight
            // Scale down powerRight
            powerRight = (1 - Math.abs(steering)) * power;
            powerLeft = power;
        } else {
            // Turning to left: powerLeft should be less than powerRight
            // Scale down powerLeft
            powerLeft = (1 - Math.abs(steering)) * power;
            powerRight = power;
        }

        setLeftPower_raw(powerLeft);
        setRightPower_raw(powerRight);
    }

    public void setPowerSteering_teleop(double power, double steering)
    {
        if (Scheduler.get().isDcMotorBusy(getLeftMotor()) || Scheduler.get().isDcMotorBusy(getRightMotor()))
        {
            if (Math.abs(power)>0.05)
                alert("Ignoring setPowerSteering_teleop while motor(s) are busy");
            return;
        }

        setPowerSteering_raw(power, steering);
    }

    protected void trackMovementSpeeds()
    {
        int currentLeftWheelPosition = getLeftWheelPosition();
        int currentRightWheelPosition = getRightWheelPosition();

        leftWheelSpeed = currentLeftWheelPosition - previousLeftWheelPosition;
        rightWheelSpeed = currentRightWheelPosition - previousRightWheelPosition;


        previousLeftWheelPosition = currentLeftWheelPosition;
        previousRightWheelPosition = currentRightWheelPosition;

        previousLeftMotorPower = getLeftMotor().getPower();
        previousRightMotorPower = getRightMotor().getPower();
    }

    // Protect robot
    protected void protectRobot() {
    }

    public EndableAction startInchMove(final double inches, final double power)
    {
        return new EndableAction("InchMove", "InchMove(%.2f inches, %.2f power", inches, power)
        {
            int startPosition = getWheelPosition();
            int encoderClicks = (int) Math.round(inches * getDriveWheelEncoderClicksPerInch());
            int stopPosition = getWheelPosition() - encoderClicks;
            EndableAction stopAction=null;

            @Override
            public EndableAction start()
            {
                log("Starting to move for %d encoder clicks", encoderClicks);
                return super.start();
            }

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                if (getWheelPosition() < stopPosition)
                    return false;
                if ( stopAction == null )
                    return false;

                return stopAction.hasFinished();
            }

            @Override
            protected void cleanup(boolean actionWasCompletedSuccessfully)
            {
                stopDrivingWheels_raw();

                super.cleanup(actionWasCompletedSuccessfully);
            }

            @Override
            public void loop() throws InterruptedException
            {
                // Stop when we've gone far enough
                if (getWheelPosition() <= stopPosition)
                {
                    if (stopAction==null)
                        stopAction= startStopping();

                    return;
                }
                // avoid skidding by using less power for first 10 inches
                double wheelPower;
                if ( getWheelPosition() - startPosition < 10*getDriveWheelEncoderClicksPerInch() )
                    wheelPower = -0.3;
                else
                    wheelPower = -power;

                // Example:
                //   correctHeading: 0  (straight ahead)
                //   currentHeading: -5 (negative is to the right)
                //   HeadingError: 5 ==> Need to turn 5 degrees to the left
                double headingError = getHeadingError();

                setStatus("%d clicks, %.1f inches to go. Heading error: %.1f degrees (robot facing too far %s)",
                        stopPosition - getWheelPosition(),
                        (stopPosition - getWheelPosition()) / getDriveWheelEncoderClicksPerInch(),
                        headingError,
                        headingError > 0 ? "right" : "left");

                if ( headingError > 10 )
                    // Too far off course, need to stop and turn
                    // Use 0 degrees so turn will just turn to the correct place
                    startTurningLeft(0, TURN_TYPE.SPIN).waitUntilFinished();
                else if ( headingError < -10 )
                    startTurningRight(0, TURN_TYPE.SPIN).waitUntilFinished();
                else
                {
                    if (headingError > 0.0)
                    {
                        //Steer to the left
                        // PowerSteering takes negative steering to the left
                        setPowerSteering_raw(wheelPower, -0.05 * Math.abs(headingError));
                    } else if (headingError < 0.0)
                    {
                        //Steer to the right
                        // PowerSteering takes positive steering to the right

                        setPowerSteering_raw(wheelPower, 0.05 * Math.abs(headingError));
                    } else
                    {
                        // Go Straight
                        driveStraight_raw(wheelPower);
                    }
                }
            }
        }.start();
    }

    public double getCRServoPosition(CRServo servo) {
        return servo.getController().getServoPosition(servo.getPortNumber());
    }

    /**
     * How far off is the robot from the correct heading?
     * @return Degrees: < 0 ==> TeamRobot needs to turn Right, >0 ==> TeamRobot needs to turn Left
     */
    private double getHeadingError() {
        return correctHeading - getTotalDegreesTurned();
    }

    public EndableAction startInchMoveBack(final double inches, final double power)
    {
        return new EndableAction("InchMoveBack", "InchmoveBack(d=%.1fin, pow=%.1f)",
                inches, power)
        {
            int startPosition = getLeftWheelPosition();

            int encoderClicks = (int)(inches * getDriveWheelEncoderClicksPerInch());
            int stopPosition = getWheelPosition() + encoderClicks;
            EndableAction stopAction=null;

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                if (getWheelPosition() > stopPosition)
                    return false;
                if ( stopAction == null )
                    return false;

                return stopAction.hasFinished();
            }

            @Override
            public void loop() throws InterruptedException
            {
                // Stop when we've gone far enough
                if (getWheelPosition() >= stopPosition)
                {
                    if (stopAction==null)
                        stopAction= startStopping();

                    return;
                }

                // avoid skidding by using less power for first 10 inches
                double wheelPower;
                if ( startPosition - getWheelPosition() < 10 * getDriveWheelEncoderClicksPerInch() )
                    wheelPower = -0.3;
                else
                    wheelPower = -power;

                //Heading is larger to the left
                // Example:
                //   correctHeading: 0  (straight ahead)
                //   currentHeading: -5 (negative is to the right)
                //   HeadingError: 5 ==> Need to turn 5 degrees to the left

                double headingError = getHeadingError();

                setStatus(String.format("%.1f inches to go. Heading error: %.1f degrees(robot front facing too far %s)",
                        (stopPosition - getWheelPosition()) / getDriveWheelEncoderClicksPerInch(),
                        headingError,
                        headingError > 0 ? "right" : "left"
                ));


                if ( headingError > 10 ) {
                    setStatus("Turning left because we're off by %.1f degrees (>10)", headingError);
                    // Use 0 degrees so turn will just turn to the correct place
                    startTurningLeft(0, TURN_TYPE.SPIN).waitUntilFinished();
                }
                else if ( headingError < -10 ) {
                    setStatus("Turning right because we're off by %.1f degrees (<-10)", headingError);

                    // Use 0 degrees so turn will just turn to the correct place
                    startTurningRight(0, TURN_TYPE.SPIN).waitUntilFinished();
                }
                else
                {
                    setStatus("Using proportional steering because we're off by %.1f degrees", headingError);

                    if (headingError > 0.0) {
                        //The current heading is too small so we turn to the right
                        // (opposite of inchMoveForward because we're going backwards)
                        setPowerSteering_raw(-wheelPower, 0.1 * Math.abs(headingError));
                    } else if (headingError < 0.0) {
                        //Current heading is too big, so we steer to the left (again since we're going backwards)
                        setPowerSteering_raw(-wheelPower, -0.1 * Math.abs(headingError));
                    } else {
                        // Go Straight
                        driveStraight_raw(-wheelPower);
                    }
                }

            }


            @Override
            protected void cleanup(boolean actionWasCompletedSuccessfully)
            {
                stopDrivingWheels_raw();

                super.cleanup(actionWasCompletedSuccessfully);
            }

        }.start();
    }

    public void resetCorrectHeading(String reasonFormat, Object... reasonArgs)
    {
        String reason = safeStringFormat(reasonFormat, reasonArgs);
        documentComponentStatus("Heading", "ResetHeading(%.1f deg, %s)", getTotalDegreesTurned(), reason);
        correctHeading = getTotalDegreesTurned();
    }

    // default to a SPIN turn
    public EndableAction startTurningRight(final double degrees)
    {
        return startTurningRight(degrees, TURN_TYPE.SPIN);
    }

    public EndableAction startTurningRight(final double degrees, final TURN_TYPE turnType)
    {
        return new EndableAction("TurnRight", "TurnRight(%.0f)", degrees)
        {
            DcMotor.RunMode originalMotorMode;
            DcMotor.ZeroPowerBehavior originalZeroPowerBehavior;
            EndableAction stopAction;

            @Override
            public EndableAction start()
            {
                super.start();
                correctHeading -= degrees;
                originalMotorMode = setDrivingMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                originalZeroPowerBehavior = setDrivingZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                return this;
            }

            @Override
            protected void cleanup(boolean actionWasCompletedSuccessfully)
            {
                setDrivingMotorMode(originalMotorMode);
                setDrivingZeroPowerBehavior(originalZeroPowerBehavior);
                stopDrivingWheels_raw();
                setStatus("Finished turning. Wheels stopped. TeamRobot is %.1f degrees off of correct heading", getHeadingError());

                super.cleanup(actionWasCompletedSuccessfully);
            }

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                statusMessage.append(safeStringFormat("DegreesToGo: %.0f, needs to be >=%.0f. Stop action: %s",
                        getHeadingError(), -TURN_APPROXIMATION, stopAction));

                if (getHeadingError() < -TURN_APPROXIMATION)
                    return false;
                else if (stopAction == null)
                    return false;
                else
                    return stopAction.hasFinished();
            }

            @Override
            public void loop() throws InterruptedException
            {
                super.loop();
                double degreesToGo = getHeadingError();

                if ( degreesToGo >= -TURN_APPROXIMATION )
                {
                    setStatus("%.0f degrees to go. (stopping)", degreesToGo);
                    if (stopAction == null)
                        stopAction = startStopping();
                }
                else if (degreesToGo >= -TURN_SLOWDOWN_DEGREES) {
                    setStatus("%.0f degrees to go. (slower turn)", degreesToGo);

                    if (turnType== TURN_TYPE.SPIN)
                        spin_raw(TURN_SLOWDOWN_POWER);
                    else
                        setDrivingPowers_raw(0, TURN_SLOWDOWN_POWER);
                } else {
                    setStatus("%.0f degrees to go. (full speed)", degreesToGo);
                    if (turnType== TURN_TYPE.SPIN)
                        spin_raw(TURN_POWER);
                    else
                        setDrivingPowers_raw(0, TURN_POWER);
                }
            }
        }.start();
    }

    // Default to a SPIN turn
    public EndableAction startTurningLeft(final double degrees)
    {
        return startTurningLeft(degrees, TURN_TYPE.SPIN);
    }

    public EndableAction startTurningLeft(final double degrees, final TURN_TYPE turnType)
    {
        return new EndableAction("TurnLeft", "TurnLeft(%.0f)", degrees)
        {
            DcMotor.RunMode originalMotorMode;
            EndableAction stopAction;

            @Override
            public EndableAction start()
            {
                super.start();
                correctHeading += degrees;
                originalMotorMode = setDrivingMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

                return this;
            }

            @Override
            protected void cleanup(boolean actionWasCompletedSuccessfully)
            {
                setStatus("Finished turning. Starting to stop when robot is %.0f degrees off of correct heading", getHeadingError());
                setDrivingMotorMode(originalMotorMode);
                stopDrivingWheels_raw();
                setStatus("Finished turning. Wheels stopped. TeamRobot is %.0f degrees off of correct heading", getHeadingError());

                super.cleanup(actionWasCompletedSuccessfully);
            }

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                statusMessage.append(safeStringFormat("DegreesToGo: %.0f, needs to be <%.0f. Stop action: %s",
                        getHeadingError(), TURN_APPROXIMATION, stopAction));

                if (getHeadingError() > TURN_APPROXIMATION)
                    return false;
                else if (stopAction == null)
                    return false;
                else
                    return stopAction.hasFinished();
            }

            @Override
            public void loop() throws InterruptedException
            {
                super.loop();
                double degreesToGo = getHeadingError();

                if ( degreesToGo <= TURN_APPROXIMATION )
                {
                    setStatus("%.0f degrees to go. (stopping)", degreesToGo);
                    if (stopAction == null)
                        stopAction = startStopping();
                }
                else if (degreesToGo <= TURN_SLOWDOWN_DEGREES) {
                    setStatus("%.0f degrees to go. (slower turn)", degreesToGo);

                    if (turnType== TURN_TYPE.SPIN)
                        spin_raw(-TURN_SLOWDOWN_POWER);
                    else
                        setDrivingPowers_raw(-TURN_SLOWDOWN_POWER, 0);
                } else {
                    setStatus("%.0f degrees to go. (full speed)", degreesToGo);
                    if (turnType== TURN_TYPE.SPIN)
                        spin_raw(-TURN_POWER);
                    else
                        setDrivingPowers_raw(-TURN_POWER, 0);
                }
            }
        }.start();
    }

    //Skooch
    public EndableAction startSkoochingRight()
    {
        return new EndableAction("SkoochRight")
        {
            boolean isDone = false;

            @Override
            public EndableAction start()
            {
                super.start();
                resetCorrectHeading("Skooching relative to where we were");
                return this;
            }

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                return isDone;
            }

            @Override
            protected void cleanup(boolean actionWasCompletedSuccessfully)
            {
                stopDrivingWheels_raw();
                super.cleanup(actionWasCompletedSuccessfully);
            }

            @Override
            public void loop() throws InterruptedException
            {
                waitFor(startInchMove(5,0.5));
                waitFor(startTurningRight(20, TURN_TYPE.SPIN));
                waitFor(startInchMove(5,0.5));
                waitFor(startTurningLeft(20, TURN_TYPE.SPIN));
                waitFor(startInchMoveBack(10,0.5));
                waitFor(startStopping());

                isDone = true;
            }
        }.start();
    }

    public EndableAction startSkoochingLeft()
    {
        return new EndableAction("SkoochLeft")
        {
            boolean isDone = false;

            @Override
            public EndableAction start()
            {
                super.start();
                resetCorrectHeading("Skooching relative to where we were");
                return this;
            }

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                return isDone;
            }

            @Override
            public void loop() throws InterruptedException
            {
                waitFor(startInchMove(5, 0.5));
                waitFor(startTurningLeft(20, TURN_TYPE.SPIN));
                waitFor(startInchMove(5, 0.5));
                waitFor(startTurningRight(20, TURN_TYPE.SPIN));
                waitFor(startInchMoveBack(10, 0.5));
                waitFor(startStopping());

                isDone = true;
            }
        }.start();
    }

    public void resetMotorEncoder(String motorName, DcMotor motor)
    {
        documentComponentStatus(motorName + "Motor-Encoder", "Reset encoder");

        DcMotor.RunMode originalMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(originalMode);
    }

    protected List<EndableAction> startCalibrationActions() {
        return new ArrayList<>();
    }

    public EndableAction startCalibratingEverything()
    {
        return new EndableAction("CalibrateEverything")
        {
            List<EndableAction> calibratingActions;
            @Override
            public EndableAction start()
            {
                super.start();
                calibratingActions = startCalibrationActions();
                return this;
            }

            @Override
            protected void cleanup(boolean actionWasCompletedSuccessfully)
            {
                for (EndableAction calibratingAction : calibratingActions) {
                    if (!calibratingAction.hasFinished())
                        calibratingAction.abort("startCalibratingEverything was aborted");
                }

                super.cleanup(actionWasCompletedSuccessfully);
            }

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                int numDone=0;
                for ( EndableAction calibratingAction : calibratingActions ) {
                    if ( calibratingAction.hasFinished() )  {
                        numDone++;
                        statusMessage.append(calibratingAction.label + ":done|");
                    } else {
                        statusMessage.append(calibratingAction.label + ":working|");
                    }
                }

                statusMessage.append(String.format("%d of %d done", numDone, calibratingActions.size()));

                if ( numDone != calibratingActions.size() )
                    return false;

                return areChildrenDone(statusMessage);
            }
        }.start();
    }

    public RobotVision getRobotVision()
    {
        if ( robotVision == null )
        {
            robotVision = new RobotVision(this);
            robotVision.init();
        }
        return robotVision;
    }

    public EndableAction startPushingIntoWall()
    {
        return new EndableAction("Pushing the wall")
        {
            int duration_ms=2500;

            @Override
            public EndableAction start()
            {
                super.start();
                setDrivingPowers_raw(0.25, 0.25);
                return this;
            }

            @Override
            public boolean isDone(StringBuilder statusMessage)
            {
                statusMessage.append(safeStringFormat("%.1f secs to go", duration_ms-getAge_ms()));
                return getAge_ms()>=duration_ms;
            }
        }.start();
    }

    public DcMotor.RunMode setDrivingMotorMode(DcMotor.RunMode newMode)
    {
        DcMotor.RunMode originalMode = getLeftMotor().getMode();

        documentComponentStatus("DrivingMotorMode", "Setting mode to %s (was %s)", newMode, originalMode);
        getLeftMotor().setMode(newMode);
        getRightMotor().setMode(newMode);

        return originalMode;
    }

}
