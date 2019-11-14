package org.firstinspires.ftc.teamcode.scheduler;

public class Utils {
    public static String safeStringFormat(String format, Object... args) {
        try {
            String result = String.format(format, args);
            return result;
        } catch (RuntimeException e) {
            //TODO: Logging
            log_raw("Formatting Error: format=%s: %s", format, e.getMessage());
            e.printStackTrace();
            return String.format("Format Error: %s", format);
        }
    }

    public static void log(String format, Object... args){
        if (Action.getCurrentAction() != null)
        {
            Action.getCurrentAction().log(format, args);
        }
        else
        {
            log_raw(format, args);
        }
    }

    public static void log_raw(String format, Object... args)
    {
        if ( Robot.sharedInstance == null )
            System.err.println("TeamCode " + safeStringFormat(format, args));
        else
            System.err.println(Robot.sharedInstance.getRobotTag()+ " TeamCode " + safeStringFormat(format, args));

    }

    /**
     * Returns newSensorValue - oldSensorValue, but handles wrap-around by keeping result between -max/2 < result < max/2
     *
     * Gyro/IMU example, let's say we have a gyro that returns -180 <= x < 180, increasing to the right.
     *    If oldSensorValue is 179 and the gyro turns 3 degrees to the right, the new value is -178.
     *    This function will subtract -178 - 179 = -357 (very diferent than 3). This will see that -357 is smaller
     *    than -180, add 360 and return 3.
     *
     *  Servo example, let's say we have a servo that returns a position from 0 <= x < 1.0, increasing clockwise.
     *    If oldSensorValue is 0 and the servo turns 1/4 turn counterclockwise, the new value is 0.75.
     *    This function will subtract 0.75 - 0 = 0.75 (different than the 0.25 that was actually turned).
     *    This function will then subtract 0.75 - 1.0 = -0.25, indicating the 1/4 turn counterclockwise.
     *
     * @param newSensorValue
     * @param oldSensorValue
     * @param minPossibleSensorValue
     * @param maxPossibleSensorValue
     * @return
     */
    public static double subtractWithWraparound(String valueLabel, double newSensorValue, double oldSensorValue, double minPossibleSensorValue, double maxPossibleSensorValue) {
        // Make sure the sensor values are in expected range
        newSensorValue = clipValue(valueLabel+".new", newSensorValue, minPossibleSensorValue, maxPossibleSensorValue);
        oldSensorValue = clipValue(valueLabel+".old", oldSensorValue, minPossibleSensorValue, maxPossibleSensorValue);

        double possibleRange = Math.abs(maxPossibleSensorValue - minPossibleSensorValue);

        double result = newSensorValue - oldSensorValue;

        // Check for wrap around.
        if ( result < -possibleRange/2 )
            // Too far negative, must have wrapped around
            result += possibleRange;
        else if ( result > possibleRange/2 )
            // too far positive, must have wrapped around
            result -= possibleRange;

        return result;
    }

    /**
     * Move the value within the allowed range, logging/alerting if it is out of range.
     *
     * Normally, a function like this would raise a IllegalArgumentException, but that is drastic
     * when controlling a robot. Perhaps, such behavior could be enabled with a gamepad button someday,
     * but we'll have to just put up with alert messages and adjusted values for now.
     *
     * @param valueLabel
     * @param value
     * @param minPossibleValue
     * @param maxPossibleValue
     * @return
     */
    public static double clipValue(String valueLabel, double value, double minPossibleValue, double maxPossibleValue)
    {
        if (value < minPossibleValue)
        {
            Robot.sharedInstance.alert("Value of %s is too small: %f < %f", valueLabel, value, minPossibleValue);
            return minPossibleValue;
        }

        if (value > maxPossibleValue)
        {
            Robot.sharedInstance.alert("Value of %s is too big: %f > %f", valueLabel, value, maxPossibleValue);
            return maxPossibleValue;
        }

        return value;
    }

    /**
     * This can be used during robot setup but should not be used when the Opmode is running!
     * @param sleep_ms
     */
    public static void sleepUninterruptably(long sleep_ms)
    {
        try {
            Thread.sleep(sleep_ms);
        } catch (InterruptedException e) {
            Robot.sharedInstance.alert("Ignoring interrupted exception");
        }
    }
}
