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
        System.err.println(Robot.get().getRobotTag()+ " " + safeStringFormat(format, args));
    }
}