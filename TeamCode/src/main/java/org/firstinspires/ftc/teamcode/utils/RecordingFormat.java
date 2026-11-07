package org.firstinspires.ftc.teamcode.utils;

import java.io.File;

/** Shared CSV contract between RecorderOp and ReplayOp, as in FTC_TestRobot. */
public final class RecordingFormat {
    public static final String BLUE_FILE_NAME = "robot_recording_blue.csv";
    public static final String RED_FILE_NAME = "robot_recording_red.csv";
    public static final String HEADER =
            "Time,LR,RR,LF,RF,X,Y,Heading,Voltage,"
                    + "DriveFwd,DriveStr,DriveTurn,DriveMode,"
                    + "APressed,BPressed,VelX,VelY,AngularVelocity,"
                    + "FieldCentric,AimingAtTarget,AimingAtBlue,"
                    + "IntakePower,TransferPower,AimPipeline,PoseReset";
    public static final int COLUMN_COUNT = 25;
    public static final int MODE_MANUAL = 0;
    public static final int MODE_FOLLOWING_PATH = 1;
    public static final int MODE_HOLDING_POSE = 2;

    public static File file(boolean blueAlliance) {
        return new File(Constants.RECORDING_DIRECTORY,
                blueAlliance ? BLUE_FILE_NAME : RED_FILE_NAME);
    }

    /** The same row checks are used before writing and while loading. */
    public static void validateRow(String[] values) {
        if (values.length != COLUMN_COUNT) {
            throw new NumberFormatException("Expected " + COLUMN_COUNT + " columns, got " + values.length);
        }
        for (int i = 0; i < values.length; i++) {
            if (!Double.isFinite(Double.parseDouble(values[i]))) {
                throw new NumberFormatException("Non-finite value in column " + (i + 1));
            }
        }
        int pipeline = Integer.parseInt(values[23]);
        if (Double.parseDouble(values[0]) < 0 || Double.parseDouble(values[8]) <= 1
                || Math.abs(Double.parseDouble(values[21])) > 1
                || Math.abs(Double.parseDouble(values[22])) > 1
                || pipeline < 0 || pipeline > 9) {
            throw new NumberFormatException("Invalid time, voltage, mechanism power, or pipeline");
        }
    }

    private RecordingFormat() {}
}
