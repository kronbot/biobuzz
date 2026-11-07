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

    private RecordingFormat() {}
}
