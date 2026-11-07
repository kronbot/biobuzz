package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.mechanisms.DriveController;

/** One source of alliance settings for driving, recording, and replay. */
public enum Alliance {
    BLUE("robot_recording_blue.csv", Constants.LIMELIGHT_BLUE_1_PIPELINE,
            Constants.LIMELIGHT_BLUE_2_PIPELINE),
    RED("robot_recording_red.csv", Constants.LIMELIGHT_RED_1_PIPELINE,
            Constants.LIMELIGHT_RED_2_PIPELINE);

    public final String recordingFileName;
    public final int basket1Pipeline;
    public final int basket2Pipeline;

    Alliance(String recordingFileName, int basket1Pipeline, int basket2Pipeline) {
        this.recordingFileName = recordingFileName;
        this.basket1Pipeline = basket1Pipeline;
        this.basket2Pipeline = basket2Pipeline;
    }

    public void configureDrive(DriveController drive) {
        // Suppliers preserve live tuning of Constants from Panels.
        if (this == BLUE) {
            drive.configureAimTargets(
                    () -> Constants.BLUE_BASKET_1_X, () -> Constants.BLUE_BASKET_1_Y,
                    () -> Constants.BLUE_BASKET_2_X, () -> Constants.BLUE_BASKET_2_Y);
        } else {
            drive.configureAimTargets(
                    () -> Constants.RED_BASKET_1_X, () -> Constants.RED_BASKET_1_Y,
                    () -> Constants.RED_BASKET_2_X, () -> Constants.RED_BASKET_2_Y);
        }
    }
}
