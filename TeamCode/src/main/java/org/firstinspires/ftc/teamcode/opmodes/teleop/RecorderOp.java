package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.RecordingFormat;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/** Same lifecycle and direct CSV writer as FTC_TestRobot's RecorderOp. */
public abstract class RecorderOp extends AllianceDrivingTeleOp {
    private static final double RECORD_INTERVAL_SEC = 0.02;
    private final ElapsedTime recordTimer = new ElapsedTime();
    private BufferedWriter dataRecorder;
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private GoBildaPinpointDriver pinpoint;
    private String recordingFilePath;
    private String recordingError;
    private double lastRecordTime;
    private double lastFlushTime;
    private double lastIntakePower;
    private double lastTransferPower;
    private boolean lastAiming;
    private boolean started;
    private int frameCount;
    private int waitPointCount;

    protected RecorderOp(KronBot robot, boolean blueAlliance) {
        super(robot, blueAlliance);
    }

    @Override protected void onRobotInit() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        File file = RecordingFormat.file(blueAlliance);
        recordingFilePath = file.getAbsolutePath();
        try {
            File directory = file.getParentFile();
            if (directory != null && !directory.isDirectory()
                    && !directory.mkdirs() && !directory.isDirectory()) {
                throw new IOException("Cannot create directory: " + directory);
            }
            // Write the final CSV directly. No temporary file or rename at STOP.
            dataRecorder = new BufferedWriter(new FileWriter(file), 32 * 1024);
            dataRecorder.write(RecordingFormat.HEADER + "\n");
            dataRecorder.flush();
        } catch (IOException e) {
            handleRecordingError(e);
            closeRecorder();
        }
    }

    @Override protected void onRobotStart() {
        started = true;
        recordTimer.reset();
        lastRecordTime = 0;
        recordFrame(0, true, false);
    }

    @Override protected void onRobotUpdated() {
        boolean importantEvent = controls.triangle.justPressed()
                || controls.leftStickButton.justPressed()
                || controls.rightStickButton.justPressed()
                || robot.drive.isAiming() != lastAiming
                || robot.intake.getPower() != lastIntakePower
                || robot.transfer.getPower() != lastTransferPower;
        recordFrame(recordTimer.seconds(), importantEvent, controls.rightStickButton.justPressed());
    }

    @Override protected void onRobotLoop() {
        telemetry.addData("Recording", recordingError != null ? "FAILED"
                : (started ? (dataRecorder == null ? "SAVED" : "ACTIVE") : "READY"));
        telemetry.addData("Recording file", recordingFilePath);
        telemetry.addData("Recorded frames", frameCount);
        telemetry.addData("AprilTag wait points", waitPointCount);
        telemetry.addLine("R3: mark AprilTag wait for replay (driving continues while recording)");
        if (recordingError != null) telemetry.addData("Recording error", recordingError);
    }

    @Override protected void onRobotStopping() {
        // Only flush/close at STOP: do not read hardware after the SDK has stopped it.
        closeRecorder();
        onRobotLoop();
        telemetry.update();
    }

    private void recordFrame(double sampleTime, boolean force, boolean waitForAprilTag) {
        if (dataRecorder == null) return;
        if (!force && sampleTime - lastRecordTime < RECORD_INTERVAL_SEC) return;
        boolean manual = robot.drive.isManual();
        int mode = manual ? RecordingFormat.MODE_MANUAL
                : (robot.follower.isBusy() ? RecordingFormat.MODE_FOLLOWING_PATH
                : RecordingFormat.MODE_HOLDING_POSE);
        Pose pose = robot.follower.getPose();
        Vector velocity = robot.follower.getVelocity();
        double intakePower = robot.intake.getPower();
        double transferPower = robot.transfer.getPower();
        try {
            String row = String.format(Locale.US,
                    "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,"
                            + "%.4f,%.4f,%.4f,%d,%d,%d,%.4f,%.4f,%.4f,%d,%d,%d,"
                            + "%.4f,%.4f,%d,%d,%d\n",
                    sampleTime, leftRear.getPower(), rightRear.getPower(),
                    leftFront.getPower(), rightFront.getPower(),
                    pose.getX(), pose.getY(), pose.getHeading(),
                    hardwareMap.voltageSensor.iterator().next().getVoltage(),
                    manual ? Constants.applyDeadzone(-controls.leftStickY) : 0,
                    manual ? Constants.applyDeadzone(-controls.leftStickX) : 0,
                    manual ? Constants.applyDeadzone(-controls.rightStickX) : 0,
                    mode, controls.cross.justPressed() ? 1 : 0, controls.circle.justPressed() ? 1 : 0,
                    velocity.getXComponent(), velocity.getYComponent(),
                    pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS),
                    robot.drive.isFieldCentric() ? 1 : 0, robot.drive.isAiming() ? 1 : 0,
                    blueAlliance ? 1 : 0, intakePower, transferPower,
                    robot.vision.getSelectedPipeline(), controls.leftStickButton.justPressed() ? 1 : 0,
                    waitForAprilTag ? 1 : 0);
            // Reject invalid sensor values before they can become the final row in the CSV.
            try {
                RecordingFormat.validateRow(row.trim().split(",", -1));
            } catch (NumberFormatException invalidFrame) {
                throw new IOException("Invalid recording frame: " + invalidFrame.getMessage(), invalidFrame);
            }
            dataRecorder.write(row);
            lastRecordTime = sampleTime;
            lastIntakePower = intakePower;
            lastTransferPower = transferPower;
            lastAiming = robot.drive.isAiming();
            frameCount++;
            if (waitForAprilTag) waitPointCount++;
            if (force || sampleTime - lastFlushTime >= 0.5) {
                dataRecorder.flush();
                lastFlushTime = sampleTime;
            }
        } catch (IOException e) {
            handleRecordingError(e);
            closeRecorder();
        }
    }

    private void handleRecordingError(IOException e) {
        recordingError = e.getMessage() == null ? e.getClass().getSimpleName() : e.getMessage();
        RobotLog.ee("Recorder", e, "CSV recording failed");
    }

    private void closeRecorder() {
        if (dataRecorder == null) return;
        try {
            dataRecorder.close(); // BufferedWriter.close() also flushes.
            if (recordingError == null) {
                RobotLog.ii("Recorder", "Saved %d frames: %s", frameCount, recordingFilePath);
            }
        } catch (IOException e) {
            handleRecordingError(e);
        } finally {
            dataRecorder = null;
        }
    }
}
