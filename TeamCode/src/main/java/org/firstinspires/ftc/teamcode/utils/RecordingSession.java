package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.RecordingFormat.Frame;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/** Adds recording to an alliance TeleOp without changing its driver controls. */
public final class RecordingSession {
    private static final double SAMPLE_INTERVAL = 0.02;
    private static final double FLUSH_INTERVAL = 0.5;
    private final KronBot robot;
    private final Alliance alliance;
    private final ElapsedTime timer = new ElapsedTime();
    private final File file;
    private final File temporary;
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor voltageSensor;
    private BufferedWriter writer;
    private Frame lastFrame;
    private double lastFlushTime;
    private int poseResetId;
    private int frameCount;
    private String error;
    private boolean started;
    private boolean hadActiveLoop;

    public RecordingSession(KronBot robot, Alliance alliance) {
        this.robot = robot;
        this.alliance = alliance;
        file = new File(Environment.getExternalStorageDirectory(), alliance.recordingFileName);
        temporary = new File(file.getAbsolutePath() + ".tmp");
    }

    public void initialize(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void start() {
        // INIT must not overwrite the last successful run.
        try {
            writer = new BufferedWriter(new FileWriter(temporary), 32 * 1024);
            writer.write(RecordingFormat.HEADER);
            writer.newLine();
            started = true;
            timer.reset();
            writeFrame(capture(0));
        } catch (IOException | RuntimeException e) {
            fail(e);
        }
    }

    public void markPoseReset() {
        poseResetId++;
    }

    public void update() {
        if (writer == null) return;
        hadActiveLoop = true;
        Frame frame = capture(timer.seconds());
        boolean stateChanged = lastFrame == null || !frame.sameDiscreteState(lastFrame);
        if (!stateChanged && frame.timestamp - lastFrame.timestamp < SAMPLE_INTERVAL) return;
        try {
            writeFrame(frame);
            if (frame.timestamp - lastFlushTime >= FLUSH_INTERVAL) {
                writer.flush();
                lastFlushTime = frame.timestamp;
            }
        } catch (IOException e) {
            fail(e);
        }
    }

    /** Called from an afterEnd hook, before the robot's hardware is released. */
    public void close() {
        if (writer == null) return;
        try {
            if (hadActiveLoop) writeFrame(capture(timer.seconds()));
            writer.close();
            writer = null;
            // NextFTC can invoke start/end after STOP during INIT; preserve the old CSV.
            if (!hadActiveLoop) return;
            if (frameCount < 2 || !temporary.renameTo(file)) {
                throw new IOException("Could not save recording to " + file.getAbsolutePath());
            }
        } catch (IOException | RuntimeException e) {
            fail(e);
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Recording", error != null ? "FAILED"
                : (writer != null ? "ACTIVE" : (started ? "SAVED" : "Starts at PLAY")));
        telemetry.addData("Recording file", file.getAbsolutePath());
        telemetry.addData("Recorded frames", frameCount);
        if (started) telemetry.addData("Recorded time", "%.2f s", timer.seconds());
        if (error != null) telemetry.addData("Recording error", error);
    }

    private Frame capture(double timestamp) {
        Pose pose = robot.follower.getPose();
        Vector velocity = robot.follower.getVelocity();
        Frame frame = new Frame();
        frame.timestamp = timestamp;
        frame.x = pose.getX();
        frame.y = pose.getY();
        frame.heading = pose.getHeading();
        frame.voltage = voltageSensor.getVoltage();
        // These are the commands actually sent, including deadzone, speed, and autoaim.
        frame.driveForward = robot.drive.getCommandForward();
        frame.driveStrafe = robot.drive.getCommandStrafe();
        frame.driveTurn = robot.drive.getCommandTurn();
        frame.velocityX = velocity.getXComponent();
        frame.velocityY = velocity.getYComponent();
        frame.angularVelocity = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        frame.fieldCentric = robot.drive.isFieldCentric();
        frame.aiming = robot.drive.isAiming();
        frame.aimPipeline = robot.vision.getSelectedPipeline();
        frame.intakePower = robot.intake.getPower();
        frame.transferPower = robot.transfer.getPower();
        frame.poseResetId = poseResetId;
        frame.alliance = alliance.name();
        return frame;
    }

    private void writeFrame(Frame frame) throws IOException {
        // CSV timestamps have microsecond precision and must remain strictly increasing.
        if (lastFrame != null && frame.timestamp - lastFrame.timestamp < 0.000001) return;
        String csv = frame.toCsv();
        try {
            // Never replace a good recording with a run containing invalid sensor data.
            Frame.parse(csv);
        } catch (IllegalArgumentException e) {
            throw new IOException("Invalid recording frame: " + e.getMessage(), e);
        }
        writer.write(csv);
        writer.newLine();
        lastFrame = frame;
        frameCount++;
    }

    private void fail(Exception failure) {
        error = failure.getMessage();
        RobotLog.ee("RecordingSession", failure, "Failed to save recording");
        if (writer != null) {
            try {
                writer.close();
            } catch (IOException ignored) {
                // Keep the original error and leave the previous complete CSV intact.
            }
            writer = null;
        }
    }
}
