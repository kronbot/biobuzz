package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.KronBot;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.RecordingFormat;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/** FTC_TestRobot replay loop, adapted to Biobuzz mechanisms and alliance aiming. */
public abstract class ReplayOp extends LinearOpMode {
    private static final int MODE_MANUAL = 0;
    private static final int MODE_FOLLOWING_PATH = 1;
    private static final int MODE_HOLDING_POSE = 2;
    private static final double TRANSLATION_KP = 0.022;
    private static final double TRANSLATION_KD = 0.013;
    private static final double ROTATION_KP = 0.30;
    private static final double ROTATION_KD = 0.08;
    private static final double VELOCITY_FILTER_ALPHA = 0.25;
    private static final double MAX_TRANSLATION_CORRECTION = 0.35;
    private static final double MAX_ROTATION_CORRECTION = 0.30;
    private static final double MIN_VOLTAGE_SCALE = 0.90;
    private static final double MAX_VOLTAGE_SCALE = 1.10;
    private static final double VOLTAGE_REFRESH_SEC = 0.10;
    private static final double MIN_DT = 0.008;
    private final KronBot robot = new KronBot();
    private final boolean blueAlliance;

    protected ReplayOp(boolean blueAlliance) {
        this.blueAlliance = blueAlliance;
    }

    private final List<RobotFrame> frames = new ArrayList<>();
    private final ElapsedTime replayTimer = new ElapsedTime();

    private File recordingFile;
    private String recordingWarning;
    private boolean robotInitialized;
    private double currentVoltage = 13.0;
    private double lastVoltageReadTime = -10;

    @Override
    public void runOpMode() {
        try {
            recordingFile = RecordingFormat.file(blueAlliance);
            frames.addAll(loadRecording(recordingFile, blueAlliance, Constants.TRANSFER_ENABLED,
                    warning -> {
                        recordingWarning = warning;
                        RobotLog.ww("Replay", "%s", warning);
                    }));
            // A preceding NextFTC/PsiKit TeleOp may have left the hubs in MANUAL cache mode.
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            RobotFrame first = frames.get(0);
            robot.initialize(hardwareMap, new Pose(first.x, first.y, first.heading));
            robotInitialized = true;
            robot.drive.configureAlliance(blueAlliance);
            robot.vision.initialize(hardwareMap, first.aimPipeline);

            telemetry.addData("Replay file", recordingFile.getAbsolutePath());
            telemetry.addData("Frames", frames.size());
            telemetry.addData("Duration", "%.2f s", duration());
            if (recordingWarning != null) telemetry.addData("Recording warning", recordingWarning);
            telemetry.addLine("Ready to replay");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            // Force the same coordinate origin used at the beginning of recording.
            robot.setPose(new Pose(first.x, first.y, first.heading));
            robot.update();
            robot.follower.startTeleopDrive(true);
            robot.vision.start();

            executeReplay();
        } catch (Exception e) {
            // An INIT failure must stay visible. Returning here before waitForStart() makes
            // the SDK replace the useful error with its "ended prematurely" warning.
            try {
                robot.stop();
            } catch (RuntimeException stoppingFailure) {
                e.addSuppressed(stoppingFailure);
            }
            RobotLog.ee("Replay", e, "Replay failed for %s", recordingFile);
            String error = e.getMessage() == null ? e.getClass().getSimpleName() : e.getMessage();
            while (!isStopRequested()) {
                telemetry.addData("Replay", "BLOCKED - motors stopped");
                telemetry.addData("Replay file", recordingFile);
                telemetry.addData("Replay error", error);
                // Also monitor PLAY while blocked; it must never start an invalid replay.
                telemetry.addLine(isStarted()
                        ? "Replay cannot run. Press STOP and fix the error above."
                        : "INIT failed. Press STOP and fix the error above.");
                telemetry.update();
                sleep(100);
            }
        } finally {
            stopRobotSafely();
        }
    }

    private void executeReplay() {
        RobotFrame first = frames.get(0);
        double firstTimestamp = first.timestamp;
        double replayDuration = duration();
        ReplayTimeline timeline = new ReplayTimeline(frames);
        int waitPipeline = -1;

        double previousLoopTime = 0;
        double previousHeading = first.heading;
        double filteredVelocityErrorX = 0;
        double filteredVelocityErrorY = 0;
        double filteredOmegaError = 0;
        double peakPositionError = 0;
        double meanPositionError = 0;
        double ticks = 0;

        replayTimer.reset();

        while (opModeIsActive()) {
            double wallTime = replayTimer.seconds();
            while (timeline.advance(wallTime)) {
                RobotFrame next = frames.get(timeline.frameIndex());
                if (next.poseReset) {
                    robot.setPose(new Pose(next.x, next.y, next.heading));
                    previousLoopTime = 0;
                    filteredVelocityErrorX = 0;
                    filteredVelocityErrorY = 0;
                    filteredOmegaError = 0;
                }
                if (timeline.isWaiting()) {
                    robot.drive.stopAiming();
                    robot.drive.stop();
                    // breakFollowing() sets FLOAT; enable BRAKE without restarting the follower.
                    robot.follower.getDrivetrain().startTeleopDrive(true);
                    robot.intake.turnOff();
                    robot.transfer.turnOff();
                    waitPipeline = waitPipeline(next);
                    robot.vision.selectPipeline(waitPipeline);
                }
            }

            if (timeline.isWaiting()) {
                // A full follower update can apply centripetal correction even with zero input.
                // Only update localization while the drivetrain is stopped.
                robot.follower.updatePose();
                boolean tagDetected = robot.vision.hasFreshAprilTag(waitPipeline);
                wallTime = replayTimer.seconds();
                boolean resumed = timeline.finishWait(wallTime, tagDetected);
                telemetry.addData("Replay", resumed ? "RESUMING" : "WAITING FOR APRILTAG");
                telemetry.addData("Time", "%.2f / %.2f s", timeline.elapsed(wallTime), replayDuration);
                telemetry.addData("AprilTag wait point", timeline.waitPointCount());
                telemetry.addData("AprilTag wait pipeline", waitPipeline);
                telemetry.addData("AprilTag wait time", "%.2f / %.2f s",
                        timeline.waitElapsed(wallTime), ReplayTimeline.TAG_WAIT_TIMEOUT_SEC);
                telemetry.addData("AprilTag wait result", timeline.lastWaitResult());
                telemetry.update();
                if (!opModeIsActive()) return;
                if (!resumed) {
                    idle();
                    continue;
                }
                previousLoopTime = 0;
                previousHeading = robot.follower.getPose().getHeading();
                filteredVelocityErrorX = 0;
                filteredVelocityErrorY = 0;
                filteredOmegaError = 0;
                lastVoltageReadTime = -10;
                robot.follower.startTeleopDrive(true);
            }

            double now = timeline.elapsed(wallTime);
            double targetTimestamp = firstTimestamp + now;
            int frameIndex = timeline.frameIndex();
            RobotFrame a = frames.get(frameIndex);
            RobotFrame b = frameIndex + 1 < frames.size()
                    ? frames.get(frameIndex + 1) : a;
            double interpolation = b.poseReset || b.waitForAprilTag
                    ? 0 : interpolationAmount(a, b, targetTimestamp);

            double targetX = lerp(a.x, b.x, interpolation);
            double targetY = lerp(a.y, b.y, interpolation);
            double targetHeading = lerpAngle(a.heading, b.heading, interpolation);
            double targetVelocityX = lerp(a.velocityX, b.velocityX, interpolation);
            double targetVelocityY = lerp(a.velocityY, b.velocityY, interpolation);
            double targetOmega = lerp(a.angularVelocity, b.angularVelocity, interpolation);

            refreshVoltage(now);
            double recordedVoltage = lerp(a.voltage, b.voltage, interpolation);
            double voltageScale = currentVoltage > 1 && recordedVoltage > 1
                    ? Range.clip(recordedVoltage / currentVoltage,
                    MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE)
                    : 1.0;

            DriveCommand feedforward = interpolateFeedforward(a, b, interpolation);
            // Field-centric input and target lock only affect manual driving.
            // Path/pose feedforward is already in the robot coordinate frame.
            boolean manualDrive = a.driveMode == MODE_MANUAL;
            boolean fieldCentric = manualDrive && a.fieldCentric;
            boolean aimingAtTarget = manualDrive && a.aimingAtTarget;
            feedforward.scale(voltageScale);

            Pose currentPose = robot.follower.getPose();
            Vector currentVelocity = robot.follower.getVelocity();
            double currentHeading = currentPose.getHeading();
            double dt = previousLoopTime > 0
                    ? Math.max(now - previousLoopTime, MIN_DT) : MIN_DT;

            double errorFieldX = targetX - currentPose.getX();
            double errorFieldY = targetY - currentPose.getY();
            double headingError = normalizeAngle(targetHeading - currentHeading);
            double velocityErrorX = targetVelocityX - currentVelocity.getXComponent();
            double velocityErrorY = targetVelocityY - currentVelocity.getYComponent();
            double actualOmega = previousLoopTime > 0
                    ? normalizeAngle(currentHeading - previousHeading) / dt
                    : targetOmega;
            double omegaError = targetOmega - actualOmega;

            filteredVelocityErrorX += VELOCITY_FILTER_ALPHA
                    * (velocityErrorX - filteredVelocityErrorX);
            filteredVelocityErrorY += VELOCITY_FILTER_ALPHA
                    * (velocityErrorY - filteredVelocityErrorY);
            filteredOmegaError += VELOCITY_FILTER_ALPHA
                    * (omegaError - filteredOmegaError);

            double correctionFieldX = errorFieldX * TRANSLATION_KP
                    + filteredVelocityErrorX * TRANSLATION_KD;
            double correctionFieldY = errorFieldY * TRANSLATION_KP
                    + filteredVelocityErrorY * TRANSLATION_KD;

            double cosHeading = Math.cos(currentHeading);
            double sinHeading = Math.sin(currentHeading);
            double correctionForward = cosHeading * correctionFieldX
                    + sinHeading * correctionFieldY;
            double correctionStrafe = -sinHeading * correctionFieldX
                    + cosHeading * correctionFieldY;

            double correctionMagnitude = Math.hypot(
                    correctionForward, correctionStrafe);
            if (correctionMagnitude > MAX_TRANSLATION_CORRECTION) {
                double correctionScale = MAX_TRANSLATION_CORRECTION
                        / correctionMagnitude;
                correctionForward *= correctionScale;
                correctionStrafe *= correctionScale;
                correctionFieldX *= correctionScale;
                correctionFieldY *= correctionScale;
            }

            double correctionTurn = Range.clip(
                    headingError * ROTATION_KP
                            + filteredOmegaError * ROTATION_KD,
                    -MAX_ROTATION_CORRECTION, MAX_ROTATION_CORRECTION);

            double commandForward = feedforward.forward
                    + (fieldCentric ? correctionFieldX : correctionForward);
            double commandStrafe = feedforward.strafe
                    + (fieldCentric ? correctionFieldY : correctionStrafe);
            if (aimingAtTarget) {
                int pipeline = robot.drive.isUsingFirstAimTarget()
                        ? (blueAlliance ? Constants.LIMELIGHT_BLUE_1_PIPELINE : Constants.LIMELIGHT_RED_1_PIPELINE)
                        : (blueAlliance ? Constants.LIMELIGHT_BLUE_2_PIPELINE : Constants.LIMELIGHT_RED_2_PIPELINE);
                robot.vision.selectPipeline(pipeline);
                robot.drive.startAiming();
            } else {
                robot.drive.stopAiming();
                robot.vision.selectPipeline(a.aimPipeline);
            }
            double commandTurn = Range.clip(
                    (aimingAtTarget ? robot.drive.getAimTurn() : feedforward.turn) + correctionTurn,
                    -1, 1);
            double translationMagnitude = Math.hypot(
                    commandForward, commandStrafe);
            if (translationMagnitude > 1) {
                commandForward /= translationMagnitude;
                commandStrafe /= translationMagnitude;
            }

            robot.intake.setPower(a.intakePower);
            robot.transfer.setPower(a.transferPower);
            robot.follower.setTeleOpDrive(
                    commandForward, commandStrafe, commandTurn, !fieldCentric);
            robot.update();
            ticks++;

            double positionError = Math.hypot(errorFieldX, errorFieldY);
            meanPositionError += positionError;
            peakPositionError = Math.max(peakPositionError, positionError);
            telemetry.addData("Time", "%.2f / %.2f s", now, replayDuration);
            if (recordingWarning != null) telemetry.addData("Recording warning", recordingWarning);
            telemetry.addData("Frame", "%d / %d", frameIndex + 1, frames.size());
            telemetry.addData("AprilTag wait points passed", timeline.waitPointCount());
            telemetry.addData("AprilTag wait result", timeline.lastWaitResult());
            telemetry.addData("Recorded mode", modeName(a.driveMode));
            telemetry.addData("Drive frame", fieldCentric ? "Field centric" : "Robot centric");
            telemetry.addData("Target lock", targetLockName(a));
            telemetry.addData("Pose reset", a.poseReset);
            telemetry.addData("Intake / transfer", "%.2f / %.2f", a.intakePower, a.transferPower);
            telemetry.addData("Position error", "%.2f in", positionError);
            telemetry.addData("Mean error", "%.2f in", meanPositionError / ticks);
            telemetry.addData("Peak error", "%.2f in", peakPositionError);
            telemetry.addData("Heading error", "%.1f deg",
                    Math.toDegrees(Math.abs(headingError)));
            telemetry.addData("Feedforward", "%.2f  %.2f  %.2f",
                    feedforward.forward, feedforward.strafe, feedforward.turn);
            telemetry.addData("Command", "%.2f  %.2f  %.2f",
                    commandForward, commandStrafe, commandTurn);
            telemetry.update();

            previousHeading = currentHeading;
            previousLoopTime = now;
            // Apply the final frame once, including a final wait marker after a slow loop.
            if (frameIndex == frames.size() - 1 && now >= replayDuration) break;
            idle();
        }
    }

    static int waitPipeline(RobotFrame frame) {
        boolean firstBasket = frame.y < 72.0;
        return frame.aimingAtBlue
                ? (firstBasket ? Constants.LIMELIGHT_BLUE_1_PIPELINE : Constants.LIMELIGHT_BLUE_2_PIPELINE)
                : (firstBasket ? Constants.LIMELIGHT_RED_1_PIPELINE : Constants.LIMELIGHT_RED_2_PIPELINE);
    }

    static List<RobotFrame> loadRecording(File file, boolean expectedBlue, boolean transferEnabled)
            throws IOException {
        return loadRecording(file, expectedBlue, transferEnabled, warning -> {});
    }

    static List<RobotFrame> loadRecording(File file, boolean expectedBlue, boolean transferEnabled,
                                         Consumer<String> reportWarning) throws IOException {
        List<RobotFrame> frames = new ArrayList<>();
        IOException invalidFinalRow = null;
        if (!file.isFile()) {
            throw new IOException("Recording not found: " + file.getAbsolutePath());
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String header = reader.readLine();
            if (header == null) {
                throw new IOException("Recording is empty");
            }

            int expectedColumns;
            if (RecordingFormat.HEADER.equals(header)) {
                expectedColumns = RecordingFormat.COLUMN_COUNT;
            } else if (RecordingFormat.LEGACY_HEADER.equals(header)) {
                expectedColumns = RecordingFormat.LEGACY_COLUMN_COUNT;
            } else {
                throw new IOException("Unsupported CSV format; record again with Biobuzz Recorder");
            }

            String line;
            int lineNumber = 1;
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                if (line.trim().isEmpty()) {
                    continue;
                }
                // A malformed row is recoverable only if no further non-empty row follows it.
                if (invalidFinalRow != null) {
                    throw invalidFinalRow;
                }
                RobotFrame frame;
                try {
                    String[] values = line.split(",", -1);
                    RecordingFormat.validateRow(values, expectedColumns);
                    frame = new RobotFrame(values);
                } catch (NumberFormatException e) {
                    invalidFinalRow = new IOException("Invalid CSV line " + lineNumber
                            + ": " + e.getMessage(), e);
                    continue;
                }
                // These errors must never be hidden, even on the final row.
                validateFrame(frame, lineNumber, frames);
                if (frame.aimingAtBlue != expectedBlue) {
                    throw new IOException("Recording belongs to the other alliance, line " + lineNumber);
                }
                if (!transferEnabled && frame.transferPower != 0) {
                    throw new IOException("Recording requires the transfer motor; record again with transfer disabled");
                }
                frames.add(frame);
            }
        }

        if (frames.size() < 2) {
            if (invalidFinalRow != null) throw invalidFinalRow;
            throw new IOException("Recording needs at least two frames; press PLAY in Recorder first");
        }
        if (invalidFinalRow != null) {
            reportWarning.accept("Ignored final row. " + invalidFinalRow.getMessage());
        }
        return frames;
    }

    private static void validateFrame(RobotFrame frame, int lineNumber, List<RobotFrame> frames)
            throws IOException {
        if (!Double.isFinite(frame.timestamp)
                || !Double.isFinite(frame.x)
                || !Double.isFinite(frame.y)
                || !Double.isFinite(frame.heading)) {
            throw new IOException("Non-finite value on CSV line " + lineNumber);
        }
        if (!frames.isEmpty()
                && frame.timestamp < frames.get(frames.size() - 1).timestamp) {
            throw new IOException("Timestamps go backwards on CSV line " + lineNumber);
        }
        if (frame.driveMode < MODE_MANUAL || frame.driveMode > MODE_HOLDING_POSE) {
            throw new IOException("Invalid DriveMode on CSV line " + lineNumber);
        }
    }

    private DriveCommand interpolateFeedforward(
            RobotFrame a, RobotFrame b, double amount) {
        DriveCommand commandA = a.feedforward();
        if (a.driveMode != b.driveMode
                || a.fieldCentric != b.fieldCentric
                || a.aimingAtTarget != b.aimingAtTarget
                || a.aimingAtBlue != b.aimingAtBlue || b.poseReset || b.waitForAprilTag) {
            return commandA;
        }

        DriveCommand commandB = b.feedforward();
        return new DriveCommand(
                lerp(commandA.forward, commandB.forward, amount),
                lerp(commandA.strafe, commandB.strafe, amount),
                lerp(commandA.turn, commandB.turn, amount));
    }

    private double interpolationAmount(
            RobotFrame a, RobotFrame b, double targetTimestamp) {
        double frameDuration = b.timestamp - a.timestamp;
        if (frameDuration <= 0) {
            return 0;
        }
        return Range.clip(
                (targetTimestamp - a.timestamp) / frameDuration, 0, 1);
    }

    private void refreshVoltage(double now) {
        if (now - lastVoltageReadTime >= VOLTAGE_REFRESH_SEC) {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageReadTime = now;
        }
    }

    private void stopRobotSafely() {
        try {
            robot.stop();
        } finally {
            try {
                if (robotInitialized && robot.follower != null) {
                    PoseStorage.savePose(robot.follower.getPose());
                }
            } finally {
                robot.release();
            }
        }
    }

    private double duration() {
        return frames.get(frames.size() - 1).timestamp
                - frames.get(0).timestamp;
    }

    private static String modeName(int driveMode) {
        if (driveMode == MODE_MANUAL) return "Manual";
        if (driveMode == MODE_FOLLOWING_PATH) return "Following path";
        return "Holding pose";
    }

    private static String targetLockName(RobotFrame frame) {
        if (!frame.aimingAtTarget) return "Off";
        return frame.aimingAtBlue ? "Blue" : "Red";
    }

    private static double lerp(double a, double b, double amount) {
        return a + (b - a) * amount;
    }

    private static double lerpAngle(double a, double b, double amount) {
        return normalizeAngle(a + normalizeAngle(b - a) * amount);
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static final class DriveCommand {
        double forward;
        double strafe;
        double turn;

        DriveCommand(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }

        void scale(double amount) {
            forward = Range.clip(forward * amount, -1, 1);
            strafe = Range.clip(strafe * amount, -1, 1);
            turn = Range.clip(turn * amount, -1, 1);
        }
    }

    static final class RobotFrame {
        final double timestamp;
        final double leftRearPower;
        final double rightRearPower;
        final double leftFrontPower;
        final double rightFrontPower;
        final double x;
        final double y;
        final double heading;
        final double voltage;
        final double driveForward;
        final double driveStrafe;
        final double driveTurn;
        final int driveMode;
        final boolean aPressed;
        final boolean bPressed;
        final double velocityX;
        final double velocityY;
        final double angularVelocity;
        final boolean fieldCentric;
        final boolean aimingAtTarget;
        final boolean aimingAtBlue;
        final double intakePower;
        final double transferPower;
        final int aimPipeline;
        final boolean poseReset;
        final boolean waitForAprilTag;

        RobotFrame(String[] values) {
            RecordingFormat.validateRow(values, values.length);
            timestamp = Double.parseDouble(values[0]);
            leftRearPower = Double.parseDouble(values[1]);
            rightRearPower = Double.parseDouble(values[2]);
            leftFrontPower = Double.parseDouble(values[3]);
            rightFrontPower = Double.parseDouble(values[4]);
            x = Double.parseDouble(values[5]);
            y = Double.parseDouble(values[6]);
            heading = Double.parseDouble(values[7]);
            voltage = Double.parseDouble(values[8]);
            driveForward = Double.parseDouble(values[9]);
            driveStrafe = Double.parseDouble(values[10]);
            driveTurn = Double.parseDouble(values[11]);
            driveMode = Integer.parseInt(values[12]);
            aPressed = parseBoolean(values[13]);
            bPressed = parseBoolean(values[14]);
            velocityX = Double.parseDouble(values[15]);
            velocityY = Double.parseDouble(values[16]);
            angularVelocity = Double.parseDouble(values[17]);
            fieldCentric = parseBoolean(values[18]);
            aimingAtTarget = parseBoolean(values[19]);
            aimingAtBlue = parseBoolean(values[20]);
            intakePower = Double.parseDouble(values[21]);
            transferPower = Double.parseDouble(values[22]);
            aimPipeline = Integer.parseInt(values[23]);
            poseReset = parseBoolean(values[24]);
            waitForAprilTag = values.length == RecordingFormat.COLUMN_COUNT && "1".equals(values[25]);
        }

        DriveCommand feedforward() {
            if (driveMode == MODE_MANUAL) {
                return new DriveCommand(driveForward, driveStrafe, driveTurn);
            }

            // Path-following and holding rows have zero joystick commands.
            // Recover their drive feedforward from the four recorded powers.
            return new DriveCommand(
                    (leftFrontPower + rightFrontPower
                            + leftRearPower + rightRearPower) / 4.0,
                    (leftFrontPower - rightFrontPower
                            - leftRearPower + rightRearPower) / 4.0,
                    (leftFrontPower - rightFrontPower
                            + leftRearPower - rightRearPower) / 4.0);
        }

        private static boolean parseBoolean(String value) {
            return "1".equals(value) || "true".equalsIgnoreCase(value);
        }
    }
}
