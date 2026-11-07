package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import android.os.Environment;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.PsiKitNextBridge;
import org.firstinspires.ftc.teamcode.utils.RecordingFormat;
import org.firstinspires.ftc.teamcode.utils.RecordingFormat.Frame;
import org.firstinspires.ftc.teamcode.utils.ReplayController;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PedroFollowerOdometryLogger;

import java.io.File;
import java.util.Collections;
import java.util.List;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;

/** Shared replay engine; concrete OpModes select an alliance and its recording. */
public abstract class ReplayOp extends NextOpMode {
    private final KronBot robot;
    private final Alliance alliance;
    private final File file;
    private final ElapsedTime timer = new ElapsedTime();
    private final ReplayController controller = new ReplayController();
    private List<Frame> frames = Collections.emptyList();
    private GoBildaPinpointDriver pinpoint;
    private VoltageSensor voltageSensor;
    private String error;
    private boolean initialized;
    private boolean running;
    private boolean completed;
    private int frameIndex;
    private int poseResetId;
    private double currentVoltage;
    private double lastVoltageTime = -1;
    private double peakError;
    private double totalError;
    private int ticks;

    protected ReplayOp(KronBot robot, Alliance alliance) {
        super(robot, stopRobotOnExit(robot), new PsiKitNextBridge.Hook());
        this.robot = robot;
        this.alliance = alliance;
        file = new File(Environment.getExternalStorageDirectory(), alliance.recordingFileName);
        try {
            // A bad/missing CSV is reported during INIT, before hardware is enabled.
            frames = RecordingFormat.load(file, alliance.name());
            robot.initialize(hardwareMap, poseOf(frames.get(0)));
            alliance.configureDrive(robot.drive);
            robot.vision.initialize(hardwareMap, frames.get(0).aimPipeline);
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            initialized = true;
        } catch (Exception failure) {
            error = failure.getMessage() == null
                    ? failure.getClass().getSimpleName() : failure.getMessage();
            robot.release();
        } catch (Error failure) {
            robot.release();
            throw failure;
        }
    }

    @Override
    public void disabledPeriodic() {
        if (initialized) robot.update();
        addStatusTelemetry();
        telemetry.addLine("Place the robot at the recorded physical starting position before PLAY");
        telemetry.update();
    }

    @Override
    public void start() {
        if (!initialized) return;
        Frame first = frames.get(0);
        robot.setPose(poseOf(first));
        robot.update();
        robot.vision.start();
        poseResetId = first.poseResetId;
        controller.reset();
        timer.reset();
        running = true;
    }

    @Override
    public void periodic() {
        if (!running) {
            addStatusTelemetry();
            telemetry.update();
            return;
        }
        double now = timer.seconds();
        if (now > duration()) {
            // Stop all mechanisms immediately; do not wait for the Driver Station STOP button.
            running = false;
            completed = true;
            robot.stop();
            PoseStorage.savePose(robot.follower.getPose());
            addStatusTelemetry();
            telemetry.update();
            return;
        }
        double timestamp = frames.get(0).timestamp + now;
        while (frameIndex + 1 < frames.size()
                && frames.get(frameIndex + 1).timestamp <= timestamp) {
            Frame next = frames.get(++frameIndex);
            if (next.poseResetId != poseResetId) {
                // Reproduce L3 as a coordinate reset, even when a slow loop skips frames.
                robot.setPose(poseOf(next));
                poseResetId = next.poseResetId;
                controller.reset();
                robot.drive.stopAiming();
            }
        }
        Frame a = frames.get(frameIndex);
        Frame b = frames.get(Math.min(frameIndex + 1, frames.size() - 1));
        double aimTurn = Double.NaN;
        if (a.aiming) {
            int pipeline = robot.drive.isUsingFirstAimTarget()
                    ? alliance.basket1Pipeline : alliance.basket2Pipeline;
            robot.vision.selectPipeline(pipeline);
            if (robot.drive.startAiming()) aimTurn = robot.drive.getAimTurn();
        } else {
            robot.drive.stopAiming();
            robot.vision.selectPipeline(a.aimPipeline);
        }
        if (now - lastVoltageTime >= 0.10) {
            currentVoltage = voltageSensor.getVoltage();
            lastVoltageTime = now;
        }
        Pose pose = robot.follower.getPose();
        Vector velocity = robot.follower.getVelocity();
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        ReplayController.Command command = controller.calculate(a, b, timestamp,
                pose.getX(), pose.getY(), pose.getHeading(),
                velocity.getXComponent(), velocity.getYComponent(), omega, currentVoltage, aimTurn);

        robot.intake.setPower(a.intakePower);
        robot.transfer.setPower(a.transferPower);
        robot.follower.setTeleOpDrive(command.forward, command.strafe, command.turn, !a.fieldCentric);
        robot.update();
        totalError += command.positionError;
        peakError = Math.max(peakError, command.positionError);
        ticks++;
        telemetry.addData("Replay", "%s | %.2f / %.2f s", alliance, now, duration());
        telemetry.addData("Frame", "%d / %d", frameIndex + 1, frames.size());
        telemetry.addData("Drive frame", a.fieldCentric ? "Field centric" : "Robot centric");
        telemetry.addData("Autoaim", a.aiming ? "ON" : "OFF");
        telemetry.addData("Pipeline", robot.vision.getSelectedPipeline());
        telemetry.addData("Intake / transfer", "%.2f / %.2f", a.intakePower, a.transferPower);
        telemetry.addData("Position error", "%.2f in", command.positionError);
        telemetry.addData("Mean / peak error", "%.2f / %.2f in", totalError / ticks, peakError);
        telemetry.addData("Heading error", "%.1f deg", Math.toDegrees(command.headingError));
        telemetry.addData("Command", "%.2f / %.2f / %.2f",
                command.forward, command.strafe, command.turn);
        telemetry.update();
        Pose updatedPose = robot.follower.getPose();
        PedroFollowerOdometryLogger.log("follower", updatedPose.getX(), updatedPose.getY(),
                updatedPose.getHeading());
        Logger.recordOutput("Replay/PositionError", command.positionError);
        Logger.recordOutput("Replay/HeadingError", command.headingError);
        Logger.recordOutput("Replay/IntakePower", a.intakePower);
        Logger.recordOutput("Replay/TransferPower", a.transferPower);
    }

    private void addStatusTelemetry() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Replay file", file.getAbsolutePath());
        if (error != null) {
            telemetry.addData("Replay error", error);
            return;
        }
        telemetry.addData("Replay", completed ? "COMPLETE - motors stopped" : "READY");
        telemetry.addData("Frames", frames.size());
        telemetry.addData("Duration", "%.2f s", duration());
        Frame first = frames.get(0);
        telemetry.addData("Recorded start", "(%.2f, %.2f, %.1f deg)",
                first.x, first.y, Math.toDegrees(first.heading));
    }

    private double duration() {
        return frames.get(frames.size() - 1).timestamp - frames.get(0).timestamp;
    }

    private static Pose poseOf(Frame frame) {
        return new Pose(frame.x, frame.y, frame.heading);
    }

    private static OpModeHook stopRobotOnExit(KronBot robot) {
        return new OpModeHook() {
            @Override
            public void afterEnd() {
                try {
                    robot.stop();
                } finally {
                    try {
                        if (robot.follower != null) PoseStorage.savePose(robot.follower.getPose());
                    } finally {
                        robot.release();
                    }
                }
            }
        };
    }
}
