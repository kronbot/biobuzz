package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.KronBot;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightController;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Controls;
import org.firstinspires.ftc.teamcode.utils.PsiKitNextBridge;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.psilynx.psikit.ftc.PedroFollowerOdometryLogger;

import org.psilynx.psikit.core.Logger;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.OpModeHook;

public abstract class AllianceDrivingTeleOp extends NextOpMode {
    static {
        // PsiKit only accepts metadata before logging starts, so record it up front.
        PsiKitNextBridge.recordMetadata("Robot", "MyRobot");
        PsiKitNextBridge.recordMetadata("Mode", "TeleOp");
    }

    private final String allianceName;
    protected final KronBot robot;
    protected final Controls controls;
    protected final boolean blueAlliance;
    private Pose selectedStartPose;
    private final int basket1Pipeline;
    private final int basket2Pipeline;
    private boolean initAttempted;
    private boolean initialized;
    private boolean stopped;
    private String initializationError;

    protected AllianceDrivingTeleOp(KronBot robot, boolean blueAlliance) {
        this(robot, blueAlliance, new ExitHook());
    }

    private AllianceDrivingTeleOp(KronBot robot, boolean blueAlliance, ExitHook exitHook) {
        super(robot, exitHook, new PsiKitNextBridge.Hook());
        exitHook.opMode = this;
        this.robot = robot;
        this.blueAlliance = blueAlliance;
        allianceName = blueAlliance ? "BLUE" : "RED";
        basket1Pipeline = blueAlliance ? Constants.LIMELIGHT_BLUE_1_PIPELINE
                : Constants.LIMELIGHT_RED_1_PIPELINE;
        basket2Pipeline = blueAlliance ? Constants.LIMELIGHT_BLUE_2_PIPELINE
                : Constants.LIMELIGHT_RED_2_PIPELINE;
        controls = new Controls(gamepad1);
        selectedStartPose = leftStartPose();
    }

    private void initializeRobot() {
        if (initAttempted) return;
        initAttempted = true;
        try {
            robot.initialize(hardwareMap, selectedStartPose);
            robot.drive.configureAlliance(blueAlliance);
            robot.vision.initialize(hardwareMap, basket1Pipeline);
            onRobotInit();
            initialized = true;
        } catch (RuntimeException failure) {
            initializationError = failure.getMessage() == null
                    ? failure.getClass().getSimpleName() : failure.getMessage();
            RobotLog.ee("TeleOp", failure, "Robot initialization failed");
            stopRobot();
        }
    }

    @Override
    public void disabledPeriodic() {
        initializeRobot();
        if (!initialized) {
            showInitializationError();
            return;
        }
        controls.update();
        if (controls.dpadLeft.justPressed()) {
            selectedStartPose = leftStartPose();
            robot.setPose(selectedStartPose);
        } else if (controls.dpadRight.justPressed()) {
            selectedStartPose = rightStartPose();
            robot.setPose(selectedStartPose);
        } else if (controls.dpadDown.justPressed()) {
            selectedStartPose = PoseStorage.loadPose();
            robot.setPose(selectedStartPose);
        }

        robot.update();
        logPsiKit();
        addInitTelemetry();
    }

    @Override
    public void start() {
        initializeRobot();
        if (!initialized) {
            showInitializationError();
            return;
        }
        robot.vision.start();
        onRobotStart();
    }

    @Override
    public void periodic() {
        if (!initialized) {
            showInitializationError();
            return;
        }
        controls.update();
        handleDrivingButtons();
        updateAutoAim();
        handlePoseReset();
        handleIntakeAndTransferButtons();

        updateAimPipeline();
        robot.drive.runManual(controls);
        robot.update();
        onRobotUpdated();
        logPsiKit();
        addDriverTelemetry();
        onRobotLoop();
        telemetry.update();
    }

    private void handleDrivingButtons() {
        if (controls.triangle.justPressed() && robot.drive.isManual()) {
            robot.drive.toggleFieldCentric();
        }
    }

    private void updateAutoAim() {
        if (controls.leftTrigger.getValue() > 0.5 && robot.drive.isManual()) {
            robot.drive.startAiming();
        } else {
            robot.drive.stopAiming();
        }
    }

    private void handlePoseReset() {
        if (controls.leftStickButton.justPressed()) {
            robot.setPose(robot.follower.getPose().getX() < 72.0
                    ? leftStartPose() : rightStartPose());
        }
    }

    private void updateAimPipeline() {
        if (!robot.drive.isAiming()) {
            return;
        }

        int pipeline = robot.drive.isUsingFirstAimTarget()
                ? basket1Pipeline
                : basket2Pipeline;
        robot.vision.selectPipeline(pipeline);
    }

    private void handleIntakeAndTransferButtons() {
        if (controls.rightBumper.isPressed()) {
            robot.intake.turnOn(false);
            robot.transfer.turnOn(false);
        } else if (controls.leftBumper.isPressed()) {
            robot.intake.turnOn(true);
            robot.transfer.turnOn(true);
        } else {
            robot.intake.turnOff();
            robot.transfer.turnOff();
        }
    }

    private void stopRobot() {
        if (stopped) return;
        stopped = true;
        try {
            robot.stop();
        } finally {
            try {
                onRobotStopping();
            } finally {
                try {
                    if (robot.follower != null) PoseStorage.savePose(robot.follower.getPose());
                } finally {
                    robot.release();
                }
            }
        }
    }

    // NextFTC guarantees afterEnd even if a loop fails. The recorder only closes its file here.
    private static final class ExitHook implements OpModeHook {
        private AllianceDrivingTeleOp opMode;

        @Override public void afterEnd() {
            opMode.stopRobot();
        }
    }

    protected void onRobotInit() {}
    protected void onRobotStart() {}
    protected void onRobotUpdated() {}
    protected void onRobotLoop() {}
    protected void onRobotStopping() {}

    private void logPsiKit() {
        if (robot.follower != null) {
            Pose pose = robot.follower.getPose();
            PedroFollowerOdometryLogger.log("follower", pose.getX(), pose.getY(), pose.getHeading());
        }
        Logger.recordOutput("Robot/FieldCentric", robot.drive.isFieldCentric());
        Logger.recordOutput("Robot/ManualDrive", robot.drive.isManual());
        Logger.recordOutput("Drive/Aiming", robot.drive.isAiming());
        Logger.recordOutput("Drive/AimTarget", robot.drive.getSelectedAimTargetNumber());
        Logger.recordOutput("Drive/Cmd/LeftStickY", controls.leftStickY);
        Logger.recordOutput("Drive/Cmd/RightStickX", controls.rightStickX);
        Logger.recordOutput("Vision/Tx", LimelightController.getAimTx());
        Logger.recordOutput("Vision/SeesTag", LimelightController.seesArpilTag());
        Logger.recordOutput("Intake/On", robot.intake.isOn());
        Logger.recordOutput("Transfer/On", robot.transfer.isOn());
    }

    private void addInitTelemetry() {
        telemetry.addData("Alliance", allianceName);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start pose", "(%.1f, %.1f, %.0f deg)",
                selectedStartPose.getX(), selectedStartPose.getY(),
                Math.toDegrees(selectedStartPose.getHeading()));
        telemetry.addLine("INIT: D-pad left/right selects the physical starting side, down selects saved pose");
        if (!robot.transfer.isAvailable()) {
            telemetry.addLine("Transfer disabled: mechanism not installed (TRANSFER_ENABLED=false)");
        }
        robot.drive.telemetry(telemetry);
        if (!robot.drive.isSelectedAimTargetConfigured()) {
            telemetry.addLine("WARNING: enter this alliance's basket coordinates in Constants");
        }
        onRobotLoop();
        telemetry.update();
    }

    private void showInitializationError() {
        telemetry.addData("Alliance", allianceName);
        telemetry.addData("Status", "INIT FAILED - driving blocked");
        telemetry.addData("Initialization error", initializationError);
        telemetry.addLine("Fix the active robot configuration, press STOP, then INIT again");
        telemetry.update();
    }

    private void addDriverTelemetry() {
        telemetry.addData("Alliance", allianceName);
        robot.drive.telemetry(telemetry);
        robot.intake.telemetry(telemetry);
        robot.transfer.telemetry(telemetry);
        robot.vision.telemetry(telemetry);
        telemetry.addLine("Triangle: field centric");
        telemetry.addLine(robot.transfer.isAvailable()
                ? "Hold R1: intake + transfer forward | Hold L1: reverse | Release: stop"
                : "Hold R1: intake forward | Hold L1: reverse | Release: stop (transfer disabled)");
        telemetry.addLine("L2 > 50%: autoaim | Basket: automatic by Y | L3: reset pose");
        telemetry.addLine("Limelight pipeline follows the aim basket");
    }

    private static Pose leftStartPose() {
        return new Pose(Constants.LEFT_START_X, Constants.START_Y,
                Math.toRadians(Constants.START_HEADING_DEG));
    }

    private static Pose rightStartPose() {
        return new Pose(Constants.RIGHT_START_X, Constants.START_Y,
                Math.toRadians(Constants.START_HEADING_DEG));
    }

    private static Pose centerPose() {
        return new Pose(Constants.CENTER_X, Constants.CENTER_Y,
                Math.toRadians(Constants.CENTER_HEADING_DEG));
    }
}
