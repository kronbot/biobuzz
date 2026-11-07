package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.utils.RecordingFormat.Frame;

/** Timed feedforward plus bounded odometry feedback, independent of the OpMode lifecycle. */
public final class ReplayController {
    private double filteredVelocityErrorX;
    private double filteredVelocityErrorY;
    private double filteredOmegaError;

    public void reset() {
        filteredVelocityErrorX = 0;
        filteredVelocityErrorY = 0;
        filteredOmegaError = 0;
    }

    /** aimTurn is NaN when target lock is off; otherwise it comes from DriveController. */
    public Command calculate(Frame a, Frame b, double timestamp,
                             double x, double y, double heading,
                             double velocityX, double velocityY, double omega,
                             double voltage, double aimTurn) {
        double amount = b.timestamp > a.timestamp
                ? clip((timestamp - a.timestamp) / (b.timestamp - a.timestamp), 0, 1) : 0;
        // An L3 reset changes coordinates, not the robot's physical position.
        if (a.poseResetId != b.poseResetId) amount = 0;
        Command command = new Command();
        command.targetX = lerp(a.x, b.x, amount);
        command.targetY = lerp(a.y, b.y, amount);
        command.targetHeading = wrap(a.heading + wrap(b.heading - a.heading) * amount);
        double errorX = command.targetX - x;
        double errorY = command.targetY - y;
        command.positionError = Math.hypot(errorX, errorY);
        command.headingError = wrap(command.targetHeading - heading);

        double alpha = Constants.REPLAY_VELOCITY_FILTER_ALPHA;
        filteredVelocityErrorX += alpha
                * (lerp(a.velocityX, b.velocityX, amount) - velocityX - filteredVelocityErrorX);
        filteredVelocityErrorY += alpha
                * (lerp(a.velocityY, b.velocityY, amount) - velocityY - filteredVelocityErrorY);
        filteredOmegaError += alpha
                * (lerp(a.angularVelocity, b.angularVelocity, amount) - omega - filteredOmegaError);

        double correctionX = Constants.REPLAY_TRANSLATION_KP * errorX
                + Constants.REPLAY_TRANSLATION_KD * filteredVelocityErrorX;
        double correctionY = Constants.REPLAY_TRANSLATION_KP * errorY
                + Constants.REPLAY_TRANSLATION_KD * filteredVelocityErrorY;
        double magnitude = Math.hypot(correctionX, correctionY);
        if (magnitude > Constants.REPLAY_MAX_TRANSLATION_CORRECTION) {
            double scale = Constants.REPLAY_MAX_TRANSLATION_CORRECTION / magnitude;
            correctionX *= scale;
            correctionY *= scale;
        }
        if (!a.fieldCentric) {
            double forward = Math.cos(heading) * correctionX + Math.sin(heading) * correctionY;
            correctionY = -Math.sin(heading) * correctionX + Math.cos(heading) * correctionY;
            correctionX = forward;
        }

        double voltageScale = voltage > 1 && Double.isFinite(voltage)
                ? clip(lerp(a.voltage, b.voltage, amount) / voltage,
                Constants.REPLAY_MIN_VOLTAGE_SCALE, Constants.REPLAY_MAX_VOLTAGE_SCALE) : 1;
        // Do not blend joystick commands across a field-centric or target-lock transition.
        double driveAmount = a.sameDiscreteState(b) ? amount : 0;
        command.forward = lerp(a.driveForward, b.driveForward, driveAmount) * voltageScale
                + correctionX;
        command.strafe = lerp(a.driveStrafe, b.driveStrafe, driveAmount) * voltageScale
                + correctionY;
        double correctionTurn = clip(Constants.REPLAY_ROTATION_KP * command.headingError
                        + Constants.REPLAY_ROTATION_KD * filteredOmegaError,
                -Constants.REPLAY_MAX_ROTATION_CORRECTION,
                Constants.REPLAY_MAX_ROTATION_CORRECTION);
        command.turn = clip((Double.isFinite(aimTurn) ? aimTurn
                : lerp(a.driveTurn, b.driveTurn, driveAmount) * voltageScale) + correctionTurn,
                -1, 1);
        magnitude = Math.hypot(command.forward, command.strafe);
        if (magnitude > 1) {
            command.forward /= magnitude;
            command.strafe /= magnitude;
        }
        return command;
    }

    public static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double clip(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    private static double lerp(double a, double b, double amount) {
        return a + (b - a) * amount;
    }

    public static final class Command {
        public double forward;
        public double strafe;
        public double turn;
        public double targetX;
        public double targetY;
        public double targetHeading;
        public double positionError;
        public double headingError;
    }
}
