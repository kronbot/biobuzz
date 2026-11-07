package org.firstinspires.ftc.teamcode.utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Biobuzz CSV contract. Intentionally distinct from the old test robot's format. */
public final class RecordingFormat {
    public static final String HEADER = "Time,X,Y,Heading,Voltage,DriveFwd,DriveStr,DriveTurn,"
            + "VelX,VelY,AngularVelocity,FieldCentric,AimingAtTarget,AimPipeline,"
            + "IntakePower,TransferPower,PoseResetId,Alliance";
    public static final int COLUMN_COUNT = 18;

    private RecordingFormat() {}

    public static List<Frame> load(File file, String alliance) throws IOException {
        List<Frame> frames = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            if (!HEADER.equals(reader.readLine())) {
                throw new IOException("Unsupported recording format; record again with Biobuzz");
            }
            String line;
            int lineNumber = 1;
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                if (line.trim().isEmpty()) continue;
                try {
                    Frame frame = Frame.parse(line);
                    if (!alliance.equals(frame.alliance)) {
                        throw new IllegalArgumentException("Recording belongs to " + frame.alliance);
                    }
                    if (frames.isEmpty()) {
                        if (frame.poseResetId != 0) {
                            throw new IllegalArgumentException("First pose reset id must be zero");
                        }
                    } else {
                        Frame previous = frames.get(frames.size() - 1);
                        if (frame.timestamp <= previous.timestamp) {
                            throw new IllegalArgumentException("Timestamps must increase strictly");
                        }
                        if (frame.poseResetId < previous.poseResetId
                                || frame.poseResetId > previous.poseResetId + 1) {
                            throw new IllegalArgumentException("Invalid pose reset sequence");
                        }
                    }
                    frames.add(frame);
                } catch (IllegalArgumentException e) {
                    throw new IOException("CSV line " + lineNumber + ": " + e.getMessage(), e);
                }
            }
        }
        if (frames.size() < 2) {
            throw new IOException("Recording needs at least two frames");
        }
        return frames;
    }

    /** Discrete states are held until the next frame; only motion is interpolated. */
    public static final class Frame {
        public double timestamp;
        public double x;
        public double y;
        public double heading;
        public double voltage;
        public double driveForward;
        public double driveStrafe;
        public double driveTurn;
        public double velocityX;
        public double velocityY;
        public double angularVelocity;
        public boolean fieldCentric;
        public boolean aiming;
        public int aimPipeline;
        public double intakePower;
        public double transferPower;
        public int poseResetId;
        public String alliance;

        public String toCsv() {
            return String.format(Locale.US,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                            + "%d,%d,%d,%.6f,%.6f,%d,%s",
                    timestamp, x, y, heading, voltage, driveForward, driveStrafe, driveTurn,
                    velocityX, velocityY, angularVelocity, fieldCentric ? 1 : 0,
                    aiming ? 1 : 0, aimPipeline, intakePower, transferPower, poseResetId, alliance);
        }

        public static Frame parse(String line) {
            String[] values = line.split(",", -1);
            if (values.length != COLUMN_COUNT) {
                throw new IllegalArgumentException("Expected " + COLUMN_COUNT + " columns");
            }
            // Validate every number, including velocities and motor powers, before any movement.
            double[] numbers = new double[17];
            for (int i = 0; i < numbers.length; i++) {
                numbers[i] = Double.parseDouble(values[i]);
                if (!Double.isFinite(numbers[i])) {
                    throw new IllegalArgumentException("Non-finite value in column " + (i + 1));
                }
            }
            Frame frame = new Frame();
            frame.timestamp = numbers[0];
            frame.x = numbers[1];
            frame.y = numbers[2];
            frame.heading = numbers[3];
            frame.voltage = numbers[4];
            frame.driveForward = power(numbers[5]);
            frame.driveStrafe = power(numbers[6]);
            frame.driveTurn = power(numbers[7]);
            frame.velocityX = numbers[8];
            frame.velocityY = numbers[9];
            frame.angularVelocity = numbers[10];
            frame.fieldCentric = booleanValue(values[11]);
            frame.aiming = booleanValue(values[12]);
            frame.aimPipeline = Integer.parseInt(values[13]);
            frame.intakePower = power(numbers[14]);
            frame.transferPower = power(numbers[15]);
            frame.poseResetId = Integer.parseInt(values[16]);
            frame.alliance = values[17];
            if (frame.timestamp < 0 || frame.voltage <= 1 || frame.poseResetId < 0) {
                throw new IllegalArgumentException("Invalid time, voltage, or pose reset id");
            }
            // Pipeline numbers are part of this version of the recording format.
            boolean blue = "BLUE".equals(frame.alliance);
            boolean red = "RED".equals(frame.alliance);
            if ((!blue && !red)
                    || (blue && frame.aimPipeline != Constants.LIMELIGHT_BLUE_1_PIPELINE
                    && frame.aimPipeline != Constants.LIMELIGHT_BLUE_2_PIPELINE)
                    || (red && frame.aimPipeline != Constants.LIMELIGHT_RED_1_PIPELINE
                    && frame.aimPipeline != Constants.LIMELIGHT_RED_2_PIPELINE)) {
                throw new IllegalArgumentException("Invalid alliance or Limelight pipeline");
            }
            return frame;
        }

        public boolean sameDiscreteState(Frame other) {
            return fieldCentric == other.fieldCentric && aiming == other.aiming
                    && aimPipeline == other.aimPipeline && poseResetId == other.poseResetId
                    && intakePower == other.intakePower && transferPower == other.transferPower;
        }

        private static double power(double value) {
            if (Math.abs(value) > 1) {
                throw new IllegalArgumentException("Motor/drive power outside [-1, 1]");
            }
            return value;
        }

        private static boolean booleanValue(String value) {
            if ("1".equals(value)) return true;
            if ("0".equals(value)) return false;
            throw new IllegalArgumentException("Expected boolean 0 or 1");
        }
    }
}
