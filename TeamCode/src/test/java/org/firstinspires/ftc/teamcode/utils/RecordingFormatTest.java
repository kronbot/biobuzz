package org.firstinspires.ftc.teamcode.utils;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.utils.RecordingFormat.Frame;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Locale;

public class RecordingFormatTest {
    @Rule public TemporaryFolder temporary = new TemporaryFolder();

    static Frame sample(double timestamp) {
        Frame frame = new Frame();
        frame.timestamp = timestamp;
        frame.x = 9;
        frame.y = 9;
        frame.heading = Math.PI / 2;
        frame.voltage = 13;
        frame.aimPipeline = Constants.LIMELIGHT_BLUE_1_PIPELINE;
        frame.alliance = "BLUE";
        return frame;
    }

    @Test public void roundTripPreservesReverseAndIndependentMechanisms() {
        Frame original = sample(0.02);
        original.intakePower = -1;
        original.transferPower = 0.6;
        original.driveForward = -0.3;
        original.driveStrafe = 0.4;
        original.driveTurn = -0.2;
        original.velocityX = -12;
        original.velocityY = 3;
        original.angularVelocity = -1.2;
        original.fieldCentric = true;
        original.aiming = true;
        original.poseResetId = 2;
        Frame parsed = Frame.parse(original.toCsv());
        assertEquals(original.toCsv(), parsed.toCsv());
        assertEquals(-1, parsed.intakePower, 0);
        assertEquals(0.6, parsed.transferPower, 0);
    }

    @Test public void csvUsesDecimalPointEvenUnderRomanianLocale() {
        Locale previous = Locale.getDefault();
        try {
            Locale.setDefault(new Locale("ro", "RO"));
            assertEquals(0.02, Frame.parse(sample(0.02).toCsv()).timestamp, 0);
        } finally {
            Locale.setDefault(previous);
        }
    }

    @Test public void loadsValidRecordingWithPoseResetAndPowerRelease() throws Exception {
        Frame first = sample(0);
        Frame on = sample(0.02);
        on.intakePower = 1;
        on.transferPower = 1;
        Frame off = sample(0.025);
        off.poseResetId = 1;
        List<Frame> frames = RecordingFormat.load(write(RecordingFormat.HEADER,
                first.toCsv(), on.toCsv(), off.toCsv()), "BLUE");
        assertEquals(3, frames.size());
        assertEquals(0, frames.get(2).transferPower, 0);
        assertEquals(1, frames.get(2).poseResetId);
    }

    @Test public void rejectsEveryNonFiniteNumericColumn() {
        for (int column = 0; column < 17; column++) {
            for (String invalid : new String[]{"NaN", "Infinity", "-Infinity"}) {
                String[] values = sample(0).toCsv().split(",");
                values[column] = invalid;
                assertThrows(IllegalArgumentException.class,
                        () -> Frame.parse(String.join(",", values)));
            }
        }
    }

    @Test public void rejectsInvalidBooleansPowersAndPipeline() {
        for (int column : new int[]{5, 6, 7, 11, 12, 13, 14, 15, 16}) {
            String[] values = sample(0).toCsv().split(",");
            values[column] = "-2";
            assertThrows(IllegalArgumentException.class,
                    () -> Frame.parse(String.join(",", values)));
        }
    }

    @Test public void rejectsOldTestRobotFormat() throws Exception {
        File file = write("Time,LR,RR,LF,RF", "0,0,0,0,0");
        assertThrows(IOException.class, () -> RecordingFormat.load(file, "BLUE"));
    }

    @Test public void rejectsWrongAllianceBeforeReplay() throws Exception {
        File file = write(RecordingFormat.HEADER, sample(0).toCsv(), sample(0.02).toCsv());
        IOException failure = assertThrows(IOException.class,
                () -> RecordingFormat.load(file, "RED"));
        assertTrue(failure.getMessage().contains("line 2"));
    }

    @Test public void rejectsDuplicateOrBackwardTime() throws Exception {
        for (double timestamp : new double[]{0.02, 0.01}) {
            File file = write(RecordingFormat.HEADER, sample(0.02).toCsv(),
                    sample(timestamp).toCsv());
            assertThrows(IOException.class, () -> RecordingFormat.load(file, "BLUE"));
        }
    }

    @Test public void rejectsMissingFramesAndTruncatedRows() throws Exception {
        for (File file : new File[]{write(RecordingFormat.HEADER),
                write(RecordingFormat.HEADER, sample(0).toCsv()),
                write(RecordingFormat.HEADER, sample(0).toCsv(), "0.02,9,9")}) {
            assertThrows(IOException.class, () -> RecordingFormat.load(file, "BLUE"));
        }
    }

    @Test public void rejectsMissingResetEvents() throws Exception {
        Frame reset = sample(0.02);
        reset.poseResetId = 2;
        File file = write(RecordingFormat.HEADER, sample(0).toCsv(), reset.toCsv());
        assertThrows(IOException.class, () -> RecordingFormat.load(file, "BLUE"));
    }

    private File write(String... lines) throws IOException {
        File file = temporary.newFile();
        try (FileWriter writer = new FileWriter(file)) {
            for (String line : lines) writer.write(line + "\n");
        }
        return file;
    }
}
