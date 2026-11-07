package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.utils.RecordingFormat;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.ArrayList;
import java.nio.file.Files;

public class ReplayOpTest {
    @Rule public TemporaryFolder directory = new TemporaryFolder();

    private String row(double time, boolean blue, double intake, double transfer, boolean reset) {
        return time + ",0.2,0.2,0.2,0.2,9,9,1.5708,13,0.4,-0.2,0.1,0,0,0,10,0,0,1,0,"
                + (blue ? 1 : 0) + "," + intake + "," + transfer + ","
                + (blue ? 3 : 1) + "," + (reset ? 1 : 0) + ",0";
    }

    private File write(String header, String... rows) throws IOException {
        File file = directory.newFile();
        try (FileWriter writer = new FileWriter(file)) {
            writer.write(header + "\n");
            for (String row : rows) writer.write(row + "\n");
        }
        return file;
    }

    @Test public void readsFlushedCsvBeforeWriterIsClosed() throws Exception {
        File file = directory.newFile();
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            writer.write(RecordingFormat.HEADER + "\n");
            writer.write(row(0, true, 0, 0, false) + "\n");
            writer.write(row(0.02, true, -1, 0, true) + "\n");
            writer.flush();
            List<ReplayOp.RobotFrame> frames = ReplayOp.loadRecording(file, true, false);
            assertEquals(2, frames.size());
            assertEquals(-1, frames.get(1).intakePower, 0);
            assertTrue(frames.get(1).poseReset);
        }
    }

    @Test public void redAllianceAndTransferAreLoaded() throws Exception {
        File file = write(RecordingFormat.HEADER,
                row(0, false, 1, -1, false), row(0.02, false, 0, 0, false));
        List<ReplayOp.RobotFrame> frames = ReplayOp.loadRecording(file, false, true);
        assertFalse(frames.get(0).aimingAtBlue);
        assertEquals(-1, frames.get(0).transferPower, 0);
        assertEquals(1, frames.get(0).aimPipeline);
    }

    @Test public void missingFileReportsExactPath() {
        File missing = new File(directory.getRoot(), "missing.csv");
        IOException error = assertThrows(IOException.class,
                () -> ReplayOp.loadRecording(missing, true, false));
        assertTrue(error.getMessage().contains(missing.getAbsolutePath()));
    }

    @Test public void rejectsWrongAllianceAndAbsentRequiredTransfer() throws Exception {
        File file = write(RecordingFormat.HEADER,
                row(0, true, 0, 1, false), row(0.02, true, 0, 0, false));
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(file, false, true));
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(file, true, false));
    }

    @Test public void rejectsOldFormatAndTruncatedRecording() throws Exception {
        File old = write("Time,X,Y", "0,9,9");
        File truncated = write(RecordingFormat.HEADER, row(0, true, 0, 0, false), "0.02,9");
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(old, true, false));
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(truncated, true, false));
    }

    @Test public void rejectsBackwardTimeAndNoActiveRecording() throws Exception {
        File backwards = write(RecordingFormat.HEADER,
                row(1, true, 0, 0, false), row(0, true, 0, 0, false));
        File empty = write(RecordingFormat.HEADER);
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(backwards, true, false));
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(empty, true, false));
    }

    @Test public void rejectsNonFiniteVelocityBeforeDriving() throws Exception {
        String[] bad = row(0.02, true, 0, 0, false).split(",");
        bad[15] = "NaN";
        File file = write(RecordingFormat.HEADER, row(0, true, 0, 0, false),
                String.join(",", bad), row(0.04, true, 0, 0, false));
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(file, true, false));
    }

    @Test public void recoversOneTruncatedFinalRowWithoutChangingTheFile() throws Exception {
        File file = write(RecordingFormat.HEADER,
                row(0, true, 0, 0, false), row(0.02, true, 0, 0, false));
        try (FileWriter writer = new FileWriter(file, true)) {
            writer.write("0.04,0.2,0.2,0.2"); // Interrupted write: no remaining columns or newline.
        }
        byte[] original = Files.readAllBytes(file.toPath());
        List<String> warnings = new ArrayList<>();
        List<ReplayOp.RobotFrame> frames = ReplayOp.loadRecording(file, true, false, warnings::add);
        assertEquals(2, frames.size());
        assertEquals(1, warnings.size());
        assertTrue(warnings.get(0).contains("line 4"));
        assertArrayEquals(original, Files.readAllBytes(file.toPath()));
    }

    @Test public void recoversOneFinalRowWithNonFiniteSensorValue() throws Exception {
        String[] invalid = row(0.04, true, 0, 0, false).split(",");
        invalid[17] = "NaN";
        File file = write(RecordingFormat.HEADER,
                row(0, true, 0, 0, false), row(0.02, true, 0, 0, false),
                String.join(",", invalid));
        List<String> warnings = new ArrayList<>();
        assertEquals(2, ReplayOp.loadRecording(file, true, false, warnings::add).size());
        assertTrue(warnings.get(0).contains("Non-finite"));
    }

    @Test public void keepsCompleteFinalRowEvenWithoutNewline() throws Exception {
        File file = write(RecordingFormat.HEADER,
                row(0, true, 0, 0, false), row(0.02, true, 0, 0, false));
        try (FileWriter writer = new FileWriter(file, true)) {
            writer.write(row(0.04, true, 0, 0, false));
        }
        List<String> warnings = new ArrayList<>();
        assertEquals(3, ReplayOp.loadRecording(file, true, false, warnings::add).size());
        assertTrue(warnings.isEmpty());
    }

    @Test public void rejectsTruncatedRowInMiddleAndMultipleBrokenFinalRows() throws Exception {
        for (String following : new String[]{row(0.06, true, 0, 0, false), "0.06,9"}) {
            File file = write(RecordingFormat.HEADER,
                    row(0, true, 0, 0, false), row(0.02, true, 0, 0, false),
                    "0.04,9", following);
            assertThrows(IOException.class, () -> ReplayOp.loadRecording(file, true, false));
        }
    }

    @Test public void rejectsWrongAllianceAndUnavailableTransferEvenOnFinalRow() throws Exception {
        for (String last : new String[]{row(0.04, false, 0, 0, false),
                row(0.04, true, 0, 1, false)}) {
            File file = write(RecordingFormat.HEADER,
                    row(0, true, 0, 0, false), row(0.02, true, 0, 0, false), last);
            assertThrows(IOException.class, () -> ReplayOp.loadRecording(file, true, false));
        }
    }

    @Test public void rejectsInvalidSensorRowsBeforeRecording() {
        for (int column : new int[]{5, 8, 15, 17}) {
            String[] values = row(0, true, 0, 0, false).split(",");
            values[column] = "NaN";
            assertThrows(NumberFormatException.class, () -> RecordingFormat.validateRow(values));
        }
        String[] invalidVoltage = row(0, true, 0, 0, false).split(",");
        invalidVoltage[8] = "0";
        assertThrows(NumberFormatException.class, () -> RecordingFormat.validateRow(invalidVoltage));
    }

    @Test public void manualFeedforwardMatchesRecordedSticks() throws Exception {
        File file = write(RecordingFormat.HEADER,
                row(0, true, 0, 0, false), row(0.02, true, 0, 0, false));
        ReplayOp.RobotFrame frame = ReplayOp.loadRecording(file, true, false).get(0);
        // Commands are preserved even for field-centric driving; the loop chooses the frame.
        assertEquals(0.4, frame.driveForward, 0);
        assertEquals(-0.2, frame.driveStrafe, 0);
        assertEquals(0.1, frame.driveTurn, 0);
        assertTrue(frame.fieldCentric);
    }

    @Test public void readsLegacy25ColumnRecordingWithoutWaitMarkers() throws Exception {
        String first = row(0, true, 0, 0, false);
        String second = row(0.02, true, -1, 0, true);
        File file = write(RecordingFormat.LEGACY_HEADER,
                first.substring(0, first.lastIndexOf(',')),
                second.substring(0, second.lastIndexOf(',')));
        List<ReplayOp.RobotFrame> frames = ReplayOp.loadRecording(file, true, false);
        assertFalse(frames.get(0).waitForAprilTag);
        assertFalse(frames.get(1).waitForAprilTag);
        assertTrue(frames.get(1).poseReset);
        assertEquals(-1, frames.get(1).intakePower, 0);
    }

    @Test public void loadsMultipleWaitMarkersAlongsideMechanismCommands() throws Exception {
        String first = row(0, true, 1, 0, false);
        String last = row(0.04, true, -1, 0, false);
        File file = write(RecordingFormat.HEADER,
                first.substring(0, first.length() - 1) + "1",
                row(0.02, true, 0, 0, false),
                last.substring(0, last.length() - 1) + "1");
        List<ReplayOp.RobotFrame> frames = ReplayOp.loadRecording(file, true, false);
        assertTrue(frames.get(0).waitForAprilTag);
        assertFalse(frames.get(1).waitForAprilTag);
        assertTrue(frames.get(2).waitForAprilTag);
        assertEquals(1, frames.get(0).intakePower, 0);
        assertEquals(-1, frames.get(2).intakePower, 0);
    }

    @Test public void rejectsInvalidWaitMarkersBeforeRecordingAndInsideReplay() throws Exception {
        for (String flag : new String[]{"-1", "2", "0.5", "1.0", "true", "", "NaN"}) {
            String[] values = row(0.02, true, 0, 0, false).split(",", -1);
            values[25] = flag;
            assertThrows(NumberFormatException.class, () -> RecordingFormat.validateRow(values));
            File file = write(RecordingFormat.HEADER, row(0, true, 0, 0, false),
                    String.join(",", values), row(0.04, true, 0, 0, false));
            assertThrows(IOException.class, () -> ReplayOp.loadRecording(file, true, false));
        }
    }

    @Test public void requiresRowWidthToMatchHeader() throws Exception {
        String fullRow = row(0, true, 0, 0, false);
        String legacyRow = fullRow.substring(0, fullRow.lastIndexOf(','));
        File newHeader = write(RecordingFormat.HEADER, legacyRow, legacyRow);
        File oldHeader = write(RecordingFormat.LEGACY_HEADER, fullRow, fullRow);
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(newHeader, true, false));
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(oldHeader, true, false));
    }

    @Test public void waitPipelineUsesRecordedAllianceAndBasketRatherThanAimPipeline() {
        for (boolean blue : new boolean[]{true, false}) {
            for (double y : new double[]{71.99, 72, 100}) {
                String[] values = row(0, blue, 0, 0, false).split(",");
                values[6] = Double.toString(y);
                values[23] = "9"; // A stale aiming pipeline must not select the wait target.
                int expected = blue ? (y < 72 ? 3 : 4) : (y < 72 ? 1 : 2);
                assertEquals(expected, ReplayOp.waitPipeline(new ReplayOp.RobotFrame(values)));
            }
        }
    }
}
