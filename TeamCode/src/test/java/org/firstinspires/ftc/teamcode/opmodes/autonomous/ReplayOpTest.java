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

public class ReplayOpTest {
    @Rule public TemporaryFolder directory = new TemporaryFolder();

    private String row(double time, boolean blue, double intake, double transfer, boolean reset) {
        return time + ",0.2,0.2,0.2,0.2,9,9,1.5708,13,0.4,-0.2,0.1,0,0,0,10,0,0,1,0,"
                + (blue ? 1 : 0) + "," + intake + "," + transfer + ","
                + (blue ? 3 : 1) + "," + (reset ? 1 : 0);
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
        File file = write(RecordingFormat.HEADER, row(0, true, 0, 0, false), String.join(",", bad));
        assertThrows(IOException.class, () -> ReplayOp.loadRecording(file, true, false));
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
}
