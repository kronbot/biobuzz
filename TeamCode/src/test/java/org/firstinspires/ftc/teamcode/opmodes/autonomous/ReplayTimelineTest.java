package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.junit.Assert.*;

import org.junit.Test;

import java.util.Arrays;

public class ReplayTimelineTest {
    private ReplayOp.RobotFrame frame(double time, boolean wait) {
        return new ReplayOp.RobotFrame((time
                + ",0,0,0,0,9,9,1.5708,13,0,0,0,0,0,0,0,0,0,0,0,1,0,0,3,0,"
                + (wait ? 1 : 0)).split(","));
    }

    @Test public void tagAlreadyVisibleResumesAtTheMarkerWithoutRepeatingIt() {
        ReplayTimeline timeline = new ReplayTimeline(Arrays.asList(frame(0, true), frame(1, false)));
        assertTrue(timeline.advance(0));
        assertTrue(timeline.isWaiting());
        assertTrue(timeline.finishWait(0, true));
        assertEquals("TAG DETECTED", timeline.lastWaitResult());
        assertFalse(timeline.advance(0));
        assertFalse(timeline.finishWait(0, true));
        assertTrue(timeline.advance(1));
        assertEquals(1, timeline.waitPointCount());
        assertFalse(timeline.isWaiting());
    }

    @Test public void freezesRecordingTimeAndResumesRemainingFramesAfterDetection() {
        ReplayTimeline timeline = new ReplayTimeline(Arrays.asList(
                frame(10, false), frame(15, true), frame(16, false)));
        assertTrue(timeline.advance(0));
        assertFalse(timeline.advance(4));
        assertTrue(timeline.advance(5));
        assertFalse(timeline.finishWait(7, false));
        assertEquals(5, timeline.elapsed(7), 0);
        assertFalse(timeline.advance(7));
        assertTrue(timeline.finishWait(7, true));
        assertEquals(5, timeline.elapsed(7), 0);
        assertEquals(5.5, timeline.elapsed(7.5), 0);
        assertFalse(timeline.advance(7.5));
        assertTrue(timeline.advance(8));
        assertEquals(2, timeline.frameIndex());
    }

    @Test public void timesOutAtThreeSecondsAndContinuesWithoutDetection() {
        ReplayTimeline timeline = new ReplayTimeline(Arrays.asList(frame(0, false), frame(5, true), frame(6, false)));
        assertTrue(timeline.advance(5));
        assertTrue(timeline.advance(5));
        assertFalse(timeline.finishWait(7.999, false));
        assertTrue(timeline.finishWait(8, false));
        assertEquals("TIMEOUT", timeline.lastWaitResult());
        assertEquals(5, timeline.elapsed(8), 0);
        assertTrue(timeline.advance(9));
    }

    @Test public void slowLoopStopsAtEarliestMarkerEvenBeyondTheRecordingEnd() {
        ReplayTimeline timeline = new ReplayTimeline(Arrays.asList(
                frame(0, false), frame(1, true), frame(2, true), frame(3, false)));
        assertTrue(timeline.advance(10));
        assertTrue(timeline.advance(10));
        assertEquals(1, timeline.frameIndex());
        assertEquals(1, timeline.elapsed(10), 0);
        assertFalse(timeline.advance(10));
        assertTrue(timeline.finishWait(12, true));
        assertFalse(timeline.advance(12));
        assertTrue(timeline.advance(13));
        assertEquals(2, timeline.frameIndex());
        assertEquals(2, timeline.waitPointCount());
        assertFalse(timeline.finishWait(15.999, false));
        assertTrue(timeline.finishWait(16, false));
        assertTrue(timeline.advance(17));
        assertEquals(3, timeline.elapsed(17), 0);
    }

    @Test public void handlesFirstAndFinalMarkersEvenAtTheSameTimestamp() {
        ReplayTimeline timeline = new ReplayTimeline(Arrays.asList(frame(1, true), frame(1, true)));
        assertTrue(timeline.advance(10));
        assertFalse(timeline.advance(10));
        assertTrue(timeline.finishWait(10, true));
        assertTrue(timeline.advance(10));
        assertEquals(1, timeline.frameIndex());
        assertTrue(timeline.isWaiting());
        assertTrue(timeline.finishWait(13, false));
        assertFalse(timeline.advance(14));
        assertEquals(2, timeline.waitPointCount());
    }
}
