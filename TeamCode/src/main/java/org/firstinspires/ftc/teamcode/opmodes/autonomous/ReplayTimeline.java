package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import java.util.List;

/** Recording time and frame cursor, kept separate from wall time during a tag wait. */
final class ReplayTimeline {
    static final double TAG_WAIT_TIMEOUT_SEC = 3.0;

    private final List<ReplayOp.RobotFrame> frames;
    private int frameIndex = -1;
    private double timeOffset;
    private double frozenTime;
    private double waitStart;
    private boolean waiting;
    private int waitPointCount;
    private String lastWaitResult = "NONE";

    ReplayTimeline(List<ReplayOp.RobotFrame> frames) {
        this.frames = frames;
    }

    /** Advance one frame at a time so pose resets and wait markers cannot be skipped. */
    boolean advance(double wallTime) {
        int nextIndex = frameIndex + 1;
        if (waiting || nextIndex >= frames.size()
                || frames.get(nextIndex).timestamp - frames.get(0).timestamp > elapsed(wallTime)) {
            return false;
        }
        frameIndex = nextIndex;
        if (frames.get(frameIndex).waitForAprilTag) {
            frozenTime = frames.get(frameIndex).timestamp - frames.get(0).timestamp;
            waitStart = wallTime;
            waiting = true;
            waitPointCount++;
            lastWaitResult = "WAITING";
        }
        return true;
    }

    boolean finishWait(double wallTime, boolean tagDetected) {
        if (!waiting || (!tagDetected && waitElapsed(wallTime) < TAG_WAIT_TIMEOUT_SEC)) {
            return false;
        }
        // Discard both the pause and any loop overshoot beyond the marker timestamp.
        timeOffset = wallTime - frozenTime;
        waiting = false;
        lastWaitResult = tagDetected ? "TAG DETECTED" : "TIMEOUT";
        return true;
    }

    double elapsed(double wallTime) {
        return waiting ? frozenTime : wallTime - timeOffset;
    }

    double waitElapsed(double wallTime) {
        return wallTime - waitStart;
    }

    boolean isWaiting() { return waiting; }
    int frameIndex() { return frameIndex; }
    int waitPointCount() { return waitPointCount; }
    String lastWaitResult() { return lastWaitResult; }
}
