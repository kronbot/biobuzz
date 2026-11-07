package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.*;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.json.JSONException;
import org.json.JSONObject;
import org.junit.Test;

public class LimelightControllerTest {
    private static final class FakeResult extends LLResult {
        private long ageMillis;

        FakeResult(boolean valid, int pipeline, boolean hasTag, long ageMillis) throws JSONException {
            super(new JSONObject("{\"v\":" + (valid ? 1 : 0) + ",\"pID\":" + pipeline
                    + ",\"Fiducial\":" + (hasTag ? "[{\"fID\":1}]" : "[]") + "}"));
            this.ageMillis = ageMillis;
        }

        @Override public long getStaleness() { return ageMillis; }
    }

    @Test public void acceptsOnlyLiveFiducialsFromTheExpectedPipeline() throws Exception {
        assertFalse(LimelightController.isFreshAprilTagResult(null, 3));
        assertTrue(LimelightController.isFreshAprilTagResult(new FakeResult(true, 3, true, 0), 3));
        assertTrue(LimelightController.isFreshAprilTagResult(new FakeResult(true, 3, true, 100), 3));
        assertFalse(LimelightController.isFreshAprilTagResult(new FakeResult(true, 3, true, 101), 3));
        assertFalse(LimelightController.isFreshAprilTagResult(new FakeResult(true, 3, true, -1), 3));
        assertFalse(LimelightController.isFreshAprilTagResult(new FakeResult(false, 3, true, 0), 3));
        assertFalse(LimelightController.isFreshAprilTagResult(new FakeResult(true, 4, true, 0), 3));
        assertFalse(LimelightController.isFreshAprilTagResult(new FakeResult(true, 3, false, 0), 3));
    }

    @Test public void repeatedReadsDoNotRefreshAnExpiredDetection() throws Exception {
        FakeResult result = new FakeResult(true, 3, true, 50);
        assertTrue(LimelightController.isFreshAprilTagResult(result, 3));
        result.ageMillis = 150;
        assertFalse(LimelightController.isFreshAprilTagResult(result, 3));
        assertFalse(LimelightController.isFreshAprilTagResult(result, 3));
    }

    @Test public void uninitializedCameraCannotReleaseAWait() {
        assertFalse(new LimelightController().hasFreshAprilTag(3));
    }
}
