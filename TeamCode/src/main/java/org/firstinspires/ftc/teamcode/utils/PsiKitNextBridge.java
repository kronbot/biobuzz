package org.firstinspires.ftc.teamcode.utils;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.FtcLoggingSession;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.LinkedHashMap;
import java.util.Map;

import dev.nextftc.robot.opmode.OpModeHook;

/**
 * Connects NextFTC to PsiKit logging.
 *
 * <p>NextFTC registers OpModes as {@code BoundNextOpMode} (a {@link LinearOpMode}), so PsiKit's
 * {@code @PsiKitAutoLog} never sees user OpMode classes, and its linear path starts sessions too
 * late (at Play) to wrap the HardwareMap before NextFTC maps devices during INIT. This bridge:
 *
 * <ul>
 *   <li>Starts an {@link FtcLoggingSession} from {@code onOpModePreInit} — before
 *       {@code runOpMode()} runs — so the HardwareMap is wrapped before any device is mapped.</li>
 *   <li>Provides {@link Hook}, a NextFTC {@link OpModeHook} that ticks
 *       {@code Logger.periodicBeforeUser()} / {@code FtcLoggingSession.logOncePerLoop()} /
 *       {@code Logger.periodicAfterUser()} around the user's init and periodic callbacks.</li>
 *   <li>Ends the session on OpMode stop (and from {@code afterEnd}).</li>
 * </ul>
 *
 * <p>Iterative OpModes (e.g. Pedro's {@code Tuning}) are handled by PsiKit's own
 * {@code @PsiKitAutoLog} wrapper — this bridge only processes linear OpModes, and defers any
 * linear OpMode that is itself annotated with {@code @PsiKitAutoLog}.
 *
 * <p>Do not enable {@code PsiKitAutoLogSettings.enabledByDefault} — that would make PsiKit start a
 * second, conflicting session on {@code BoundNextOpMode} at Play.
 */
public final class PsiKitNextBridge {

    /**
     * Live RLOG streaming port for AdvantageScope. Set to {@code 0} to disable streaming.
     * Live streaming is NOT allowed during competition — disable it before your matches.
     */
    public static final int RLOG_PORT = 5800;

    private static final String RLOG_FOLDER = "/sdcard/FIRST/PsiKit/";

    private static final Listener LISTENER = new Listener();
    private static final Map<String, String> PENDING_METADATA = new LinkedHashMap<>();

    private static OpModeManagerImpl manager;
    private static volatile FtcLoggingSession session;
    private static volatile LinearOpMode currentOpMode;
    private static long tickStartNs;
    private static long userStartNs;

    private PsiKitNextBridge() {
    }

    /**
     * Records metadata for PsiKit sessions. Must be called <b>before</b> the OpMode starts (for
     * example from a static initializer or an {@code @OnCreateEventLoop} hook) — PsiKit silently
     * ignores {@code Logger.recordMetadata(...)} calls made after logging has started.
     */
    public static synchronized void recordMetadata(String key, String value) {
        PENDING_METADATA.put(key, value);
    }

    @OnCreateEventLoop
    public static void onCreateEventLoop(Context context, FtcEventLoop eventLoop) {
        OpModeManagerImpl opModeManager = eventLoop.getOpModeManager();
        // Idempotent: keeps us from stacking duplicate listeners across event-loop reloads.
        opModeManager.unregisterListener(LISTENER);
        opModeManager.registerListener(LISTENER);
        manager = opModeManager;
    }

    private static synchronized Map<String, String> metadataSnapshot() {
        return new LinkedHashMap<>(PENDING_METADATA);
    }

    private static synchronized void startSession(LinearOpMode opMode) {
        endSessionLocked();

        final Map<String, String> metadata = metadataSnapshot();
        final String registeredName = manager != null
                ? manager.getActiveOpModeName()
                : opMode.getClass().getSimpleName();

        FtcLoggingSession newSession = new FtcLoggingSession();
        newSession.start(
                opMode,
                RLOG_PORT,
                "", // blank filename -> default (OpModeName_log_timestamp.rlog)
                true, // writeFile
                RLOG_FOLDER,
                null, // replaySource
                opMode, // metadataOpMode
                () -> {
                    Logger.recordMetadata("OpMode Name", registeredName);
                    for (Map.Entry<String, String> entry : metadata.entrySet()) {
                        Logger.recordMetadata(entry.getKey(), entry.getValue());
                    }
                    return kotlin.Unit.INSTANCE;
                });
        session = newSession;
        currentOpMode = opMode;
    }

    private static synchronized void endSession() {
        endSessionLocked();
    }

    private static void endSessionLocked() {
        FtcLoggingSession previous = session;
        session = null;
        currentOpMode = null;
        if (previous != null) {
            previous.end();
        }
    }

    private static void tickBefore() {
        FtcLoggingSession active = session;
        LinearOpMode opMode = currentOpMode;
        if (active == null || opMode == null) {
            return;
        }
        tickStartNs = System.nanoTime();
        Logger.periodicBeforeUser();
        active.logOncePerLoop(opMode);
        userStartNs = System.nanoTime();
    }

    private static void tickAfter() {
        if (session == null) {
            return;
        }
        long now = System.nanoTime();
        Logger.periodicAfterUser(
                (now - userStartNs) / 1_000_000_000.0,
                (userStartNs - tickStartNs) / 1_000_000_000.0);
    }

    /**
     * Pass to a NextOpMode constructor:
     * {@code super(robot, stopRobotOnExit(robot), new PsiKitNextBridge.Hook())}.
     */
    public static final class Hook implements OpModeHook {
        @Override
        public void beforeDisabled() {
            tickBefore();
        }

        @Override
        public void afterDisabled() {
            tickAfter();
        }

        @Override
        public void beforePeriodic() {
            tickBefore();
        }

        @Override
        public void afterPeriodic() {
            tickAfter();
        }

        @Override
        public void afterEnd() {
            endSession();
        }
    }

    private static final class Listener implements OpModeManagerNotifier.Notifications {
        @Override
        public void onOpModePreInit(OpMode opMode) {
            if (!(opMode instanceof LinearOpMode)) {
                return;
            }
            // A linear OpMode using PsiKit's own autolog manages its session itself.
            if (opMode.getClass().isAnnotationPresent(PsiKitAutoLog.class)) {
                return;
            }
            startSession((LinearOpMode) opMode);
        }

        @Override
        public void onOpModePreStart(OpMode opMode) {
        }

        @Override
        public void onOpModePostStop(OpMode opMode) {
            if (opMode == currentOpMode) {
                endSession();
            }
        }
    }
}
