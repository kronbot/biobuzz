package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.RecordingFormatTest.sample;
import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.utils.RecordingFormat.Frame;
import org.firstinspires.ftc.teamcode.utils.ReplayController.Command;
import org.junit.Test;

public class ReplayControllerTest {
    @Test public void unchangedPoseReproducesActualRecordedCommands() {
        Frame a = sample(0);
        a.driveForward = 0.4;
        a.driveStrafe = -0.2;
        a.driveTurn = 0.1;
        Command command = atPose(new ReplayController(), a, a, 0, a.x, a.y, a.heading);
        assertEquals(0.4, command.forward, 1e-9);
        assertEquals(-0.2, command.strafe, 1e-9);
        assertEquals(0.1, command.turn, 1e-9);
    }

    @Test public void interpolatesHeadingAcrossPiByTheShortRoute() {
        Frame a = sample(0);
        Frame b = sample(1);
        a.heading = Math.toRadians(179);
        b.heading = Math.toRadians(-179);
        Command command = atPose(new ReplayController(), a, b, 0.5, a.x, a.y, Math.PI);
        assertEquals(Math.PI, Math.abs(command.targetHeading), 1e-9);
        assertEquals(0, command.headingError, 1e-9);
    }

    @Test public void holdsCommandsUntilFieldCentricTransition() {
        Frame a = sample(0);
        Frame b = sample(1);
        a.driveForward = 0.2;
        b.driveForward = 0.9;
        b.fieldCentric = true;
        Command command = atPose(new ReplayController(), a, b, 0.9, a.x, a.y, a.heading);
        assertEquals(0.2, command.forward, 1e-9);
    }

    @Test public void neverInterpolatesAcrossCoordinateReset() {
        Frame a = sample(0);
        Frame b = sample(1);
        b.x = 135;
        b.poseResetId = 1;
        Command command = atPose(new ReplayController(), a, b, 0.99, a.x, a.y, a.heading);
        assertEquals(a.x, command.targetX, 0);
        assertEquals(0, command.forward, 1e-9);
        assertEquals(0, command.strafe, 1e-9);
    }

    @Test public void fieldAndRobotCentricCorrectionsPointToSamePhysicalTarget() {
        Frame frame = sample(0);
        frame.x = 10;
        frame.y = 0;
        frame.heading = Math.PI / 2;
        frame.fieldCentric = true;
        Command field = atPose(new ReplayController(), frame, frame, 0, 0, 0, frame.heading);
        frame.fieldCentric = false;
        Command robot = atPose(new ReplayController(), frame, frame, 0, 0, 0, frame.heading);
        assertTrue(field.forward > 0);
        assertEquals(0, field.strafe, 1e-9);
        assertEquals(0, robot.forward, 1e-9);
        assertEquals(-field.forward, robot.strafe, 1e-9);
    }

    @Test public void boundsCorrectionInBothCoordinateFrames() {
        for (boolean fieldCentric : new boolean[]{false, true}) {
            Frame frame = sample(0);
            frame.x = 10000;
            frame.y = 10000;
            frame.fieldCentric = fieldCentric;
            Command command = atPose(new ReplayController(), frame, frame, 0, 0, 0, 0);
            assertTrue(Math.hypot(command.forward, command.strafe)
                    <= Constants.REPLAY_MAX_TRANSLATION_CORRECTION + 1e-9);
            assertTrue(Math.abs(command.turn) <= Constants.REPLAY_MAX_ROTATION_CORRECTION);
        }
    }

    @Test public void voltageCompensationIsBoundedAndInvalidReadingFallsBack() {
        Frame frame = sample(0);
        frame.driveForward = 0.5;
        ReplayController controller = new ReplayController();
        for (double voltage : new double[]{2, 100, Double.NaN}) {
            Command command = controller.calculate(frame, frame, 0,
                    frame.x, frame.y, frame.heading, 0, 0, 0, voltage, Double.NaN);
            assertTrue(command.forward >= 0.45 && command.forward <= 0.55);
        }
    }

    @Test public void liveAimReplacesRecordedTurnAndOutputRemainsBounded() {
        Frame frame = sample(0);
        frame.driveForward = 1;
        frame.driveStrafe = 1;
        frame.driveTurn = -1;
        Command command = new ReplayController().calculate(frame, frame, 0,
                frame.x, frame.y, frame.heading, 0, 0, 0, 13, 0.7);
        assertEquals(0.7, command.turn, 1e-9);
        assertEquals(1, Math.hypot(command.forward, command.strafe), 1e-9);
    }

    @Test public void feedbackReducesInitialPositionErrorOnMovingTrajectory() {
        ReplayController controller = new ReplayController();
        Frame a = sample(0);
        Frame b = sample(2);
        a.x = 0;
        a.y = 0;
        a.heading = 0;
        b.x = 40;
        b.y = 0;
        b.heading = 0;
        a.driveForward = b.driveForward = 0.4;
        a.velocityX = b.velocityX = 20;
        double x = -5;
        double velocity = 20;
        double dt = 0.02;
        for (int step = 0; step < 100; step++) {
            Command command = controller.calculate(a, b, step * dt,
                    x, 0, 0, velocity, 0, 0, 13, Double.NaN);
            velocity = command.forward * 50;
            x += velocity * dt;
        }
        // Feedforward alone would retain the five-inch initial error.
        assertTrue("Remaining error: " + Math.abs(40 - x), Math.abs(40 - x) < 2);
    }

    @Test public void resetClearsVelocityFilterAfterOdometryReset() {
        Frame frame = sample(0);
        ReplayController controller = new ReplayController();
        controller.calculate(frame, frame, 0, frame.x, frame.y, frame.heading,
                100, -100, 5, 13, Double.NaN);
        controller.reset();
        Command command = atPose(controller, frame, frame, 0, frame.x, frame.y, frame.heading);
        assertEquals(0, command.forward, 1e-9);
        assertEquals(0, command.strafe, 1e-9);
        assertEquals(0, command.turn, 1e-9);
    }

    private Command atPose(ReplayController controller, Frame a, Frame b,
                           double time, double x, double y, double heading) {
        return controller.calculate(a, b, time, x, y, heading, 0, 0, 0, 13, Double.NaN);
    }
}
