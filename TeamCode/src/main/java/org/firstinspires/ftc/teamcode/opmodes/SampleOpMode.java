package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * A sample OP Mode to be copied when creating a new OP Mode.
 * The sample OP Mode should be edited to keep it up to date with the rest of the code.
 * <br>B)
 */

public class SampleOpMode extends OpMode {
    double lastRuntime = 0; // Used to calculate the delta in init_loop() and loop()

    public void init() {

    }

    public void init_loop() {
        double now = getRuntime();
        double delta = now - lastRuntime;
        lastRuntime = now;



        updateTelemetry(telemetry);
    }

    public void start() {

    }

    public void loop() {
        double now = getRuntime();
        double delta = now - lastRuntime;
        lastRuntime = now;



        updateTelemetry(telemetry);
    }

    public void stop() {

    }
}
