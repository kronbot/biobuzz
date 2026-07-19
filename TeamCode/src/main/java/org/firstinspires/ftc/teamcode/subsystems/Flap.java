package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Flap implements Subsystem {
    public static final Flap INSTANCE = new Flap();
    public Flap() { }

    private ServoEx servo;

    public Command open() {
        return new SetPosition(servo, FLAP_OPEN).requires(this);
    }
    public Command close() {
        return new SetPosition(servo, FLAP_CLOSED).requires(this);
    }

    @Override
    public void initialize() {
        servo = new ServoEx("flapsServo");
    }
}
