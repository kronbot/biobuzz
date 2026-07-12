package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    private MotorEx motor;

    public Command activate(double power) {
        return new SetPower(motor, power).requires(this);
    }

    public Command deactivate() {
        return new SetPower(motor, 0).requires(this);
    }

    @Override
    public void initialize() {
        motor = new MotorEx("intakeMotor");
    }
}
