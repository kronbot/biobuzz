package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.bindings.Range;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@TeleOp(name = "MainTeleOp", group = "Test")
public class MainTeleOp extends NextFTCOpMode {
    public MainTeleOp() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                true // robot-centric (false for field centric)
        );
        driverControlled.schedule();

        Range rightStickY = Gamepads.gamepad2().rightStickY().deadZone(0.1);

        rightStickY.asButton(value -> value != 0)
                .whenTrue(() -> Intake.INSTANCE.activate(rightStickY.get()).invoke())
                .whenBecomesFalse(() -> Intake.INSTANCE.deactivate().invoke());

    }

}
