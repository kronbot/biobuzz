package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Constants.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Loader;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.function.DoubleSupplier;

import dev.nextftc.bindings.Range;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
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
                new SubsystemComponent(
                        Outtake.INSTANCE,
                        Intake.INSTANCE,
                        Loader.INSTANCE,
                        Flap.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final Range rightTrigger = Gamepads.gamepad1().rightTrigger().deadZone(0.1);
    private final Range leftTrigger  = Gamepads.gamepad1().leftTrigger().deadZone(0.1);
    private final Range utilityStickY = Gamepads.gamepad2().rightStickY().deadZone(0.1);

    @NonNull
    private Command gamepadRumble(double rumble1, double rumble2, int duration) {
        return new Command() {
            @Override
            public void start() {
                gamepad1.rumble(rumble1, rumble2, duration);
            }
            @Override
            public boolean isDone() {
                return !gamepad1.isRumbling();
            }
        }.requires(gamepad1);
    }

    @Override
    public void onStartButtonPressed() {
        BindingsComponent.INSTANCE.preWaitForStart();
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                true // robot-centric (false for field centric)
        );
        driverControlled.schedule();

        DoubleSupplier driverLoaderSpeed = () -> rightTrigger.get() - leftTrigger.get();
        DoubleSupplier utilityLoaderSpeed = utilityStickY::get;

        Gamepads.gamepad1().rightBumper()
                .whenBecomesFalse(() -> Flap.INSTANCE.close().invoke())
                .whenFalse(() -> Loader.INSTANCE.activate(utilityLoaderSpeed.getAsDouble())
                        .and(Intake.INSTANCE.activate(utilityLoaderSpeed.getAsDouble()))
                        .invoke())
                .whenBecomesTrue(() -> Flap.INSTANCE.open().invoke())
                .whenTrue(() -> Loader.INSTANCE.activate(driverLoaderSpeed.getAsDouble())
                        .and(Intake.INSTANCE.activate(driverLoaderSpeed.getAsDouble() / 2))
                        .invoke());

        Gamepads.gamepad1().triangle().whenBecomesTrue(
                () -> Outtake.INSTANCE.setGoalVelocity(RANGE_1_VELOCITY)
                        .then(gamepadRumble(0, 1, 100))
                        .invoke()
        );

        Gamepads.gamepad1().square().whenBecomesTrue(
                () -> Outtake.INSTANCE.setGoalVelocity(RANGE_2_VELOCITY)
                        .then(gamepadRumble(0, 1, 100))
                        .invoke()
        );

        Gamepads.gamepad1().cross().whenBecomesTrue(
                () -> Outtake.INSTANCE.setGoalVelocity(RANGE_3_VELOCITY)
                        .then(gamepadRumble(0, 1, 100))
                        .invoke()
        );

        Gamepads.gamepad1().circle().whenBecomesTrue(
                () -> Outtake.INSTANCE.setGoalVelocity(RANGE_4_VELOCITY)
                        .then(gamepadRumble(0, 1, 100))
                        .invoke()
        );

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
                new IfElseCommand(
                        Outtake.INSTANCE::isSpinning,
                        Outtake.INSTANCE.setGoalVelocity(0)
                                .then(gamepadRumble(1, 0, 100)),
                        gamepadRumble(1, 1, 50)
                )
        );
    }

    @Override
    public void onUpdate() {
        BindingsComponent.INSTANCE.preUpdate();
    }

}
