package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.FeedbackElement;
import dev.nextftc.control.feedforward.FeedforwardElement;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();
    public Outtake() { }

    private MotorEx motorRight, motorLeft;
    private ServoEx angleServo;

    private final ControlSystem controlSystem = ControlSystem.builder()
            .feedforward(
                    new FeedforwardElement() {
                        @Override
                        public double calculate(@NonNull KineticState reference) {
                            double velocity = motorRight.getVelocity();
                            double error = reference.getVelocity() - velocity;
                            double keep = Math.max(reference.getVelocity() * 0.0005 - 0.3, 0.1);

                            if(reference.getVelocity() > 0) {
                                if (error > 0) {
                                    return 1;
                                } else if (error < -120) {
                                    return keep * 0.6;
                                } else {
                                    return keep;
                                }
                            } else {
                                // Gradually increase active brake as the motor gets slower
                                // (normal brake is less effective and active brake is less damaging)
                                double activeBrake = Math.min(-(53 * velocity / (velocity * velocity + 10000) - 0.065), 0);
                                // Put the formula above in desmos and it makes sense
                                // https://www.desmos.com/calculator/lxm59axfuy

                                if(velocity < 21) {
                                    return 0;
                                } else {
                                    return activeBrake;
                                }
                            }
                        }

                        @Override
                        public void reset() {

                        }
                    }
            )
            .build();

    private double calculateAngleServo(double targetVelocity) {
        if(targetVelocity <= 950) {
            return 0;
        } else if(targetVelocity <= 1080) {
            return 0.00384615 * targetVelocity - 3.65385;
        } else if(targetVelocity <= 1250) {
            return 0.00129412 * targetVelocity - 0.897647;
        } else {
            return 0.72;
        }
    }

    public Command setGoalVelocity(double velocity) {
        return new RunToVelocity(controlSystem, velocity, 40)
                .and(new SetPosition(angleServo, calculateAngleServo(velocity)))
                .requires(this);
    }

    @Override
    public void initialize() {
        motorRight = new MotorEx("shooter0"); // This one has the encoder cable
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.reverse();
        motorLeft = new MotorEx("shooter1");
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.reverse();
        angleServo = new ServoEx("anglePivot");

    }

    @Override
    public void periodic() {
        double power = controlSystem.calculate(motorRight.getState());
        motorRight.setPower(power);
        motorLeft.setPower(power);
    }
}
