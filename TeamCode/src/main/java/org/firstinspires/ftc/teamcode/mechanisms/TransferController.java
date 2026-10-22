package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Constants;

import dev.nextftc.robot.Mechanism;

public class TransferController implements Mechanism {
    private DcMotorEx transferMotor;
    private boolean on;

    public void initialize(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnOff();
    }

    public void setPower(double power) {
        transferMotor.setPower(power);
        on = Math.abs(power) > 0.0;
    }

    public void turnOn(boolean reversed) {
        setPower(reversed ? -Constants.TRANSFER_POWER : Constants.TRANSFER_POWER);
    }

    public void turnOff() {
        if (transferMotor != null) {
            transferMotor.setPower(0.0);
        }
        on = false;
    }

    public void toggle(boolean reversed) {
        if (on) {
            turnOff();
        } else {
            turnOn(reversed);
        }
    }

    public boolean isOn() {
        return on;
    }

    public void telemetry(Telemetry telemetry) {
        if (transferMotor == null) {
            return;
        }
        telemetry.addLine("--- TRANSFER ---");
        telemetry.addData("Power", "%.2f", transferMotor.getPower());
    }
}
