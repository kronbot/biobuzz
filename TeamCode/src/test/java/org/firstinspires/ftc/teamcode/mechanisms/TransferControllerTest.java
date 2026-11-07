package org.firstinspires.ftc.teamcode.mechanisms;

import static org.junit.Assert.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.junit.After;
import org.junit.Test;

import java.lang.reflect.Proxy;

public class TransferControllerTest {
    private final boolean previousEnabled = Constants.TRANSFER_ENABLED;

    @After public void restoreConfiguration() {
        Constants.TRANSFER_ENABLED = previousEnabled;
    }

    @Test public void uninstalledTransferDoesNotAccessHardwareMapOrAcceptPower() {
        Constants.TRANSFER_ENABLED = false;
        TransferController transfer = new TransferController();
        transfer.initialize(null); // Any hardware access would throw.
        transfer.turnOn(false);
        assertFalse(transfer.isOn());
        transfer.turnOn(true);
        transfer.toggle(false);
        transfer.setPower(1);
        transfer.turnOff();
        assertFalse(transfer.isAvailable());
        assertFalse(transfer.isOn());
        assertEquals(0, transfer.getPower(), 0);
    }

    @Test public void enabledMissingTransferBlocksInitializationWithOriginalError() {
        Constants.TRANSFER_ENABLED = true;
        TransferController transfer = new TransferController();
        IllegalArgumentException missingMotor = new IllegalArgumentException(
                "Unable to find a hardware device with name \"transfer\" and type DcMotorEx");
        HardwareMap emptyMap = new HardwareMap(null, null) {
            @Override public <T> T get(Class<? extends T> type, String name) {
                assertEquals(DcMotorEx.class, type);
                assertEquals("transfer", name);
                throw missingMotor;
            }
        };
        assertSame(missingMotor, assertThrows(IllegalArgumentException.class,
                () -> transfer.initialize(emptyMap)));
        assertFalse(transfer.isAvailable());
        assertEquals(0, transfer.getPower(), 0);
    }

    @Test public void installedTransferStillSupportsForwardReverseAndStop() {
        Constants.TRANSFER_ENABLED = true;
        double[] power = {0};
        DcMotorEx motor = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class},
                (proxy, method, args) -> {
                    if ("setPower".equals(method.getName())) power[0] = (Double) args[0];
                    if ("getPower".equals(method.getName())) return power[0];
                    return null;
                });
        HardwareMap map = new HardwareMap(null, null) {
            @Override public <T> T get(Class<? extends T> type, String name) {
                assertEquals("transfer", name);
                return type.cast(motor);
            }
        };
        TransferController transfer = new TransferController();
        transfer.initialize(map);
        assertTrue(transfer.isAvailable());
        assertFalse(transfer.isOn());
        transfer.turnOn(false);
        assertEquals(Constants.TRANSFER_POWER, transfer.getPower(), 0);
        assertTrue(transfer.isOn());
        transfer.turnOn(true);
        assertEquals(-Constants.TRANSFER_POWER, transfer.getPower(), 0);
        transfer.turnOff();
        assertEquals(0, transfer.getPower(), 0);
        assertFalse(transfer.isOn());
    }
}
