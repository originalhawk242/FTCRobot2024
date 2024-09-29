package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;
import java.util.function.Consumer;

/**
 * Represents a hardware device that may or may not be accessible.
 * Used to ensure functionality of the robot during testing, when not everything is always connected to the Control Hub.
 */
public final class ConditionalHardwareDevice<T extends HardwareDevice> {
    /**
     * The name used to get the hardware device
     */
    private final String name;

    /**
     * The hardware device
     */
    private final T device;

    /**
     * Is the device accessible?
     */
    private final boolean available;

    /**
     * Constructs this class with the specified state
     * @param device The hardware device
     * @param name The name used to retrieve the device from the hardware map
     */
    private ConditionalHardwareDevice(T device, String name) {
        this.name = name;
        this.device = device;
        available = device != null;
    }

    /**
     * Attempts to get a hardware device
     * @param hardwareMap The {@link HardwareMap} object to query
     * @param deviceClass The class of the hardware device
     * @param deviceName The name of the hardware device
     * @return A {@link ConditionalHardwareDevice} object with the retrieved hardware device, if it can be retrieved
     * @param <U> The type of the hardware device
     */
    public static <U extends HardwareDevice> ConditionalHardwareDevice<U> tryGetHardwareDevice(HardwareMap hardwareMap, Class<? extends U> deviceClass, String deviceName) {
        try {
            final U device;
            if (deviceClass.equals(PIDFDcMotor.class)) {
                // Since deviceClass is the same as U.class, we know that U is of type PIDFDcMotor,
                // so a cast from PIDFDcMotor to U will be guaranteed to succeed
                device = (U) PIDFDcMotor.get(hardwareMap, deviceName);
            }
            else {
                device = hardwareMap.get(deviceClass, deviceName);
            }
            // If no name is registered, the hardware map call will fail.
            // However, the hardware map doesn't care about whether or not the device is actually
            // plugged in, so it will happily return a hardware device that can't actually do
            // anything.  We have to do something that fails if no device is connected to ensure
            // that the hardware map isn't lying to us.
            device.getConnectionInfo(); // hopefully this fails and doesn't just produce garbage data
            return new ConditionalHardwareDevice<>(device, deviceName);
        }
        catch (Throwable th) {
            if (th.getClass() != IllegalArgumentException.class) {
                throw th;
            }
            else {
                return new ConditionalHardwareDevice<>(null, deviceName);
            }
        }
    }

    /**
     * Gets the name used to retrieve this device from the hardware map
     * @return The device name
     */
    public String getName() {
        return name;
    }

    /**
     * Is the hardware device accessible?
     * @return True if the hardware device exists, otherwise false
     */
    public boolean isAvailable() {
        return available;
    }

    /**
     * Gets the hardware device
     * @return The hardware device
     * @throws NullPointerException The hardware device is inaccessible
     */
    public T requireDevice() {
        return Objects.requireNonNull(device);
    }

    /**
     * Runs provided code only if the hardware device is accessible
     * @param it The code to run
     */
    public void runIfAvailable(Consumer<T> it) {
        runIfAvailable(it, () -> {}); // do nothing if the device is unavailable
    }

    /**
     * Runs provided code only if the hardware device is accessible
     * @param runnable The code to run
     * @param onUnavailable A function to run if the device is unavailable
     */
    public void runIfAvailable(Consumer<T> runnable, Runnable onUnavailable) {
        if (isAvailable()) {
            runnable.accept(device);
        }
        else {
            onUnavailable.run();
        }
    }

    @Override
    public boolean equals(@Nullable Object obj) {
        if (super.equals(obj)) {
            return true; // we are comparing the same memory address
        }
        if (obj == null) {
            return false;
        }
        if (!(obj instanceof ConditionalHardwareDevice)) {
            return false;
        }
        ConditionalHardwareDevice<?> otherDevice = ((ConditionalHardwareDevice<?>) obj);
        if (!otherDevice.name.equals(name)) {
            return false;
        }
        if (!isAvailable()) {
            return !otherDevice.isAvailable();
        }
        return requireDevice().equals(otherDevice.requireDevice());
    }

    @Override
    public int hashCode() {
        if (!isAvailable()) {
            return name.hashCode();
        }
        return requireDevice().hashCode();
    }
}
