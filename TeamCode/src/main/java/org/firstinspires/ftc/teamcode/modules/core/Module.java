package org.firstinspires.ftc.teamcode.modules.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The base class for hardware modules
 * Used to interface with hardware elements
 */
public abstract class Module {
    /**
     * The module's parent {@link OpMode}
     */
    public final OpMode parent;

    /**
     * Initializes the module and registers it with the specified OpMode.  This is where references to any hardware
     *  devices used by the module are loaded.
     * @param registrar The OpMode initializing the module
     * @implNote In order to be used in {@link ModuleManager}, all modules should have a public constructor that takes
     *  exactly the same parameters as this one
     * @see ModuleManager#getModule(Class)
     */
    public Module(OpMode registrar) {
        parent = registrar;
    }

    /**
     * Ensures that the module is in a safe state for other modules to operate.
     * Between calling this method and calling any other method on this module that modifies
     * hardware devices, the module is guaranteed to not damage itself or anything else when
     * other modules modify hardware state
     */
    public abstract void ensureSafety();

    /**
     * Checks if this module is connected to the hardware it requires
     * @return false if the module cannot change the state of the hardware, true otherwise
     */
    public abstract boolean isConnected();

    /**
     * Used for logging from modules
     */
    protected Telemetry getTelemetry() {
        return parent.telemetry;
    }

    /**
     * Ran by parent OpMode in its stop() method
     * Cleans up items like background threads
     */
    public void cleanupModule() {}

    /**
     * Logs data about the module to telemetry without changing the state of any hardware devices
     */
    public abstract void log();
}
