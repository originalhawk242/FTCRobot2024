package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Arm;
import org.firstinspires.ftc.teamcode.modules.LinearSlide;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;
import org.firstinspires.ftc.teamcode.modules.core.MotorPowerUpdater;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

/**
 * A base class containing functionality common to all autonomous opmodes
 */
public abstract class AutonomousBase extends LinearOpMode {
    /**
     * The {@link ModuleManager} for this opmode.
     * All modules should be accessed through this to avoid creating multiple instances
     */
    protected final ModuleManager moduleManager = new ModuleManager(this);

    /**
     * Blocks the current thread from progressing until a condition is met.
     * While waiting, updates all loaded {@linkplain MotorPowerUpdater MotorPowerUpdaters}.
     * @param conditionToStop When this returns true, the method will return
     * @throws InterruptedException This opmode has been stopped
     */
    private void waitUntil(BooleanSupplier conditionToStop) throws InterruptedException {
        while (!conditionToStop.getAsBoolean()) {
            if (isStopRequested()) {
                throw new InterruptedException();
            }
            moduleManager.updateMotorPowerLoops();
        }
    }

    /**
     * Blocks the current thread from progressing until the specified time has elapsed.
     * While waiting, this method will update all loaded {@linkplain MotorPowerUpdater MotorPowerUpdaters}
     * @param timeToWait The amount of time this method will wait for, in the specified unit
     * @param timeUnit The unit of time timeToWait is using
     * @throws InterruptedException This opmode has been stopped
     */
    protected final void waitForTime(long timeToWait, TimeUnit timeUnit) throws InterruptedException {
        final ElapsedTime run = new ElapsedTime();
        run.reset();
        waitUntil(() -> run.time(timeUnit) >= timeToWait);
    }

    /**
     * Blocks the current thread from progressing until the provided mechanisms no longer need to update.
     * While waiting, updates all loaded {@linkplain MotorPowerUpdater MotorPowerUpdaters}.
     * @param mechanisms the mechanisms to update along with all other loaded mechanisms
     * @throws InterruptedException This opmode has been stopped
     */
    protected final void waitForMotorUpdaters(MotorPowerUpdater ...mechanisms) throws InterruptedException {
        waitUntil(() -> Arrays.stream(mechanisms).reduce( // true == stop waiting, false == keep waiting
                true, // a && true == a
                (stop, mechanism) -> {
                    if (mechanism.isUpdateNecessary()) {
                        // more updates necessary, keep waiting
                        mechanism.updateMotorPowers();
                        return false;
                    }
                    return stop; // this mechanism has finished updating
                },
                (a, b) -> a && b // returns true only if both a and b are true -> only stops when all mechanisms are done
        ));
    }
}
