package org.firstinspires.ftc.teamcode.modules.core;

import java.util.function.BooleanSupplier;

public interface MotorPowerUpdater {
    /**
     * Called in a loop; updates the power levels of all motors attached to this instance
     */
    void updateMotorPowers();

    /**
     * Checks if the motors need to be updated
     * @return true if {@link #updateMotorPowers()} needs to be called, false otherwise
     */
    boolean isUpdateNecessary();

    /**
     * updates all provided mechanisms until no more updates are necessary
     * @param mechanisms the mechanisms to update
     * @param killSwitch An external 'kill switch' -- if this returns true at any point,
     *                   this method will throw an exception
     * @throws InterruptedException the kill switch has been triggered
     */
    static void updateWhileNecessary(BooleanSupplier killSwitch, MotorPowerUpdater ...mechanisms) throws InterruptedException {
        boolean continueLoop;
        do {
            if (killSwitch.getAsBoolean()) {
                throw new InterruptedException();
            }

            continueLoop = false;
            for (MotorPowerUpdater mechanism : mechanisms) {
                // only update mechanisms that need updating
                if (mechanism.isUpdateNecessary()) {
                    continueLoop = true;
                    mechanism.updateMotorPowers();
                }
            }
        } while(continueLoop);
    }
}
