package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;

/**
 * This module controls the intake, which collects specimens and samples
 */
public class Intake extends Module {

    private final ConditionalHardwareDevice<CRServo> intakeServo;

    private final ConditionalHardwareDevice<Servo> wristServo;

    // Names of the servos on the robot configuration
    public static final String INTAKE_SERVO_NAME = "Intake Servo";

    public static final String WRIST_SERVO_NAME = "Wrist Servo";

    private final double SERVO_SPEED = 0.5;

    /**
     * Initializes the module and registers it with the specified OpMode.  This is where references to any hardware
     * devices used by the module are loaded.
     *
     * @param registrar The OpMode initializing the module
     * @implNote In order to be used in {@link ModuleManager}, all modules should have a public constructor that takes
     * exactly the same parameters as this one
     * @see ModuleManager#getModule(Class)
     */
    public Intake(OpMode registrar) {
        super(registrar);

        intakeServo = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, CRServo.class, INTAKE_SERVO_NAME);
        wristServo = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, Servo.class, WRIST_SERVO_NAME);
    }

    /**
     * Sets the intake to eject whatever it is holding.
     * The intake will stay in this mode until either {@link #grab()} or {@link #settle()} is called.
     */
    public void eject() {
        intakeServo.runIfAvailable(i -> {
            i.setPower(SERVO_SPEED);
        });
    }

    /**
     * Sets the intake to collect whatever is in front of it.
     * The intake will stay in this mode until either {@link #eject()} or {@link #settle()} is called.
     */
    public void grab() {
        intakeServo.runIfAvailable(i -> {
            i.setPower(-SERVO_SPEED);
        });
    }

    /**
     * Sets the intake to hold the object it has
     */
    public void settle() {
        intakeServo.runIfAvailable(i -> {
            i.setPower(0);
        });
    }

    /**
     * Toggles the rotation of the wrist
     */
    public void turn() {
        wristServo.runIfAvailable(w -> {
            if (w.getPosition() == 0.0) {
                w.setPosition(0.5);
            } else if (w.getPosition() == 0.5) {
                w.setPosition(0.0);
            }
        });
    }

    /**
     * Sets the wrist to its default rotation
     */
    public void holdWristRotation() {
        wristServo.runIfAvailable(w -> w.setPosition(0.45));
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if(!intakeServo.isAvailable()) {return;}
        CRServo intake = intakeServo.requireDevice();

        telemetry.addData("Current Intake Servo Power: ", intake.getPower());
    }
}
