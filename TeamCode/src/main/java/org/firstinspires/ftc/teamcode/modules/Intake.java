package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ConditionalHardwareDevice;
import org.firstinspires.ftc.teamcode.modules.core.Module;
import org.firstinspires.ftc.teamcode.modules.core.ModuleManager;

public class Intake extends Module {

    private ConditionalHardwareDevice<Servo> intakeServo;

    // Name of the servo on the robot configuration
    private final String INTAKE_SERVO_NAME = "Intake Servo";

    // The value to set the servo to in order to "open" and "close" it
    private final double OPEN_INTAKE_SERVO_VALUE = 0;
    private final double CLOSED_INTAKE_SERVO_VALUE = 0.5;

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

        intakeServo = ConditionalHardwareDevice.tryGetHardwareDevice(registrar.hardwareMap, Servo.class, INTAKE_SERVO_NAME);
    }

    public void openIntake() {
        intakeServo.runIfAvailable(i -> {
            i.setPosition(OPEN_INTAKE_SERVO_VALUE);
        });
    }

    public void closeIntake() {
        intakeServo.runIfAvailable(i -> {
            i.setPosition(CLOSED_INTAKE_SERVO_VALUE);
        });
    }

    @Override
    public void log() {
        Telemetry telemetry = getTelemetry();
        if(!intakeServo.isAvailable()) {return;}
        Servo intake = intakeServo.requireDevice();

        telemetry.addData("Current servo target: ", intake.getPosition());
    }
}
