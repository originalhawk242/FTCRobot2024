package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.Arm;

@Config
@TeleOp
public class RawMotorDriver extends OpMode {
    private DcMotor motor;
    public static String MOTOR_NAME = Arm.RIGHT_ARM_MOTOR_NAME;
    private String prevMotorName = MOTOR_NAME;
    public static double MOTOR_ENCODER_RESOLUTION = ((((1+(46.0/17))) * (1+(46.0/17))) * (1+(46.0/17)) * 28);

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
    }

    /**
     * User-defined init_loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the init button is pressed and when the play button is pressed (or the
     * OpMode is stopped).
     * <p>
     * This method is optional. By default, this method takes no action.
     */
    @Override
    public void init_loop() {
        monitorKillSwitch();
        logMotorStatus();
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        monitorKillSwitch();
        if (!prevMotorName.equals(MOTOR_NAME)) { // so we can change it on the fly
            resetMotor();
            motor = hardwareMap.tryGet(DcMotor.class, MOTOR_NAME);
            prevMotorName = MOTOR_NAME;
        }
        if (motor == null) {
            return;
        }

        motor.setPower(gamepad1.left_stick_y);
        motor.setTargetPosition((int)(gamepad1.right_stick_y * MOTOR_ENCODER_RESOLUTION));
        if (gamepad1.a) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (gamepad1.b) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (gamepad1.x) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (gamepad1.y) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        logMotorStatus();
    }

    private void resetMotor() {
        if (motor == null) {
            return;
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        motor.setTargetPosition(0);
    }

    private void monitorKillSwitch() {
        if (gamepad1.guide || gamepad1.ps) {
            requestOpModeStop();
        }
    }

    private void logMotorStatus() {
        if (motor == null) {
            telemetry.addLine("No motor found!");
            return;
        }
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Current position", motor.getCurrentPosition());
        telemetry.addData("Target position", motor.getTargetPosition());
        telemetry.addData("Run mode", motor.getMode().toString());
        telemetry.addData("Is busy?", motor.isBusy());
        telemetry.addData("Port number", motor.getPortNumber());
        telemetry.addData("Type", motor.getMotorType());
    }
}
