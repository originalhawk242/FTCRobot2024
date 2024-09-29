package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class PIDFDcMotor extends PIDFController implements DcMotorEx {
    private final DcMotorEx internalMotor;
    public PIDFDcMotor(DcMotorEx motor, double kp, double ki, double kd, double kf) {
        super(kp, ki, kd, kf);
        internalMotor = motor;
    }
    public PIDFDcMotor(DcMotorEx motor) {
        this(motor, 0, 0, 0, 0);
    }

    public static PIDFDcMotor get(HardwareMap hardwareMap, String deviceName, double kp, double ki, double kd, double kf) {
        return new PIDFDcMotor(hardwareMap.get(DcMotorEx.class, deviceName), kp, ki, kd, kf);
    }
    public static PIDFDcMotor get(HardwareMap hardwareMap, String deviceName) {
        return new PIDFDcMotor(hardwareMap.get(DcMotorEx.class, deviceName));
    }

    public void applyMotorPIDF() {
        internalMotor.setPower(calculate(internalMotor.getCurrentPosition()));
    }

    @Override
    public boolean atSetPoint() {
        applyMotorPIDF();
        return super.atSetPoint();
    }


    @Override
    public void setMotorEnable() {
        internalMotor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        internalMotor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return internalMotor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        internalMotor.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        internalMotor.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return internalMotor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return internalMotor.getVelocity();
    }

    @Deprecated
    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        internalMotor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        internalMotor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        internalMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        internalMotor.setPositionPIDFCoefficients(p);
    }

    @Deprecated
    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return internalMotor.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return internalMotor.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        internalMotor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return internalMotor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return internalMotor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return internalMotor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        internalMotor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return internalMotor.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return internalMotor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        internalMotor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return internalMotor.getController();
    }

    @Override
    public int getPortNumber() {
        return internalMotor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        internalMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return null;
    }

    @Deprecated
    @Override
    public void setPowerFloat() {
        internalMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return internalMotor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        internalMotor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return internalMotor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return internalMotor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return internalMotor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        internalMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return internalMotor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        internalMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return internalMotor.getDirection();
    }

    @Override
    public void setPower(double power) {
        internalMotor.setPower(power);
    }

    @Override
    public double getPower() {
        return internalMotor.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return internalMotor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return internalMotor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return internalMotor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return internalMotor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        internalMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        internalMotor.close();
    }
}
