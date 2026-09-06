package frc.robot.util.motorstatus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXStatus implements MotorStatus {
    private final TalonFX motor;
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temperature;

    public TalonFXStatus(final TalonFX motor) {
        this.motor = motor;
        this.position = motor.getPosition();
        this.velocity = motor.getVelocity();
        this.appliedVoltage = motor.getMotorVoltage();
        this.supplyCurrent = motor.getSupplyCurrent();
        this.statorCurrent = motor.getStatorCurrent();
        this.temperature = motor.getDeviceTemp();
    }

    public void setUpdateFrequency(Frequency frequency) {
        BaseStatusSignal.setUpdateFrequencyForAll(
                frequency, position, velocity, appliedVoltage,
                supplyCurrent, statorCurrent, temperature);
        motor.optimizeBusUtilization();
    }

    public void refresh() {
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVoltage,
                supplyCurrent, statorCurrent, temperature);
    }

    public boolean getConnected() {
        return motor.isConnected();
    }

    public Angle getPosition() {
        return position.getValue();
    }

    public AngularVelocity getVelocity() {
        return velocity.getValue();
    }

    public Voltage getAppliedVoltage() {
        return appliedVoltage.getValue();
    }

    public Current getSupplyCurrent() {
        return supplyCurrent.getValue();
    }

    public Current getStatorCurrent() {
        return statorCurrent.getValue();
    }

    public Temperature getTemperature() {
        return temperature.getValue();
    }
}
