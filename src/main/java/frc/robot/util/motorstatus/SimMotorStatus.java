package frc.robot.util.motorstatus;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimMotorStatus implements MotorStatus {
    private final DCMotorSim motor;

    public SimMotorStatus(final DCMotorSim motor) {
        this.motor = motor;
    }

    public boolean getConnected() {
        return true;
    }

    public Angle getPosition() {
        return motor.getAngularPosition();
    }

    public AngularVelocity getVelocity() {
        return motor.getAngularVelocity();
    }

    public Voltage getAppliedVoltage() {
        return Volts.of(motor.getInputVoltage());
    }

    public Current getSupplyCurrent() {
        return Amps.of(motor.getCurrentDrawAmps());
    }

    public Current getStatorCurrent() {
        return Amps.of(motor.getTorqueNewtonMeters() / motor.getGearbox().KtNMPerAmp);
    }

    public Temperature getTemperature() {
        return Celsius.zero();
    }
}
