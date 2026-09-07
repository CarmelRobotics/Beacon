package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs;
    private final int index;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;
    private SwerveModulePosition[] odometryPositions;

    public Module(
            ModuleIO io,
            int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.io = io;
        this.inputs = new ModuleIOInputsAutoLogged();
        this.index = index;
        this.constants = constants;
        this.odometryPositions = new SwerveModulePosition[] {};
    }

    public void runSetpoint(SwerveModuleState state) {
        state.optimize(getSteerAngle());
        state.cosineScale(inputs.steerPosition);

        io.setVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / constants.WheelRadius));
        io.setSteer(state.angle);
    }

    public void runCharacterization(Voltage voltage) {
        io.setDriveVoltage(voltage);
        io.setSteer(Rotation2d.kZero);
    }

    public void stop() {
        io.setDriveVoltage(Volts.zero());
        io.setSteerVoltage(Volts.zero());
    }

    public Distance getDriveDistance() {
        return Meters.of(inputs.drivePosition.in(Radians) * constants.WheelRadius);
    }

    public Rotation2d getSteerAngle() {
        return inputs.steerPosition;
    }

    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(inputs.driveVelocity.in(RadiansPerSecond) * constants.WheelRadius);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), getSteerAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }
    
    public Angle getWheelRadiusCharacterizationPosition() {
        return inputs.drivePosition;
    }

    public AngularVelocity getFFCharacterizationVelocity() {
        return inputs.driveVelocity;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        final int sampleCount = inputs.odometryTimestamps.length;
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            final Distance position = Meters.of(inputs.odometryDrivePositionsRads[i] * constants.WheelRadius);
            final Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(position, angle);
        }
    }
}
