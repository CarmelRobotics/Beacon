// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.motorstatus.SimMotorStatus;

public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveMotor;
    private final DCMotorSim steerMotor;

    private final SimMotorStatus driveMotorStatus;
    private final SimMotorStatus steerMotorStatus;

    private boolean driveClosedLoop;
    private boolean steerClosedLoop;
    private PIDController driveController;
    private PIDController steerController;
    private double driveFeedForwardVolts;
    private double driveAppliedVolts;
    private double steerAppliedVolts;

    public ModuleIOSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.driveMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DriveConstants.Simulation.kDriveMotor,
                        constants.DriveInertia,
                        constants.DriveMotorGearRatio),
                DriveConstants.Simulation.kDriveMotor);
        this.steerMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DriveConstants.Simulation.kSteerMotor,
                        constants.SteerInertia,
                        constants.SteerMotorGearRatio),
                DriveConstants.Simulation.kSteerMotor);

        this.driveMotorStatus = new SimMotorStatus(driveMotor);
        this.steerMotorStatus = new SimMotorStatus(steerMotor);

        this.driveClosedLoop = false;
        this.steerClosedLoop = false;
        this.driveController = new PIDController(DriveConstants.Simulation.DriveGains.kP, 0,
                DriveConstants.Simulation.DriveGains.kD);
        this.steerController = new PIDController(DriveConstants.Simulation.SteerGains.kP, 0,
                DriveConstants.Simulation.SteerGains.kD);
        this.driveFeedForwardVolts = 0.0;
        this.driveAppliedVolts = 0.0;
        this.steerAppliedVolts = 0.0;

        steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        driveClosedLoop = false;
        driveAppliedVolts = voltage.in(Volts);
    }

    @Override
    public void setSteerVoltage(Voltage voltage) {
        steerClosedLoop = false;
        steerAppliedVolts = voltage.in(Volts);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        final double velocityRadPerSec = velocity.in(RadiansPerSecond);
        driveClosedLoop = true;
        driveFeedForwardVolts = 
                DriveConstants.Simulation.DriveGains.kS * Math.signum(velocityRadPerSec)
                + DriveConstants.Simulation.DriveGains.kV * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setSteer(Rotation2d rotation) {
        steerClosedLoop = true;
        steerController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void updateInputs(ModuleIOInputsAutoLogged inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts = driveFeedForwardVolts
                    + driveController.calculate(driveMotor.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (steerClosedLoop) {
            steerAppliedVolts = steerController.calculate(steerMotor.getAngularPositionRad());
        } else {
            steerController.reset();
        }

        driveMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        steerMotor.setInputVoltage(MathUtil.clamp(steerAppliedVolts, -12.0, 12.0));
        driveMotor.update(DriveConstants.kDeltaTime.in(Seconds));
        steerMotor.update(DriveConstants.kDeltaTime.in(Seconds));

        inputs.driveConnected = driveMotorStatus.getConnected();
        inputs.drivePosition = driveMotorStatus.getPosition();
        inputs.driveVelocity = driveMotorStatus.getVelocity();
        inputs.driveAppliedVoltage = driveMotorStatus.getAppliedVoltage();
        inputs.driveSupplyCurrent = driveMotorStatus.getSupplyCurrent();
        inputs.driveStatorCurrent = driveMotorStatus.getStatorCurrent();
        inputs.driveTemperature = driveMotorStatus.getTemperature();

        inputs.steerConnected = steerMotorStatus.getConnected();
        inputs.steerPosition = new Rotation2d(steerMotorStatus.getPosition());
        inputs.steerVelocity = steerMotorStatus.getVelocity();
        inputs.steerAppliedVoltage = steerMotorStatus.getAppliedVoltage();
        inputs.steerSupplyCurrent = steerMotorStatus.getSupplyCurrent();
        inputs.steerStatorCurrent = steerMotorStatus.getStatorCurrent();
        inputs.steerTemperature = steerMotorStatus.getTemperature();

        inputs.encoderConnected = true;
        inputs.encoderMagnetIsOk = true;
        inputs.encoderPosition = new Rotation2d(steerMotorStatus.getPosition());

        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRads = new double[] {inputs.drivePosition.in(Radians)};
        inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerPosition};
    }
}
