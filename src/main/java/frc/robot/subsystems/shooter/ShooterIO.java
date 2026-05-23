package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {

    }

    public enum ShooterState {
        STOW,
        OFF,
        PASS,
        SCORE_PRESET,
        SCORE;
    }

    public void zeroHere();
    public void updatePivotPID(double kA, double kD, double kI, double kP, double kS, double kV);
    public default void updatePivotMotionMagic(double kCruiseVelocity, double kAcceleration, double kJerk) {}
    public void updateRollerPID(double kA, double kD, double kI, double kP, double kS, double kV);
    public void updateKickerPID(double kA, double kD, double kI, double kP, double kS, double kV);
    public void updateInputs(ShooterIOInputsAutoLogged inputs);
}
