// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.PhoenixOdometryThread;

import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeon2() {
        this.pigeon = new Pigeon2(DriveConstants.kPigeonId, DriveConstants.kCANBus);

        this.yaw = pigeon.getYaw();
        this.yawVelocity = pigeon.getAngularVelocityZWorld();

        this.yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());

        pigeon.getConfigurator().apply(DriveConstants.Pigeon2Config);
        pigeon.getConfigurator().setYaw(0.0);
        
        yaw.setUpdateFrequency(DriveConstants.kOdometryFrequency);
        yawVelocity.setUpdateFrequency(DriveConstants.kStatusFrequency);
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocity = yawVelocity.getValue();

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
