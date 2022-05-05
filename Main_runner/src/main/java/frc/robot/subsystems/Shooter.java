// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX talonSRX = RobotContainer.shooter_motor;
  /** Creates a new Shooter. */
  public Shooter() {
    talonSRX.setNeutralMode(NeutralMode.Brake);
    talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    talonSRX.setSelectedSensorPosition(0);
  }

  public void shoot(double velocity) { // in position change per 100ms
    talonSRX.set(ControlMode.Velocity, velocity);
  }

  public double getVelocity() {
    return talonSRX.getSelectedSensorVelocity();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
