// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.RobotContainer.*;

public class Turret extends SubsystemBase {
  private WPI_TalonSRX rotate = turret_motor;
  private Limelight vision = RobotContainer.limelight;
  /** Creates a new Turret. */
  public Turret() {
    rotate.configFactoryDefault();
    rotate.setNeutralMode(NeutralMode.Brake);
    rotate.setInverted(false);
    rotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rotate.setSelectedSensorPosition(0);
  }

  public void spinTurret(double velocity) {
    rotate.set(ControlMode.Velocity, velocity);
  }

  public double getYaw() {
    return rotate.getSelectedSensorPosition();
  }

  public double getSpeed() {
    return rotate.getSelectedSensorVelocity();
  }

  public double getSetpoint() {
    return vision.getX();
  }

  public double getY() {
    return vision.getY();
  }

  public boolean getActivestate() {
    return vision.getActivestate();
  }

  public boolean getTargetstate() {
    return vision.getTargetstate();
  }

  public boolean getSpinstate() {
    return vision.getSpinstate();
  }

  public void reset() {
    vision.resetTurret();
  }

  
  @Override
  public void periodic() {
    vision.setTargetState();
    vision.setActiveState();
    vision.setSpinState(getYaw());
    SmartDashboard.putNumber("Turret angle",  getYaw());
    SmartDashboard.putNumber("Y angle", getY());
    SmartDashboard.putNumber("Turret speed", getSpeed());
    SmartDashboard.putBoolean("Turret active state", vision.getActivestate());
    SmartDashboard.putBoolean("Turret spin state", vision.getSpinstate());
    SmartDashboard.putBoolean("Turret target state", vision.getTargetstate());
    SmartDashboard.putNumber("Turret pipeline", vision.getPipeline());
    // This method will be called once per scheduler run
  }
}
