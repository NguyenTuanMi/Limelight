// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static frc.robot.RobotContainer.hood_pid;
import static frc.robot.Constants.HOOD_CONSTANTS.*;
import static frc.robot.RobotContainer.hood_motor;;

public class Hood extends SubsystemBase {
  private WPI_TalonSRX hood = hood_motor;
  private Mechanism2d mech = new Mechanism2d(2.0, 3.0);
  private MechanismRoot2d robot = mech.getRoot("Robot", 1.0, 0.0);
  private MechanismLigament2d tower2d;
  private MechanismLigament2d hood2d;
  
    
  /** Creates a new Hood. */
  public Hood() {
    hood_pid.reset();
    hood.configFactoryDefault();
    hood.setNeutralMode(NeutralMode.Brake);
    hood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    hood.setSelectedSensorPosition(0);
    hood_pid.setIntegratorRange(0, 180);
    hood_pid.setTolerance(hoodPositionTolerence, hoodVelocityTolerence);
    hood_pid.reset();
    tower2d = robot.append(new MechanismLigament2d("Tower", 1, 90, 20, new Color8Bit(Color.kBlue)));
  }

  public void setInverted(boolean isInverted) {
    hood.setInverted(isInverted);
  }

  public void manipulate(double velocity) {
    hood.set(velocity);
  }

  public double getAngle() {
    return hood.getSelectedSensorPosition();
  }

  public double getVelocity() {
    return hood.getActiveTrajectoryVelocity();
  }

  public void moveToAngle(double angle) {
    angle = MathUtil.clamp(angle, min_angle, max_angle);
    double velocity = hood_pid.calculate(getAngle(), angle);
    velocity = MathUtil.clamp(velocity, min_velocity, max_velocity);
    manipulate(velocity);
  }

  public boolean reachVelocity(double setpoint) {
    if (Math.abs(getVelocity()-setpoint) < hoodVelocityTolerence) {
      return true;
    }
    else {
      return false;
    }
  }

  public void moveToBottom() {
    moveToAngle(-getAngle());
  }

  @Override
  public void periodic() {
    hood2d = tower2d.append(new MechanismLigament2d("Hood", 0.5, 45, 10, new Color8Bit(Color.kDarkRed)));
    SmartDashboard.putNumber("Hood angle", getAngle());
    SmartDashboard.putNumber("Hood speed", getVelocity());
    SmartDashboard.putData("Hood", mech);
    // This method will be called once per scheduler run
  }
}
