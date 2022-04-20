// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.RobotContainer.*;

public class Turret extends SubsystemBase {
  private WPI_TalonSRX rotate = turret_motor;
  /** Creates a new Turret. */
  public Turret() {
    rotate.configFactoryDefault();
    rotate.setNeutralMode(NeutralMode.Brake);
    rotate.setInverted(false);
  }

  public void spinTurret(double velocity) {
    rotate.set(velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
