// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Constants.CAMERA_DATA.*;

public class Limelight extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  /** Creates a new Limelight. */
  public Limelight() {}

  public double checkTarget() {
    return table.getEntry("tv").getDouble(0);
  }

  public double getX() {
    return table.getEntry("tx").getDouble(0);
  }

  public double getY() {
    return table.getEntry("ty").getDouble(0);
  }

  public double getDistance() {
    double alpha = camera_pitch + getY();
    return (target_height - camera_height)/Math.tan(alpha);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
