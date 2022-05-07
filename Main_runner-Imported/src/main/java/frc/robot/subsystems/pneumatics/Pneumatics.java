// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import static frc.robot.Constants.PNEUMATICS_CONSTANTS.*;

public class Pneumatics extends SubsystemBase {
  private final Compressor compressor = RobotContainer.compressor;
  private final DoubleSolenoid solenoid_1 = RobotContainer.solenoid_alpha;
  private final DoubleSolenoid solenoid_2 = RobotContainer.solenoid_beta;
  public XboxController xbox;
  /** Creates a new Pneumatics. */
  public Pneumatics(XboxController controller) {
    xbox = controller;
  }
  
  public Compressor getInstanceCompressor() {
    return compressor;
  }

  public boolean getCompressorState() {
    return RobotContainer.module.getCompressor();
  }

  public boolean getCompressor() {
    return RobotContainer.module.getCompressorNotConnectedFault() || RobotContainer.module.getCompressorNotConnectedFault();
  }
  
  public boolean reverseCompressor() {
    return RobotContainer.module.reserveCompressor();
  }

  public boolean getPressureSwitch() {
    return RobotContainer.module.getPressureSwitch();
  }
  
  public double getCurrent() {
    return compressor.getCurrent();
  }

  public double getPresure() {
    return RobotContainer.module.getPressure(compressor_id);
  }

  @Override
  public void periodic() {
    if (xbox.getLeftBumperPressed()) {
      solenoid_1.set(DoubleSolenoid.Value.kForward);
      solenoid_2.set(DoubleSolenoid.Value.kForward);
    }
    else if (xbox.getRightBumperPressed()) {
      solenoid_1.set(DoubleSolenoid.Value.kReverse);
      solenoid_2.set(DoubleSolenoid.Value.kReverse);
    }

    if (xbox.getAButton()) {
      compressor.enableDigital();
    }
    else if(xbox.getBButton()) {
      compressor.disable();
    }
    
    SmartDashboard.putNumber("Pneumatics pressure", getPresure());
    SmartDashboard.putNumber("Compressor current", getCurrent());
    SmartDashboard.putBoolean("Compressor connection", getCompressor());
    SmartDashboard.putBoolean("Compressor state", getCompressorState());
    // This method will be called once per scheduler run
  }
}
