// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Turret;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import static frc.robot.Constants.SHOOTER_CONSTANTS.*;
import static frc.robot.Constants.Speed.*;

public class TurretPID extends CommandBase {
  private Turret turret = RobotContainer.turret;
  private Limelight calculator = RobotContainer.limelight;
  private PIDController controller;
  private boolean not_active;
  private boolean has_target;
  private boolean has_spin;
  /** Creates a new Turretpid. */
  public TurretPID() {
    controller = new PIDController(.02, 0, 0);
    controller.setTolerance(turretPositionTolerance, turretVelocityTolerance);
    controller.setIntegratorRange(turretMaximumInput, turretMaximumInput);
    has_target = false;
    has_spin = false;
    not_active = true;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void resetTurret() {
    has_target = false;
    has_spin = false;
    not_active = true;
  }

  public void uploadState() {
    SmartDashboard.putBoolean("Spin state", has_spin);
    SmartDashboard.putBoolean("Target state", has_target);
    SmartDashboard.putBoolean("Unactive state", not_active);
  }

  public void targetState() {
    if (calculator.checkTarget() > 0) {
      has_target = true;
    }
    else {
      has_target = false;
    }
  }

  public void activeState() {
    if(RobotContainer.xbox.getAButton()) {
      not_active = false;
    }
    else {
      not_active = true;
    }
  }

  public void spinState(double setpoint) {
    if (Math.abs(setpoint) > 10f) {
      has_spin = true;
    }
    else {
      has_spin = false;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity;
    double distance = calculator.getDistance();
    targetState();
    activeState();
    if (!not_active) {
      calculator.setPipeline(pipeline);
      if (!has_target) {
        velocity = 0.1;
      }
      else {
        double setpoint = calculator.getX();
        distance = calculator.getDistance();
        spinState(setpoint);
        if (has_spin) {
          velocity = Math.min(controller.calculate(0, -setpoint), maxRotation);
        }
        else {
          velocity = 0;
        }
      turret.spinTurret(velocity);
      }
    }
    uploadState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    calculator.ledMode(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return not_active;
  }
}
