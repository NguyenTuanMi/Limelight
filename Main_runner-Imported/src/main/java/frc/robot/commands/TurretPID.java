// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.Turret;

import static frc.robot.Constants.SHOOTER_CONSTANTS.*;
import static frc.robot.Constants.Speed.*;

public class TurretPID extends CommandBase {
  private Turret turret = RobotContainer.turret;
  private PIDController controller = RobotContainer.turret_pid;
  /** Creates a new Turretpid. */
  public TurretPID() {
    controller.setTolerance(turretPositionTolerance, turretVelocityTolerance);
    controller.setIntegratorRange(turretMaximumInput, turretMaximumInput);
    controller.reset();
    turret.reset();
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity = 0;
    //double distance = calculator.getDistance();
    if (turret.getActivestate()) {
      if (!turret.getTargetstate()) {
        velocity = 0.1;
      }
      else {
        double setpoint = turret.getSetpoint();
        //distance = calculator.getDistance();
        if (turret.getSpinstate()) {
          velocity = Math.min(controller.calculate(0, -setpoint), maxRotation);
        }
        else {
          velocity = 0;
        }
      turret.spinTurret(velocity);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !turret.getActivestate();
  }
}
