// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Limelight;
import frc.robot.RobotContainer;
import static frc.robot.Constants.SHOOTER_CONSTANTS.*;

public class TurretPID extends CommandBase {
  private Turret turret = RobotContainer.turret;
  private Limelight limelight = new Limelight();
  private PIDController controller;
  private boolean not_active = false;
  /** Creates a new Turretpid. */
  public TurretPID() {
    controller = new PIDController(.02, 0, 0);
    controller.setTolerance(turretPositionTolerance, turretVelocityTolerance);
    controller.setIntegratorRange(turretMaximumInput, turretMaximumInput);
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocity;
    //double distance;
    if (RobotContainer.xbox.getAButton()) {
      if (limelight.checkTarget() == 0) {
        velocity = 0.1;
      }
      else {
        double setpoint = limelight.getX();
        velocity = controller.calculate(0, -setpoint);
      //distance = limelight.getDistance();
      }
      turret.spinTurret(velocity);
    }
    else {
      not_active = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return not_active;
  }
}
