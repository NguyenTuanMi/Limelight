// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Shooter_function;
import static frc.robot.Constants.SHOOTER_CONSTANTS.*;

import java.io.IOException;

import frc.robot.Limelight;
import frc.robot.RobotContainer;

public class ShooterPID extends CommandBase {
  private Shooter shooter = RobotContainer.shooter;
  private PIDController pid = new PIDController(angular_kp, 0, angular_kd);
  private Limelight limelight = RobotContainer.limelight;
  /** Creates a new ShooterPID. */
  public ShooterPID() {
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = limelight.getDistance();
    double setpoint = 0;
    try {
      setpoint = Shooter_function.calculate(distance)[0];
    }
    catch(IOException exception) {
    }
    double velocity = pid.calculate(shooter.getVelocity(), setpoint);
    shooter.shoot(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
