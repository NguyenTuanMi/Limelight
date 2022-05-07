// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.hood.Hood;
import frc.robot.Limelight;
import frc.robot.commands.TurretPID;
import frc.robot.commands.ShooterPID;

import static frc.robot.Constants.MECA.*;
import static frc.robot.Constants.HOOD_CONSTANTS.*;
import static frc.robot.Constants.SHOOTER_CONSTANTS.*;
import static frc.robot.Constants.PNEUMATICS_CONSTANTS.*;
import static frc.robot.Constants.TURRET_CONSTANTS.*;
import static frc.robot.Constants.controller.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static WPI_TalonSRX turret_motor = new WPI_TalonSRX(TURRET_MOTOR);
  public static WPI_TalonSRX shooter_motor = new WPI_TalonSRX(SHOOTER_MOTOR);
  public static WPI_TalonSRX hood_motor = new WPI_TalonSRX(HOOD_MOTOR);
  public static XboxController xbox = new XboxController(xbox_port);
  
  public static Turret turret = new Turret();
  public static Shooter shooter = new Shooter();
  public static Hood hood = new Hood();
  public static Limelight limelight = new Limelight();
  public static PneumaticsControlModule module = new PneumaticsControlModule();
  public static Compressor compressor = new Compressor(compressor_id, PneumaticsModuleType.CTREPCM);
  public static DoubleSolenoid solenoid_alpha = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, solenoid_alpha_forward_channel, solenoid_alpha_reverse_channel);
  public static DoubleSolenoid solenoid_beta = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, solenoid_beta_forward_channel, solenoid_beta_reverse_channel);

  public static PIDController hood_pid = new PIDController(hood_kp, 0, hood_kd);
  public static PIDController shooter_pid = new PIDController(angular_kp, 0, angular_kd);
  public static PIDController turret_pid = new PIDController(turret_kp, 0, turret_kd);
  
  public static Command turretpid = new TurretPID();
  public static Command shooterpid = new ShooterPID();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
