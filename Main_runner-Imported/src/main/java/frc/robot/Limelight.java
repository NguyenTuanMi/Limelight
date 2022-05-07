package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Constants.CAMERA_DATA.*;

public class Limelight {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private boolean not_active;
  private boolean has_spin;
  private boolean has_target;
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

  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public double getPipeline() {
    return table.getEntry("getpipe").getDouble(0);
  }

  public void ledMode(int ledMode) {
    table.getEntry("ledMode").setNumber(ledMode);
  }

  public double getDistance(double camera_pitch) {
    double alpha = camera_pitch + getY();
    return (target_height - camera_height)/Math.tan(alpha);
  }

  public boolean getActivestate() {
    return !not_active;
  }

  public boolean getTargetstate() {
    return has_target;
  }

  public boolean getSpinstate() {
    return has_spin;
  }

  public void resetTurret() {
    has_target = false;
    has_spin = false;
    not_active = true;
  }

  public void setTargetState() {
    if (checkTarget() > 0) {
      has_target = true;
    }
    else {
      has_target = false;
    }
  }

  public void setActiveState() {
    if(RobotContainer.xbox.getAButton()) {
      not_active = false;
    }
    else {
      not_active = true;
    }
  }

  public void setSpinState(double setpoint) {
    if (Math.abs(setpoint) > 10f) {
      has_spin = true;
    }
    else {
      has_spin = false;
    }
  }
}
