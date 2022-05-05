package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Constants.CAMERA_DATA.*;

public class Limelight {
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

  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public void ledMode(int ledMode) {
    table.getEntry("ledMode").setNumber(ledMode);
  }

  public double getDistance() {
    double alpha = camera_pitch + getY();
    return (target_height - camera_height)/Math.tan(alpha);
  }
}
