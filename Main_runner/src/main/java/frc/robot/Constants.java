// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class MECA{
        public static final int RFMOTOR = 1;
        public static final int LFMOTOR = 2;
        public static final int RBMOTOR = 3;
        public static final int LBMOTOR = 4;
    }
    public static class controller{
        public static final int Sq = 1;
        public static final int X = 2;
        public static final int O = 3;
        public static final int Tri = 4;
    
        public static final int L1 = 5;
        public static final int R1 = 6;
        public static final int L2 = 7;
        public static final int R2 = 8;

        public static final int L3 = 11;
        public static final int R3 = 12;

        public static final int Share = 9;
        public static final int Option = 10;
        public static final int TouchPad = 13;
        
        public static final int Up = 14;
        public static final int Down = 15;
        public static final int Left = 16;
        public static final int Right = 17;

        public static final int xbox_port = 2;
    }
    public static class Speed{
        public static final double maxSpeed = .8;
        public static final double maxRotation = .5; // use for turret as well 
        public static final double minSpeed = .4;
    }

    public static class FIELD_DATA {
        public static final double target1_x = 10;
        public static final double target1_y = 10;
        public static final double target2_x = 11;
        public static final double target2_y = 11;
        public static final double target3_x = 12;
        public static final double target3_y = 12;
    }

    public static class ROBOT_DATA {
        // the 4 wheels position in the center of robot mass frame
        public static final Translation2d m_frontLeft = new Translation2d(2,3); 
        public static final Translation2d m_frontRight = new Translation2d(2,3);
        public static final Translation2d m_backRight = new Translation2d(2,3);
        public static final Translation2d m_backLeft = new Translation2d(2,3);
    }

    public static class CAMERA_DATA {
        public static final double camera_height = 1;        // compare to the floor
        public static final double target_height = 1;        // compare to the floor
        public static final double camera_pitch = 
        Units.degreesToRadians(23);                          // in field's frame
    }

    public static class SHOOTER_CONSTANTS {
        public static final double angular_kp = 0.5;
        public static final double angular_kd = 0.6;
        public static final double yaw = 10;
        public static final double const_delta = 1; // angle in radian
        public static final double turretMinimumInput = 10;
        public static final double turretMaximumInput = 180;
        public static final double turretVelocityTolerance = .01;
        public static final double turretPositionTolerance = .08;
        public static final int pipeline = 0;
    }
}
