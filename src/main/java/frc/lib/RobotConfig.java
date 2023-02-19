package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public class RobotConfig {

    private static RobotConfig robotConfig;
    public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot
   
    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73); 
    public static final double wheelBase = Units.inchesToMeters(21.73); 
         
    public RobotConfig() {
      RobotConfig.robotConfig = this;
    }
     /* Swerve Kinematics 
      * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static SwerveDriveKinematics getSwerveDriveKinematics() { 
         return new SwerveDriveKinematics(
             new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
             new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
     }
     
     public static double getRobotMaxAngularVelocity() {
      return maxAngularVelocity;
  }

    
 
}
