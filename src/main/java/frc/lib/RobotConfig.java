package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public class RobotConfig {

    public static final double maxAngularVelocity = 2; //TODO: This must be tuned to specific robot
   
    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73); 
    public static final double wheelBase = Units.inchesToMeters(21.73); 
        
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
