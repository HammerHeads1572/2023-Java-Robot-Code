package frc.lib;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public class RobotConfig {

    private static RobotConfig robotConfig;
    
    public static RobotConfig getInstance() {
        return robotConfig;
      }

      protected RobotConfig() {
        RobotConfig.robotConfig = this;
      }

     /* Drivetrain Constants */
     public static final double trackWidth = Units.inchesToMeters(21.73); 
     public static final double wheelBase = Units.inchesToMeters(21.73); 
     
 
     /* Swerve Kinematics 
      * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public SwerveDriveKinematics getSwerveDriveKinematics() { 
         return new SwerveDriveKinematics(
             new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
             new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
             new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
     }
 
}
