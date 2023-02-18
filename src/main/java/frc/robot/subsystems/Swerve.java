package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.RobotConfig;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;   
    
    /* Auto Drive Motor PID Values */
        private static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
        private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
        private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
        private static final double AUTO_TURN_P_CONTROLLER = 10.0;
        private static final double AUTO_TURN_I_CONTROLLER = 0.0;
        private static final double AUTO_TURN_D_CONTROLLER = 0.0;
        public static final double trackWidth = Units.inchesToMeters(21.73); 
        public static final double wheelBase = Units.inchesToMeters(21.73); 

        public double getAutoDriveKP() {
            return AUTO_DRIVE_P_CONTROLLER;
        }

        public double getAutoDriveKI() {
            return AUTO_DRIVE_I_CONTROLLER;
        }

        public double getAutoDriveKD() {
            return AUTO_DRIVE_D_CONTROLLER;
        }

        public double getAutoTurnKP() {
          return AUTO_TURN_P_CONTROLLER;
        }
      
        public double getAutoTurnKI() {
          return AUTO_TURN_I_CONTROLLER;
        }
      
        public double getAutoTurnKD() {
          return AUTO_TURN_D_CONTROLLER;
        }

        public final PIDController autoXController =
            new PIDController(getAutoDriveKP(), getAutoDriveKI(), getAutoDriveKD());
        public final PIDController autoYController =
            new PIDController(getAutoDriveKP(), getAutoDriveKI(), getAutoDriveKD());
        public final PIDController autoThetaController =
            new PIDController(getAutoTurnKP(), getAutoTurnKI(), getAutoTurnKD());
    
    public PIDController getAutoXController() {
        return autoXController;
    }
    
    public PIDController getAutoYController() {
        return autoYController;
    }
    
    public PIDController getAutoThetaController() {
        return autoThetaController;
    }
    public void resetOdometry(Pose2d pathPlannerState) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pathPlannerState);
    }
    private Translation2d centerGravity;
    
    private final SwerveDriveKinematics kinematics =
      RobotConfig.getSwerveDriveKinematics();
    
      private ChassisSpeeds chassisSpeeds;


    public void stop() {
        chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
        setModuleStates(states);
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

    
    

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    
    
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    
    
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public void setSwerveModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, RobotConfig.getRobotMaxAngularVelocity());
    
        for (SwerveModule swerveModule :  mSwerveMods ) {
          swerveModule.setDesiredState(states[swerveModule.getModuleNumber()], false);
        }
      }
    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }


    public void resetOdometry(PathPlannerState initialState) {
    }
    
}