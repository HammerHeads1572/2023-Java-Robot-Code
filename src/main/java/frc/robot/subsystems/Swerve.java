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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.lang.Math;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;    
    public final SwerveModule flModule;
    public final SwerveModule frModule;
    public final SwerveModule blModule;
    public final SwerveModule brModule;

    public static double changeSpeed;
    public static double FOB;
    public static double RotationCount;
   
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
    public void resetOdometry(Pose2d pose2d) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose2d);
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

    
    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID,"Canivore");
        gyro.configFactoryDefault();
        zeroGyro();
        RotationCount = 0;
        
                            
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        brModule = mSwerveMods[0];
        flModule = mSwerveMods[1];
        blModule = mSwerveMods[2]; 
        frModule = mSwerveMods[3];

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.changeSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        
    }    
    
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.changeSpeed);
        
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

    public static void speedChange(double currentSpeed){

        changeSpeed = currentSpeed;
        
    }
    public void zeroGyro(){
        gyro.setYaw(-90);
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
        RotationCount = Math.round(gyro.getYaw()/360);
        FOB = gyro.getYaw() - (RotationCount*360);


        SmartDashboard.putNumber("Yaw", (FOB));

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }


    public void EasyArm(){


    }


}