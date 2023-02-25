package frc.lib.util;
import frc.robot.SwerveModule;
import frc.lib.RobotConfig;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public final Pigeon2 gyro;    
    public final SwerveModule flModule;
    public final SwerveModule frModule;
    public final SwerveModule blModule;
    public final SwerveModule brModule;
    
    
    /* Auto Drive Motor PID Values */
    private static final double AUTO_TURN_P_CONTROLLER = 10.0;
    private static final double AUTO_TURN_I_CONTROLLER = 0.0;
    private static final double AUTO_TURN_D_CONTROLLER = 0.0;
        
    public double getAutoTurnKP() {
      return AUTO_TURN_P_CONTROLLER;
    }
      
    public double getAutoTurnKI() {
      return AUTO_TURN_I_CONTROLLER;
    }
      
    public double getAutoTurnKD() {
      return AUTO_TURN_D_CONTROLLER;
    }
    
    public final PIDController autoThetaController =
        new PIDController(getAutoTurnKP(), getAutoTurnKI(), getAutoTurnKD());
    
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

    /** Constructs a new DrivetrainSubsystem method. 
     * 
     * 
    */
    public DriveTrain(
        Pigeon2 gyro,
        SwerveModule brModule,
        SwerveModule flModule,
        SwerveModule blModule,
        SwerveModule frModule
        
        ) {
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
    this.gyro = gyro;
    this.brModule = brModule;
    this.flModule = flModule;
    this.blModule = blModule;
    this.frModule = frModule;
    
    this.mSwerveMods[0] = brModule;
    this.mSwerveMods[1] = flModule;
    this.mSwerveMods[2] = blModule;
    this.mSwerveMods[3] = frModule;
    
    
    
    gyro = new Pigeon2(Constants.Swerve.pigeonID,"Canivore");
    gyro.configFactoryDefault();
    zeroGyro();

    this.autoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.centerGravity = new Translation2d(); // default to (0,0)

    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
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

    public void setSwerveModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, RobotConfig.getRobotMaxAngularVelocity());
    
        for (SwerveModule swerveModule :  mSwerveMods ) {
          swerveModule.setDesiredState(states[swerveModule.getModuleNumber()], false);
        }
      }
    
    
}
