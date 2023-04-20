package frc.robot.Autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.IntakeMotor;



public class exampleAuto extends SequentialCommandGroup {
   


    public exampleAuto(Swerve s_Swerve, Arm arm, Wrist m_Wrist, IntakeMotor m_Intake ){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

       
        Trajectory exampleTrajectory =
        
            TrajectoryGenerator.generateTrajectory(

            
    
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0,1.25),new Translation2d(0, 2.5),new Translation2d(0,3.75)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0,5,new Rotation2d(0)),
                
                config);
              
            

        

    

        var thetaController =
        //seting up PID
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                
                s_Swerve::getPose,

                Constants.Swerve.swerveKinematics,

                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),

                thetaController,

                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            new InstantCommand(() -> {arm.setArmAngle(-45); m_Wrist.setWristAngle(55);}),
            new WaitCommand(2.),
            new InstantCommand(() -> m_Intake.setSpeed(-.15)),
            new WaitCommand(1.),
            new InstantCommand(() -> {arm.setArmAngle(0); m_Wrist.setWristAngle(0);}),
            new WaitCommand(1.),
            new InstantCommand(() -> m_Intake.setSpeed(0)),
            swerveControllerCommand

        );
    }
}