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

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        // to make a new trajectry just copy and paste this whole block
        Trajectory exampleTrajectory =

        
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1),
                        new Translation2d(0,0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);
              

        var thetaController =
        //seting up PID
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                //chnge this to whatever trajectry you want
                exampleTrajectory,
                //s_swerve is the cLass this is in. This line is setting the starting pose which comes from line 37
                s_Swerve::getPose,
                // convrts between chasis speed and swerve drive state
                Constants.Swerve.swerveKinematics,
                //A PID ia a specil type of feedback loop
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                //this uses the PID to make a auto path
                thetaController,
                //this checks what state moudles are in
                s_Swerve::setModuleStates,
                //this line does nothing I think
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}