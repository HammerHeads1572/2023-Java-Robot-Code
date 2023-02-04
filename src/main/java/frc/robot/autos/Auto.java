package frc.robot.autos;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class Auto extends SequentialCommandGroup {
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));

    // This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
    // Or the path can be sampled at a given point in time for custom path following

}