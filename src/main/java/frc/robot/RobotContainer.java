package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;


import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;
import frc.lib.util.DriveTrain;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = Joystick.AxisType.kX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    
    
    //SwerveModuleConstants flModule = new SwerveModuleConstants(2, 1, 9, Rotation2d.fromDegrees(266.57));
    SwerveModule flModule = new SwerveModule(2, 1, 9, Rotation2d.fromDegrees(266.57));
    SwerveModule frModule = new SwerveModule(4, 3, 9, Rotation2d.fromDegrees(185.53));
    SwerveModule blModule = new SwerveModule(6, 5, 9, Rotation2d.fromDegrees(139.04));
    SwerveModule brModule = new SwerveModule(8, 7, 9, Rotation2d.fromDegrees(262.79));
    Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID,"Canivore");
    

    //tried deretly copying it into robotcontainer
    //tried making it into a class in swerve but this is the best soulution we have
    //tried messing around with name 
    //tried making it static 
    
    
    DriveTrain Drivetrain = new DriveTrain(gyro, flModule, frModule, blModule, brModule);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver2.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        
        configureAutoCommands();
        configureButtonBindings();
    }

    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }
    // A chooser for autonomous commands
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();



    private void configureAutoCommands() {
        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        PathPlannerTrajectory examplePath = 
        PathPlanner.loadPath(
                "basic path", new PathConstraints(4, 3));

                if (examplePath == null ) {
                    throw new IllegalArgumentException("example path is null.");
                }
                if (Drivetrain == null) {
                    throw new IllegalArgumentException("drivetrain is null");
                }
//  TODO: drivrain hates humman life and makes me very sad with nullpointexeptions :(
    
        Command examplePathCommand = new FollowPath(examplePath, s_Swerve, true){  
        };

        m_chooser.setDefaultOption("examplePath", examplePathCommand);
     
        SmartDashboard.putData(m_chooser);
    }    
    
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
        
    }

     
}
