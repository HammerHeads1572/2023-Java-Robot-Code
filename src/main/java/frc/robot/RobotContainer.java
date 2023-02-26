package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;



import frc.robot.Autos.exampleAuto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.IntakeMotor;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /*Arm */
    //TODO: PID values need to be set
    private double[] kPIDArray = {5, 5, 5};
     Arm arm = new Arm(kPIDArray, Constants.armLeaderID, Constants.armFollowerID);

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);
    private final CommandXboxController exampleCommandController = new CommandXboxController(2); 
    
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = Joystick.AxisType.kX.value;
    int x =3;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton xButton = new JoystickButton(driver, 3);

    //private final XboxController xButton = new CommandXboxControllerButton(exampleCommandController,XboxController.Button.kX.value);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    
    /*Robotcontainer singleton */  
    private static RobotContainer robotContainer = new RobotContainer();
    
    /* Intake Declaration? */
    private final IntakeMotor robotIntake = new IntakeMotor(Constants.intakeMotorID);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        /* Setting up Operator controls */
        //Trigger xButton = exampleCommandController.x()
        //  .whileTrue(new TeleopArm(90, arm)); 
        // xButton.whileTrue(new TeleopArm(90, arm));
        new TeleopArm(0, arm);
        if (x==3)
        {
            System.err.println("hello");
            return;
        }
        new TeleopArm(90, arm);
        Trigger yButton = exampleCommandController.y()
          .whileTrue(new TeleopArm(0, arm)); 
        Trigger inTake = exampleCommandController.b()
            .whileTrue(new RunIntakeCommand())
            .whileFalse(new HoldIntakeCommand());
        Trigger outPut = exampleCommandController.a()
            .whileTrue(new ReleaseIntakeCommand());

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

    
    public static RobotContainer getInstance() {
        return robotContainer;
    }

    /*A chooser for autonomous commands*/
    private SendableChooser<Command> m_chooser = new SendableChooser<>();
    
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

    private void configureAutoCommands() {
        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
      
           /*  commandChooser.setDefaultOption("example auto", new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())));
           
            commandChooser.addOption("test auto",new InstantCommand(() -> s_Swerve.resetOdometry(testpath.getInitialPose())));
                   
                   SmartDashboard.putData("Command Chooser", commandChooser);
           
               

        */
        SmartDashboard.putData(m_chooser);

       
     
    }    
    
    //public Command getAutonomousCommand() {
       // return m_chooser.getSelected();
        
    //}
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);

     
}
}