package frc.robot;

import java.time.format.TextStyle;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;




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
    public Arm arm = new Arm(Constants.armPID, Constants.armLeaderID, Constants.armFollowerID);
    public Wrist m_Wrist = new Wrist(Constants.wristPID, 1, Constants.wristMotorID);
    public IntakeMotor m_Intake = new IntakeMotor(Constants.intakeMotorID);

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);
    private final XboxController m_XboxController = new XboxController(2);
    Trigger xButton = new JoystickButton(m_XboxController, 1);
    Trigger yButton = new JoystickButton(m_XboxController, 4);
    Trigger bButton = new JoystickButton(m_XboxController, 3);
    Trigger aButton = new JoystickButton(m_XboxController, 2);

    Trigger rTrigger = new JoystickButton(m_XboxController, 8);
    
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = Joystick.AxisType.kX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    
    /*Robotcontainer singleton */  
    private static RobotContainer robotContainer = new RobotContainer();

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
        /* Operator Buttons */
        xButton.onTrue(new InstantCommand(() -> {arm.setArmAngle(120); m_Wrist.setWristAngle(110);}));
        yButton.onTrue(new InstantCommand(() -> {arm.setArmAngle(0); m_Wrist.setWristAngle(0);}));
        bButton.onTrue(new InstantCommand(() -> arm.setArmAngle(-120)));

        aButton.whileTrue(new InstantCommand(() -> m_Intake.setSpeed(-1)));
        aButton.onFalse(new InstantCommand(() -> m_Intake.setSpeed(0)));
        rTrigger.onTrue(new InstantCommand(() -> m_Intake.setSpeed(1)));
        rTrigger.onFalse(new InstantCommand(() -> m_Intake.setSpeed(0)));
        
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