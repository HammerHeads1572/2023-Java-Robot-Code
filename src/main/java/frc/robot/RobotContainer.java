package frc.robot;



import java.lang.ModuleLayer.Controller;
import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.node.BooleanNode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    public IntakeMotor m_Intake = new IntakeMotor(Constants.intakeMotorID,Constants.ArmLED);

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    

    private final Joystick driver2 = new Joystick(1);
    public final XboxController Driver = new XboxController(3);
    
    Trigger speedTrigger = new JoystickButton(Driver, 6);




    Trigger D1 = new JoystickButton(Driver, 1);//TODO set buttons to correct
    Trigger D2 = new JoystickButton(Driver, 2);
    Trigger D3 = new JoystickButton(Driver, 3);
    Trigger D4 = new JoystickButton(Driver, 4);

    Trigger D5 = new JoystickButton(Driver, 5);//TODO set buttons to correct
   // Trigger D6 = new JoystickButton(Driver, 6);
    Trigger D7 = new JoystickButton(Driver, 7);
    //Trigger D8 = new JoystickButton(Driver, 8);

    Trigger D9 = new JoystickButton(Driver, 9);
    Trigger D10 = new JoystickButton(Driver, 10);

    
  







    
    private final XboxController m_XboxController = new XboxController(2);
    Trigger aButton = new JoystickButton(m_XboxController, 1);
    Trigger bButton = new JoystickButton(m_XboxController, 2);
    Trigger xButton = new JoystickButton(m_XboxController, 3);
    Trigger yButton = new JoystickButton(m_XboxController, 4);

    
    Trigger lButton = new JoystickButton(m_XboxController, 5);
    Trigger rButton = new JoystickButton(m_XboxController, 6);


    Trigger Minus = new JoystickButton(m_XboxController, 7);
    Trigger Plus = new JoystickButton(m_XboxController, 8);


    Trigger Lstick = new JoystickButton(m_XboxController,9);
    Trigger Rstick = new JoystickButton(m_XboxController,10);

    Trigger RTrigger;
    Trigger LTrigger;
    



    
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;


    

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(Driver, 8);
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
                () -> -Driver.getRawAxis(translationAxis), 
                () -> -Driver.getRawAxis(strafeAxis), 
                () -> -Driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // trigger creation
        RTrigger = new Trigger(() -> {
            return m_XboxController.getRightTriggerAxis() > 0.25;
        });

        LTrigger = new Trigger(() -> {
            return m_XboxController.getLeftTriggerAxis() > 0.25;
        });

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
        Swerve.speedChange(1.);
    
        //Intake

       rButton.onTrue(new InstantCommand(() -> m_Intake.setSpeed(.6)));
       rButton.whileFalse(new InstantCommand(() -> m_Intake.setSpeed(0.)));

        

        //Exhast

       lButton.onTrue(new InstantCommand(() -> m_Intake.setSpeed(-.3)));
       lButton.whileFalse(new InstantCommand(() -> m_Intake.setSpeed(0.)));

        //xButton.onTrue(new InstantCommand(() -> {arm.setArmAngle(38);m_Wrist.setWristAngle(140);;}));

        yButton.onTrue(new InstantCommand(() -> {arm.setArmAngle(0);m_Wrist.setWristAngle(0);;}));

        //bButton.onTrue(new InstantCommand(() -> {arm.setArmAngle(-42);m_Wrist.setWristAngle(35);;}));

        aButton.onTrue(new InstantCommand(() -> {arm.setArmAngle(122);m_Wrist.setWristAngle(0);;}));

        //lButton.onTrue(new InstantCommand(() -> {arm.setArmAngle(40);m_Wrist.setWristAngle(140);;}));

        //rButton.onTrue(new InstantCommand(() -> {arm.setArmAngle(-45);m_Wrist.setWristAngle(60);;}));
        
        //Lstick.onTrue(new InstantCommand(() -> {arm.setArmAngle(88.5);m_Wrist.setWristAngle(135);;}));

        //Rstick.onTrue(new InstantCommand(() -> {arm.setArmAngle(-89);m_Wrist.setWristAngle(50);;}));
    
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        //Speed Control Button
        speedTrigger.onFalse(new InstantCommand(() -> Swerve.speedChange(1.)));
        speedTrigger.onTrue(new InstantCommand(() -> Swerve.speedChange(6.)));


//Driver Home Arm

        D4.onTrue(new InstantCommand(() -> {arm.setArmAngle(0);m_Wrist.setWristAngle(0);;}));

// opperator wrist offset
    RTrigger.onTrue(new InstantCommand(() -> { if (arm.TargetAngle < 0){
        m_Wrist.setWristAngle(m_Wrist.CurrentAngle + 10.);
    }
    else {
        m_Wrist.setWristAngle(m_Wrist.CurrentAngle + -10.);
    }
    }));

    LTrigger.onTrue(new InstantCommand(() -> { if (arm.TargetAngle < 0){
        m_Wrist.setWristAngle(m_Wrist.CurrentAngle + -10.);
    }
    else {
        m_Wrist.setWristAngle(m_Wrist.CurrentAngle + 10.);
    }
    }));



      //  RTrigger.whileTrue(new InstantCommand(() -> m_Wrist.setWristAngle(m_Wrist.CurrentAngle + 10.)));
       // RTrigger.onFalse(new InstantCommand(() -> m_Wrist.setWristAngle(m_Wrist.CurrentAngle)));
       // LTrigger.whileTrue(new InstantCommand(() -> m_Wrist.setWristAngle(m_Wrist.CurrentAngle + -10.)));
       // LTrigger.onFalse(new InstantCommand(() -> m_Wrist.setWristAngle(m_Wrist.CurrentAngle)));

// HP scoring
        xButton.onTrue(new InstantCommand(() -> { if (Swerve.FOB < 0){
            arm.setArmAngle(40);m_Wrist.setWristAngle(145);
        }
        else {
            arm.setArmAngle(-45);m_Wrist.setWristAngle(60);
        }
        }));
        
//Cone intake      
            
         bButton.onTrue(new InstantCommand(() -> { if (Swerve.FOB < 0){
                arm.setArmAngle(-45);m_Wrist.setWristAngle(45);
         }
          else {
              arm.setArmAngle(40);m_Wrist.setWristAngle(155);
          }
        }));

// Cube Scoring

      //  Rstick.onTrue(new InstantCommand(() -> { if (Swerve.FOB < 0){
      //      arm.setArmAngle(-45);m_Wrist.setWristAngle(60);
    // }
    //  else {
     //     arm.setArmAngle(40);m_Wrist.setWristAngle(140);
     // }
    //}));

    Rstick.onTrue(new InstantCommand(() -> {arm.setArmAngle(40);m_Wrist.setWristAngle(100);;}));


        




        D5.whileTrue(new InstantCommand(() -> arm.setArmAngle(arm.TargetAngle + 3.)));
        D5.onFalse(new InstantCommand(() -> arm.setArmAngle(arm.TargetAngle)));
        D7.whileTrue(new InstantCommand(() -> arm.setArmAngle(arm.TargetAngle + -3.)));
        D7.onFalse(new InstantCommand(() -> arm.setArmAngle(arm.TargetAngle)));
       


        D9.whileTrue(new InstantCommand(() -> m_Wrist.setWristAngle(m_Wrist.CurrentAngle + 10.)));
        D9.onFalse(new InstantCommand(() -> m_Wrist.setWristAngle(m_Wrist.CurrentAngle)));
        D10.whileTrue(new InstantCommand(() -> m_Wrist.setWristAngle(m_Wrist.CurrentAngle + -10.)));
        D10.onFalse(new InstantCommand(() -> m_Wrist.setWristAngle(m_Wrist.CurrentAngle)));
        //Driver.setRumble(RumbleType.kLeftRumble,1);

       // Driver.setRumble(RumbleType.kRightRumble,.1);

        int pov = Driver.getPOV();
        

       

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
        return new exampleAuto(s_Swerve, arm, m_Wrist, m_Intake);
 
    }
    /**
	 * 
	 * @param jValue is the joystick value input
	 * @return returns joystick value if outside of joystick threshold, else returns
	 *         zero
	 */
	public static double joystickThreshold(double jValue) {
		if (Math.abs(jValue) < .09) {
			return 0;
		} else {
			return 1 * jValue;
		}
	}
}