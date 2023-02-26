package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunIntakeCommand extends CommandBase {
    /*IMPORTANT:
     * intake runs -> holdingIntake = true, releaseIntake = false
     * holdingIntake = true -> intake holds
     * intake releases -> holdingIntake = true, releaseIntake = true
     * releaseIntake = true -> holdingIntake = false
     * releaseIntake = true & holdingIntake = false -> releaseIntake = false
     */
    public static boolean holdingIntake;

    public void execute() {
        
        holdingIntake = true;

    }

    public void end() {
        if (ReleaseIntakeCommand.releaseIntake == true) {
            holdingIntake = false;
        }
    }
    
}

