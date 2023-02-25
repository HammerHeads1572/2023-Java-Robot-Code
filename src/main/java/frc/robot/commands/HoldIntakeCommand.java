package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class HoldIntakeCommand extends CommandBase {
    /*IMPORTANT:
     * intake runs -> holdingIntake = true, releaseIntake = false
     * holdingIntake = true -> intake holds
     * intake releases -> holdingIntake = true, releaseIntake = true
     * releaseIntake = true -> holdingIntake = false
     * releaseIntake = true & holdingIntake = false -> releaseIntake = false
     */
    public void execute() {
        if (RunIntakeCommand.holdingIntake == true) {
            
        }
    }
}
