package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReleaseIntakeCommand extends CommandBase{
    /*IMPORTANT:
     * intake runs -> holdingIntake = true, releaseIntake = false
     * holdingIntake = true -> intake holds
     * intake releases -> holdingIntake = true, releaseIntake = true
     * releaseIntake = true -> holdingIntake = false
     * releaseIntake = true & holdingIntake = false -> releaseIntake = false
     */
    public static boolean releaseIntake;

    public void execute() {
        
        releaseIntake = true;

        if (RunIntakeCommand.holdingIntake == false && releaseIntake == true) {
            releaseIntake = false;
        }
    }
}
