package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class CharacterizationTesting extends Command{

    private Drive swerveDrive;

    private double voltage = 0;
    public CharacterizationTesting(Drive swerveDrive){
        this.swerveDrive = swerveDrive;
        addRequirements(this.swerveDrive);
    }

    @Override
    public void initialize(){
        //voltage = 4;

    }

    @Override
    public void execute(){
        voltage += 0.01;
        swerveDrive.runCharacterization(voltage);

        Logger.recordOutput("/Characterization", voltage);

    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
