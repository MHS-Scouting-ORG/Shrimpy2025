package frc.robot.commands.CoralStates;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;


public class CoralDeployerSlower extends Command {
    private CoralIntakeSubsystem coralIntakeSub; 
  
    public CoralDeployerSlower(CoralIntakeSubsystem coralIntakeSub) {
      this.coralIntakeSub = coralIntakeSub;
      addRequirements(this.coralIntakeSub);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      coralIntakeSub.setIntakeSpeed(0.7);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      coralIntakeSub.stopIntake();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
