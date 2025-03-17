package frc.robot.commands.CoralStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;


public class CoralDeployerCommand extends Command {
    private CoralIntakeSubsystem coralIntakeSub; 
  
    public CoralDeployerCommand(CoralIntakeSubsystem coralIntakeSub) {
      this.coralIntakeSub = coralIntakeSub;
      addRequirements(this.coralIntakeSub);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      SmartDashboard.putString("Deploy Command State:", "Started");
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      SmartDashboard.putString("Deploy Command State:", "Running");
      coralIntakeSub.setIntakeSpeed(CoralConstants.CORAL_DEPLOY_SPEED);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      SmartDashboard.putString("Deploy Command State:", "Ended");
      coralIntakeSub.stopIntake();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
