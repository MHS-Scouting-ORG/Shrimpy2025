package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotStates.Descore;
import frc.robot.commands.ElevatorStates.L3State;
import frc.robot.commands.ElevatorStates.StorageState;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.ElevatorSubsystem;

public class LowDealgifyCommand extends SequentialCommandGroup {

  public LowDealgifyCommand(ElevatorSubsystem elevatorSubsystem, AlgaePivot algaePivotSubsystem) {
  
    addCommands(
      new StorageState(elevatorSubsystem), 
      new ParallelCommandGroup( 
        new L3State(elevatorSubsystem), 
        new Descore(algaePivotSubsystem)
      )
    );
  }
}
