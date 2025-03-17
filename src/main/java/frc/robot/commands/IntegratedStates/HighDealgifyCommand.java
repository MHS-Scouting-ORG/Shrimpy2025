package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AlgaePivotStates.Descore;
import frc.robot.commands.ElevatorStates.L4State;
import frc.robot.commands.ElevatorStates.StorageState;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.ElevatorSubsystem;

public class HighDealgifyCommand extends ParallelCommandGroup {

  public HighDealgifyCommand(ElevatorSubsystem elevatorSubsystem, AlgaePivot algaePivotSubsystem) {
    addCommands(
      new StorageState(elevatorSubsystem), 
      new ParallelCommandGroup( 
        new L4State(elevatorSubsystem), 
        new Descore(algaePivotSubsystem)
      )
    );
  }
}
