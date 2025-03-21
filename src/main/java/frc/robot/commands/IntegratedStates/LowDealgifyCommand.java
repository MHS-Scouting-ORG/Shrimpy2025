package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotStates.Descore;
import frc.robot.commands.CoralStates.L2AndL3PosCommand;
import frc.robot.commands.ElevatorStates.L3State;
import frc.robot.commands.ElevatorStates.LowDescoreState;
import frc.robot.commands.ElevatorStates.StorageState;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class LowDealgifyCommand extends SequentialCommandGroup {

  public LowDealgifyCommand(ElevatorSubsystem elevatorSubsystem, AlgaePivot algaePivotSubsystem, CoralPivotSubsystem coralPivotSubsystem) {
  
    addCommands(
      new ParallelCommandGroup(
        new StorageState(elevatorSubsystem), 
        new L2AndL3PosCommand(coralPivotSubsystem)
      ),
      new ParallelCommandGroup( 
        new LowDescoreState(elevatorSubsystem), 
        new Descore(algaePivotSubsystem)
      )
    );
  }
}
