package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotStates.Processor;
import frc.robot.commands.AlgaePivotStates.Storage;
import frc.robot.commands.CoralStates.L2AndL3PosCommand;
import frc.robot.commands.ElevatorStates.ProcessorState;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ElevatorStates.StorageState;

public class ProcessorSequenceCommand extends SequentialCommandGroup {

  public ProcessorSequenceCommand(ElevatorSubsystem elevatorSubsystem, AlgaePivot algaePivot, CoralPivotSubsystem coralPivotSubsystem) {
    addCommands(
      new ParallelCommandGroup(
        new StorageState(elevatorSubsystem), 
        new L2AndL3PosCommand(coralPivotSubsystem)
      ), 
      new Processor(algaePivot), 
      new ProcessorState(elevatorSubsystem));
  }
}
