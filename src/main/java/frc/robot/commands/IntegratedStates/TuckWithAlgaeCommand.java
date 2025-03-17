package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AlgaePivotStates.Storage;
import frc.robot.commands.CoralStates.L2AndL3PosCommand;
import frc.robot.commands.ElevatorStates.StorageState;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class TuckWithAlgaeCommand extends ParallelCommandGroup {

  public TuckWithAlgaeCommand(ElevatorSubsystem elevatorSubsystem, AlgaePivot algaePivot, CoralPivotSubsystem coralPivotSubsystem) {
    addCommands(new StorageState(elevatorSubsystem), new Storage(algaePivot), new L2AndL3PosCommand(coralPivotSubsystem));
  }
}
