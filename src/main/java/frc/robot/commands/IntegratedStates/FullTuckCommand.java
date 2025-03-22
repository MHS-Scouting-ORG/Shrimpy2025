package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotStates.Tuck;
import frc.robot.commands.CoralStates.L2AndL3PosCommand;
import frc.robot.commands.ElevatorStates.StorageState;
import frc.robot.commands.ElevatorStates.TuckState;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FullTuckCommand extends SequentialCommandGroup {

  public FullTuckCommand(ElevatorSubsystem elevatorSubsystem, AlgaePivot algaePivot, CoralPivotSubsystem coralPivotSubsystem) {
    addCommands(
      new L2AndL3PosCommand(coralPivotSubsystem), 
      new ParallelCommandGroup(
        new StorageState(elevatorSubsystem), 
        new Tuck(algaePivot)
      ), 
    new TuckState(elevatorSubsystem)
    );
  }
}
