package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CoralStates.L4PosCommand;
import frc.robot.commands.ElevatorStates.L4State;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L4SequenceCommand extends SequentialCommandGroup {

  public L4SequenceCommand(ElevatorSubsystem elevatorSubsystem, CoralPivotSubsystem coralPivotSubsystem) {
    addCommands(new L4State(elevatorSubsystem), new L4PosCommand(coralPivotSubsystem));
  }
}
