package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotStates.Barge;
import frc.robot.commands.ElevatorStates.L4State;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.ElevatorSubsystem;

public class BargeSequenceCommand extends SequentialCommandGroup {

  public BargeSequenceCommand(ElevatorSubsystem elevatorSubsystem, AlgaePivot algaePivot) {
    addCommands(new L4State(elevatorSubsystem), new Barge(algaePivot));
  }
}
