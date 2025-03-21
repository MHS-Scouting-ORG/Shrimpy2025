package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralStates.CoralDeployerCommand;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L4AutoScoreCommand extends SequentialCommandGroup {

  public L4AutoScoreCommand(ElevatorSubsystem elevatorSubsystem, CoralPivotSubsystem coralPivotSubsystem, AlgaePivot algaePivot, CoralIntakeSubsystem coralIntakeSubsystem) {

    addCommands(
      new L4SequenceCommand(elevatorSubsystem, coralPivotSubsystem), 
      new ParallelRaceGroup(
        new CoralDeployerCommand(coralIntakeSubsystem), 
        new WaitCommand(0.5)
      ), 
      new FullTuckCommand(elevatorSubsystem, algaePivot, coralPivotSubsystem)
    );
  }
}
