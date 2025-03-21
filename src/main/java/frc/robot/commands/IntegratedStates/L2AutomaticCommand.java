package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralStates.CoralDeployerCommand;
import frc.robot.commands.ElevatorStates.L2State;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class L2AutomaticCommand extends SequentialCommandGroup {

  public L2AutomaticCommand(ElevatorSubsystem elevatorSubsystem, CoralIntakeSubsystem coralIntakeSubsystem, AlgaePivot algaePivot, CoralPivotSubsystem coralPivotSubsystem) {
    addCommands(
      new L2State(elevatorSubsystem), 
      new ParallelRaceGroup(
        new CoralDeployerCommand(coralIntakeSubsystem), 
        new WaitCommand(0.5)
      ), 
      new FullTuckCommand(elevatorSubsystem, algaePivot, coralPivotSubsystem)
    );
  }
}
