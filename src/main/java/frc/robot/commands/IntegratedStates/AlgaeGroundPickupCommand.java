package frc.robot.commands.IntegratedStates;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotStates.GroundPickup;
import frc.robot.commands.AlgaePivotStates.Storage;
import frc.robot.commands.ElevatorStates.GroundPickupState;
import frc.robot.commands.ElevatorStates.StorageState;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeGroundPickupCommand extends SequentialCommandGroup {

  public AlgaeGroundPickupCommand(ElevatorSubsystem elevatorSubsystem, AlgaePivot algaePivot) {
    addCommands(
      new StorageState(elevatorSubsystem), 
      new GroundPickup(algaePivot), 
      new GroundPickupState(elevatorSubsystem)
    );
  }
}
