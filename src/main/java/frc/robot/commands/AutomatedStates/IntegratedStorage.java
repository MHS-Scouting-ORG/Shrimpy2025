// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutomatedStates;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.commands.ElevatorStates.ProcessorState;
import frc.robot.commands.AlgaePivotStates.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntegratedStorage extends SequentialCommandGroup {
  ElevatorSubsystem elevatorSub;
  AlgaePivot algaePivotSub;

  public IntegratedStorage(ElevatorSubsystem newElevatorSub, AlgaePivot newAlgaePivotSub) {
    elevatorSub = newElevatorSub;
    algaePivotSub = newAlgaePivotSub;
    addCommands(new ProcessorState(elevatorSub), new Storage(algaePivotSub));
    addRequirements(elevatorSub, algaePivotSub);
  }
}
