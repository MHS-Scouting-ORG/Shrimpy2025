// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralStates;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1PosCommand extends Command {
  /** Creates a new L1CoralPivotCommand. */
  private CoralPivotSubsystem coralPivotSub;
  public L1PosCommand(CoralPivotSubsystem coralPivotSub) {
    this.coralPivotSub = coralPivotSub;
    addRequirements(this.coralPivotSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralPivotSub.setCoralPivotSetpoint(10);//test and change val
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(coralPivotSub.getPivotLimitSwitch()){
      coralPivotSub.resetCoralPivotEnc();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralPivotSub.atSetpoint() || coralPivotSub.getPivotLimitSwitch();
  }
}
