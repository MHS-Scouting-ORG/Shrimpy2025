// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.AlgaeShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralPivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Lights;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.Telemetry;
import frc.robot.commands.AlgaePivotStates.Tuck;
import frc.robot.commands.AlgaeShooterStates.AlgaeIntake;
import frc.robot.commands.AlgaeShooterStates.AlgaeShoot;
import frc.robot.commands.CoralStates.CoralDeployerCommand;
import frc.robot.commands.CoralStates.CoralIntakeCommand;
import frc.robot.commands.ElevatorStates.L2State;
import frc.robot.commands.ElevatorStates.L3State;
import frc.robot.commands.IntegratedStates.AlgaeGroundPickupCommand;
import frc.robot.commands.IntegratedStates.BargeSequenceCommand;
import frc.robot.commands.IntegratedStates.FullTuckCommand;
import frc.robot.commands.IntegratedStates.HighDealgifyCommand;
import frc.robot.commands.IntegratedStates.L4SequenceCommand;
import frc.robot.commands.IntegratedStates.LowDealgifyCommand;
import frc.robot.commands.IntegratedStates.ProcessorSequenceCommand;
import frc.robot.commands.IntegratedStates.TuckWithAlgaeCommand;

public class RobotContainer {

    /* * * SWERVE * * */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* * * CONTROLLERS * * */ 
    private final CommandXboxController xbox = new CommandXboxController(0);
    private final Joystick joystick = new Joystick(1); 

    /* * * AUTO CHOOSER * * */
  private final SendableChooser<Command> autoChooser;

    /* * * SUBSYSTEMS * * */
    // SPARKS
    private SparkMax intakeMotor = new SparkMax(8, MotorType.kBrushless);
    private final SparkMax coralPivot = new SparkMax(7, MotorType.kBrushless);
    // CORAL INTAKE 
    private final CoralIntakeSubsystem coralIntakeSub = new CoralIntakeSubsystem(intakeMotor);
    // CORAL PIVOT 
    private final CoralPivotSubsystem coralPivotSub = new CoralPivotSubsystem(intakeMotor.getForwardLimitSwitch(), coralPivot);
    // ALGAE PIVOT 
    private final AlgaePivot algaePivotSub = new AlgaePivot(coralPivot.getForwardLimitSwitch());
    private final AlgaeShooter algaeShooterSubsystem = new AlgaeShooter(); 
    // ELEVATOR 
    private final ElevatorSubsystem elevatorSub = new ElevatorSubsystem();
    // LIGHTS 
    private final Lights lights = new Lights(); 

    //LIGHT TRIGGER 
    // public final Trigger intakeCoralTrigger = new Trigger(() -> coralIntakeSub.get);


  public RobotContainer() {
    registerNamedCommands(); 

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    ///////////////////////////////
    ///     DRIVER CONTROLS     ///
    ///////////////////////////////

    // ZERO HEADING 
    xbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // zero heading 

    // FIELD CENTRIC DEFAULT COMMAND 
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
        drive.withVelocityX(-xbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-xbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-xbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    //ROBOT CENTRIC / ALIGN REEF 
    xbox.x().whileTrue(
      drivetrain.applyRequest(() -> 
          driveRobotCentric.withVelocityX(-xbox.getLeftY() * MaxSpeed * 0.5)
          .withVelocityY(-xbox.getLeftX() * MaxSpeed * 0.5)
          .withRotationalRate(-xbox.getRightX() * MaxAngularRate * 0.75))
    );

    //LIGHTS FOR ALIGN MODE 
    xbox.x().whileTrue(new InstantCommand(() -> lights.setSolidColor(0, 2, 61))); 
    xbox.x().whileFalse(new InstantCommand(() -> lights.off())); 

    // ALGAE GROUND 
    xbox.a().onTrue(new AlgaeGroundPickupCommand(elevatorSub, algaePivotSub)); 

    /////////////////////////////////
    ///     OPERATOR CONTROLS     ///
    /////////////////////////////////
    
    new JoystickButton(joystick, 12).onTrue(new FullTuckCommand(elevatorSub, algaePivotSub, coralPivotSub)); 
    new JoystickButton(joystick, 4).onTrue(new TuckWithAlgaeCommand(elevatorSub, algaePivotSub, coralPivotSub));

    // PROCESSOR 
    new JoystickButton(joystick, 3).onTrue(new ProcessorSequenceCommand(elevatorSub, algaePivotSub, coralPivotSub)); 

    // SCORING W/O DEALGIFYING 
    new JoystickButton(joystick, 7).onTrue(new L4SequenceCommand(elevatorSub, coralPivotSub)); 
    new JoystickButton(joystick, 9).onTrue(new L3State(elevatorSub)); 
    new JoystickButton(joystick, 11).onTrue(new L2State(elevatorSub)); 

    // DEALGIFY 
    new JoystickButton(joystick, 8).onTrue(new HighDealgifyCommand(elevatorSub, algaePivotSub, coralPivotSub)); 
    new JoystickButton(joystick, 10).onTrue(new LowDealgifyCommand(elevatorSub, algaePivotSub, coralPivotSub)); 

    // ALGAE TUCK 
    new JoystickButton(joystick, 1).onTrue(new  Tuck(algaePivotSub)); 

    // BARGE
    new JoystickButton(joystick, 6).onTrue(new BargeSequenceCommand(elevatorSub, algaePivotSub));

    // ALGAE INTAKE OVERRIDE 
    new JoystickButton(joystick, 2).whileTrue(new AlgaeIntake(algaeShooterSubsystem)); 
    new JoystickButton(joystick, 2).whileFalse(new InstantCommand(() -> algaeShooterSubsystem.stopIntake()));
    new JoystickButton(joystick, 5).whileTrue(new AlgaeShoot(algaeShooterSubsystem)); 
    new JoystickButton(joystick, 5).whileFalse(new InstantCommand(() -> algaeShooterSubsystem.stopIntake()));

    // CORAL INTAKE
    xbox.leftBumper().whileTrue(new CoralIntakeCommand(coralIntakeSub));
    xbox.leftBumper().whileFalse(new InstantCommand(() -> coralIntakeSub.stopIntake()));
    xbox.rightBumper().whileTrue(new CoralDeployerCommand(coralIntakeSub));
    xbox.rightBumper().whileFalse(new InstantCommand(() -> coralIntakeSub.stopIntake()));
    

    // TESTING L4 
    // xbox.y().onTrue(new L4SequenceCommand(elevatorSub, coralPivotSub)); 
    // xbox.a().onTrue(new FullTuckCommand(elevatorSub, algaePivotSub, coralPivotSub)); 

    // xbox.leftBumper().whileTrue(new CoralIntakeCommand(coralIntakeSub)); 
    // xbox.leftBumper().whileFalse(new InstantCommand(() -> coralPivotSub.stop())); 

    // xbox.rightBumper().whileTrue(new CoralDeployerCommand(coralIntakeSub)); 
    // xbox.rightBumper().whileFalse(new InstantCommand(() -> coralPivotSub.stop()));

    /* * * TRIGGERS * * */
    // intakeCoralTrigger.whileTrue(new InstantCommand(() -> lights.setSolidColor(255, 239, 2))); 
    // intakeCoralTrigger.whileFalse(new InstantCommand( () -> lights.off()));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {

  }
}
