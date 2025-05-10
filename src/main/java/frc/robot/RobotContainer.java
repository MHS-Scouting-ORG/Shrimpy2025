// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
import frc.robot.commands.AlgaePivotStates.Barge;
import frc.robot.commands.AlgaePivotStates.Storage;
import frc.robot.commands.AlgaePivotStates.Tuck;
import frc.robot.commands.AlgaeShooterStates.AlgaeIntake;
import frc.robot.commands.AlgaeShooterStates.AlgaeShoot;
import frc.robot.commands.CoralStates.CoralDeployerCommand;
import frc.robot.commands.CoralStates.CoralIntakeCommand;
import frc.robot.commands.CoralStates.L2AndL3PosCommand;
import frc.robot.commands.ElevatorStates.L2State;
import frc.robot.commands.ElevatorStates.L3State;
import frc.robot.commands.ElevatorStates.StorageState;
import frc.robot.commands.IntegratedStates.AlgaeGroundPickupCommand;
import frc.robot.commands.IntegratedStates.BargeSequenceCommand;
import frc.robot.commands.IntegratedStates.FullTuckCommand;
import frc.robot.commands.IntegratedStates.HighDealgifyCommand;
import frc.robot.commands.IntegratedStates.L2AutomaticCommand;
import frc.robot.commands.IntegratedStates.L3AutomaticCommand;
import frc.robot.commands.IntegratedStates.L4AutomaticCommand;
import frc.robot.commands.IntegratedStates.L4SequenceCommand;
import frc.robot.commands.IntegratedStates.LowDealgifyCommand;
import frc.robot.commands.IntegratedStates.ProcessorSequenceCommand;
import frc.robot.commands.IntegratedStates.TuckWithAlgaeCommand;
import frc.robot.commands.SwerveStates.AlignModeCommand;

import com.pathplanner.lib.auto.NamedCommands;

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

    private final SwerveRequest.FieldCentricFacingAngle driveFieldFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5, 0, 0)
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private double desiredAngle = 0; 

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
    // public final Trigger rightTrigger = new Trigger(() -> xbox.right)
    public final Trigger algaeHeld = new Trigger(() -> algaeShooterSubsystem.algaeHeld); 

    public final Trigger intakeCoralTrigger = new Trigger(() -> coralIntakeSub.getOpticalSensor());

    public final Trigger readyToShoot = new Trigger(() -> (coralPivotSub.atSetpoint() && algaePivotSub.isAtSetpoint() && elevatorSub.atSetpoint() && elevatorSub.getEncoder() > 0)); 

    public final Trigger elevUp = new Trigger(() -> elevatorSub.getEncoder() > 30); 


  public RobotContainer() {
    registerNamedCommands(); 

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AUTO CHOOSER", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    ///////////////////////////////
    ///     DRIVER CONTROLS     ///
    ///////////////////////////////

    // ZERO HEADING 
    xbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // zero heading 

    //FIELD CENTRIC DEFAULT COMMAND 
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(-xbox.getLeftY() * MaxSpeed * 0.3) // Drive forward with negative Y (forward)
          .withVelocityY(-xbox.getLeftX() * MaxSpeed * 0.3) // Drive left with negative X (left)
          .withRotationalRate(-xbox.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
      )
    );

    // //ROBOT CENTRIC 
    xbox.x().whileTrue(
      drivetrain.applyRequest(() -> 
          driveRobotCentric.withVelocityX(-xbox.getLeftY() * MaxSpeed * 0.15)
          .withVelocityY(-xbox.getLeftX() * MaxSpeed * 0.15)
          .withRotationalRate(-xbox.getRightX() * MaxAngularRate * 0.15))
    );

    /* * * TESTING * * */
    // ALIGN REEF FIELD CENTRIC 
    // xbox.rightTrigger().and(() -> LimelightHelpers.getFiducialID("limelight") != -1).whileTrue(
    //   drivetrain.applyRequest(
    //     () -> driveFieldFacingAngle
    //     .withVelocityX(-xbox.getLeftY())
    //     .withVelocityY(-xbox.getLeftX())
    //     .withTargetDirection(Rotation2d.fromDegrees(getDesiredHeading()))
    //   )
    // );

    // LIGHTS FOR ROTATION ALIGN 
    xbox.back().whileTrue(new InstantCommand(() -> lights.setSolidColor(255,0,255)));
    xbox.back().whileFalse(new InstantCommand(() -> lights.off())); 

    //LIGHTS FOR ROBOT CENTRIC 
    xbox.x().whileTrue(new InstantCommand(() -> lights.setSolidColor(0, 2, 61))); 
    xbox.x().whileFalse(new InstantCommand(() -> lights.off())); 

    // ALGAE GROUND 
    xbox.a().onTrue(new AlgaeGroundPickupCommand(elevatorSub, algaePivotSub)); 

    // ALGAE OUTTAKE 
    xbox.b().whileTrue(new AlgaeShoot(algaeShooterSubsystem)); 
    xbox.b().whileFalse(new InstantCommand(() -> algaeShooterSubsystem.stopIntake())); 

    // CORAL INTAKE a
    xbox.rightBumper().whileTrue(new CoralIntakeCommand(coralIntakeSub));
    xbox.rightBumper().whileFalse(new InstantCommand(() -> coralIntakeSub.stopIntake()));
    // CORAL OUTTAKE 
    xbox.leftBumper().whileTrue(new CoralDeployerCommand(coralIntakeSub));
    xbox.leftBumper().whileFalse(new InstantCommand(() -> coralIntakeSub.stopIntake()));

    // BARGE
    xbox.y().onTrue(new BargeSequenceCommand(elevatorSub, algaePivotSub));
    // xbox.y().onTrue(new SequentialCommandGroup(new L3State(elevatorSub), new Barge(algaePivotSub)));


    /////////////////////////////////
    ///     OPERATOR CONTROLS     ///
    /////////////////////////////////
    
    // PROCESSOR 
    new JoystickButton(joystick, 3).onTrue(new ProcessorSequenceCommand(elevatorSub, algaePivotSub, coralPivotSub)); 

    // SCORING W/O DEALGIFYING 
    new JoystickButton(joystick, 7).onTrue(new L4AutomaticCommand(elevatorSub, coralPivotSub, algaePivotSub, coralIntakeSub)); 
    // new JoystickButton(joystick, 7).onTrue(new L4SequenceCommand(elevatorSub, coralPivotSub));
    
    // new JoystickButton(joystick, 9).onTrue(new L3State(elevatorSub)); 
    new JoystickButton(joystick, 9).onTrue(new L3AutomaticCommand(elevatorSub, coralIntakeSub, algaePivotSub, coralPivotSub));
    new JoystickButton(joystick, 11).onTrue(new L2AutomaticCommand(elevatorSub, coralIntakeSub, algaePivotSub, coralPivotSub)); 

    // new JoystickButton(joystick, 6).onTrue(new L2State(elevatorSub)); 

    // DEALGIFY 
    // new JoystickButton(joystick, 8).onTrue(new HighDealgifyCommand(elevatorSub, algaePivotSub, coralPivotSub)); 
    new JoystickButton(joystick, 8).onTrue(new SequentialCommandGroup(new L3State(elevatorSub), new Barge(algaePivotSub)));
    new JoystickButton(joystick, 10).onTrue(new LowDealgifyCommand(elevatorSub, algaePivotSub, coralPivotSub)); 

    // ALGAE TUCK 
    new JoystickButton(joystick, 1).onTrue(new Tuck(algaePivotSub)); 

    // TUCKING TEST 
    new JoystickButton(joystick, 12).and(
      () -> algaeShooterSubsystem.algaeHeld).onTrue(
        new TuckWithAlgaeCommand(elevatorSub, algaePivotSub, coralPivotSub)); //FIXME tucking test

    new JoystickButton(joystick, 12).and(
      () -> !algaeShooterSubsystem.algaeHeld).onTrue(
        new FullTuckCommand(elevatorSub, algaePivotSub, coralPivotSub)); //FIXME tucking test

    // ALGAE INTAKE 
    new JoystickButton(joystick, 2).onTrue(new AlgaeIntake(algaeShooterSubsystem)); 

    // MANUAL TUCK PIVOT 
    new JoystickButton(joystick, 4).onTrue(new L2AndL3PosCommand(coralPivotSub)); 

    /* * * TRIGGERS * * */
    intakeCoralTrigger.whileTrue(new InstantCommand(() -> lights.setSolidColor(255, 239, 2))); 
    intakeCoralTrigger.whileFalse(new InstantCommand( () -> lights.off()));




    // new JoystickButton(joystick, 5).onTrue(new L4SequenceCommand(elevatorSub, coralPivotSub));

    // readyToShoot.whileTrue(new InstantCommand(() -> lights.setSolidColor(124,252,0)));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("L4", new L4SequenceCommand(elevatorSub, coralPivotSub));

    NamedCommands.registerCommand("seqTuck", new SequentialCommandGroup(
      new L2AndL3PosCommand(coralPivotSub), 
      new FullTuckCommand(elevatorSub, algaePivotSub, coralPivotSub)
    ));

    NamedCommands.registerCommand("lowDealgify", new LowDealgifyCommand(elevatorSub, algaePivotSub, coralPivotSub));

    NamedCommands.registerCommand("outtake", new CoralDeployerCommand(coralIntakeSub));

    NamedCommands.registerCommand("intake", new CoralIntakeCommand(coralIntakeSub));

    NamedCommands.registerCommand("algaeIntake", new AlgaeIntake(algaeShooterSubsystem));

    NamedCommands.registerCommand("algaeTuck", new TuckWithAlgaeCommand(elevatorSub, algaePivotSub, coralPivotSub));

    NamedCommands.registerCommand("barge", new BargeSequenceCommand(elevatorSub, algaePivotSub));

    NamedCommands.registerCommand("algaeShoot", new AlgaeShoot(algaeShooterSubsystem));

    NamedCommands.registerCommand("fullTuck", new FullTuckCommand(elevatorSub, algaePivotSub, coralPivotSub));

    NamedCommands.registerCommand("elevDealgify", new StorageState(elevatorSub));
  }

  public double getDesiredHeading() {
    double tagID = LimelightHelpers.getFiducialID("limelight"); 

    if (tagID == 20 || tagID == 11) { // FAR A 
      return 240; 
      } else if (tagID == 21 || tagID == 10) { // FAR B 
      return 180; 
      } else if (tagID == 22 || tagID == 9) { // FAR C 
      return 120; 
      } else if (tagID == 19 || tagID == 6) { // CLOSE A 
      return 300; 
      } else if (tagID == 18 || tagID == 7) { // CLOSE B 
      return 0; 
      } else if (tagID == 17 || tagID == 8) { // CLOSE C 
      return 60; 
      } else {
        return 0; 
      }
  }
}
