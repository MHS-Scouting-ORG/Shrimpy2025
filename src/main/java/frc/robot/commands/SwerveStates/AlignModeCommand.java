// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveStates;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.DriverStation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignModeCommand extends Command {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
   
    private CommandSwerveDrivetrain drivetrain; 
    private DoubleSupplier xSupp, ySupp, zSupp; 
    public double desiredHeading, tagID; 

    // ROBOT CENTRIC FACING ANGLE SWERVE REQUEST 
    private final SwerveRequest.RobotCentricFacingAngle driveRobotFacingAngle = new SwerveRequest.RobotCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // REGULAR ROBOT CENTRIC SWERVE REQUEST 
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // FIELD CENTRIC FACING ANGLE SWERVE REQUEST  
    private final SwerveRequest.FieldCentricFacingAngle driveFieldFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // REGULAR FIELD CENTRIC SWERVE REQUEST 
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** Creates a new AlignModeCommands. */
  public AlignModeCommand(CommandSwerveDrivetrain drivetrain, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier zSupp) {
    this.drivetrain = drivetrain; 
    this.xSupp = xSupp; 
    this.ySupp = ySupp;
    this.zSupp = zSupp;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // HEADING STUFF 
    tagID = LimelightHelpers.getFiducialID("limelight");

    double xSpeed = xSupp.getAsDouble(); 
    double ySpeed = ySupp.getAsDouble(); 
    double zSpeed = zSupp.getAsDouble(); 

    if (tagID == 20 || tagID == 11) { // FAR A 
    desiredHeading = 240; 
    } else if (tagID == 21 || tagID == 10) { // FAR B 
    desiredHeading = 180; 
    } else if (tagID == 22 || tagID == 9) { // FAR C 
    desiredHeading = 120; 
    } else if (tagID == 19 || tagID == 6) { // CLOSE A 
    desiredHeading = 300; 
    } else if (tagID == 18 || tagID == 7) { // CLOSE B 
    desiredHeading = 0; 
    } else if (tagID == 17 || tagID == 8) { // CLOSE C 
    desiredHeading = 60; 
    } 

    if (tagID == -1) {
        SmartDashboard.putString("DRIVE MODE", "FIELD CENTRIC"); 
        drivetrain.setControl(drive
            .withVelocityX(-xSpeed * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-ySpeed * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-zSpeed * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    } else {
        SmartDashboard.putString("DRIVE MODE", "FIELD CENTRIC FACING ANGLE");
        drivetrain.setControl(driveFieldFacingAngle
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withVelocityX(-xSpeed * MaxSpeed)
            .withVelocityY(-ySpeed * MaxSpeed)
            .withTargetDirection(new Rotation2d(Units.degreesToRadians(desiredHeading)))
        );
    }

    SmartDashboard.putNumber("desiredHeading", desiredHeading); 
    // SmartDashboard.putNumber("", );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // private double getTagHeading() {
  //       double tagID = LimelightHelpers.getFiducialID("limelight"); 

  //       if (tagID == 20 || tagID == 11) { // FAR A 
  //           return 240; 
  //       } else if (tagID == 21 || tagID == 10) { // FAR B 
  //           return 180; 
  //       } else if (tagID == 22 || tagID == 9) { // FAR C 
  //           return 120; 
  //       } else if (tagID == 19 || tagID == 6) { // CLOSE A 
  //           return 300; 
  //       } else if (tagID == 18 || tagID == 7) { // CLOSE B 
  //           return 0; 
  //       } else if (tagID == 17 || tagID == 8) { // CLOSE C 
  //           return 60; 
  //       } else {
  //         return 0;
  //       }
  //   }
}
