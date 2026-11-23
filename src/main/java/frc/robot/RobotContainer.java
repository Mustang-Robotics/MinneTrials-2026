// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  //private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> m_chooser; //creates a place to pick which auto routine we want to run from shuffleboard

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Pathplanner auto commands are registered here, this is where we can create our own commands to be used in pathplanner if we want to
    NamedCommands.registerCommand("ElevatorUp", new InstantCommand(() -> m_elevator.changeSetpoint(Setpoints.kElevatorHigh), m_elevator));
    NamedCommands.registerCommand("ArmDown", new InstantCommand(() -> m_arm.changeSetpoint(Setpoints.kArmLow), m_arm));
    NamedCommands.registerCommand("Reset", new ParallelCommandGroup(
      new InstantCommand(() -> m_elevator.changeSetpoint(Setpoints.kElevatorLow), m_elevator),
      new InstantCommand(() -> m_arm.changeSetpoint(Setpoints.kArmHigh), m_arm)
    ));

    // Configure the trigger bindings
    configureBindings(); //this is a method created below, we are running it here to define the triggers for commands
    m_chooser = AutoBuilder.buildAutoChooser(); //the chooser created above gets defined to select between pathplanner autos
    SmartDashboard.putData("Auto Chooser", m_chooser); //adding the chooser to shuffleboard
  }

  //method to configure trigger bindings any thing we want the robot to do based on some state (button press, or sensor value) should go in here
  private void configureBindings() {
    
    //m_driverController.a().onTrue(new InstantCommand(() -> m_elevator.changeSetpoint(1)));
    m_driverController.rightTrigger().onTrue(new InstantCommand(() -> m_elevator.changeSetpoint(Setpoints.kElevatorHigh), m_elevator));
    m_driverController.rightTrigger().onFalse(new InstantCommand(() -> m_arm.changeSetpoint(Setpoints.kArmLow), m_arm));
    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> m_elevator.changeSetpoint(Setpoints.kElevatorMid), m_elevator));
    m_driverController.leftTrigger().onFalse(new InstantCommand(() -> m_arm.changeSetpoint(Setpoints.kArmLow), m_arm));
    m_driverController.rightBumper().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> m_elevator.changeSetpoint(Setpoints.kElevatorLow), m_elevator), 
      new InstantCommand(() -> m_arm.changeSetpoint(Setpoints.kArmHigh), m_arm)
    ));
    m_driverController.start().whileTrue(new RunCommand(() -> m_elevator.ManualDown(), m_elevator));
    m_driverController.start().onFalse(new SequentialCommandGroup(
      new InstantCommand(() -> m_elevator.ResetElevatorZero(), m_elevator), 
      new InstantCommand(() -> m_elevator.StopElevator(), m_elevator),
      new InstantCommand(() -> m_arm.changeSetpoint(Setpoints.kArmHigh), m_arm)
    ));

    m_drive.setDefaultCommand(
        new ArcadeDrive(m_drive,() -> -m_driverController.getLeftY(),() -> -m_driverController.getRightX(),() -> m_driverController.leftBumper().getAsBoolean())); //using arcade drive command. telling which controller buttons are inputs
  }
  

  //making a command to call autonomous in Robot.java nothing else needs to be done here if we are just using pathplanner
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
