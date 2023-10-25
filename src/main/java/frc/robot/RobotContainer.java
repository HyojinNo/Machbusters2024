// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Swerve;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final POVButton dPad_Right = new POVButton(driver, 90, 0);
  private final POVButton dPad_Top = new POVButton(driver, 0, 0);
  private final POVButton dPad_Left = new POVButton(driver, 270, 0);
  private final POVButton dPad_Down = new POVButton(driver, 180);

  /* Driver Buttons */
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);
  // private final JoystickButton fastSpeed = new JoystickButton(driver,
  // XboxController.Button.kRightBumper.value);
  private final SendableChooser<String> chooser;
  private final JoystickButton slowSpeed = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton m_intakeIn = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton m_intakeOut = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton m_push = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton m_pull = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton turbo = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Intaker s_Intaker = new Intaker();
  public final Lifter s_Lifter = new Lifter();

  /* Commands */
  public final IntakeIn c_IntakeIn = new IntakeIn(s_Intaker);
  public final IntakeOut c_IntakeOut = new IntakeOut(s_Intaker);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    chooser = new SendableChooser<String>();
    chooser.setDefaultOption("straight forward", "straight");
    chooser.addOption("straight right", "right"); // etc
    chooser.addOption("straight left", "left");
    chooser.addOption("does nothing", "stand still");

    SmartDashboard.putData("Auto Selector", chooser);
    // SendableRegistry.setName(chooser, "Auto Selector");

    new ShuffleboardWrapper(chooser);
  }

  public void teleopInit() {
    this.resetToAbsoluteNorth();

    s_Swerve.setDefaultCommand(
        // Command that's continuously run to update the swerve state
        new TeleopSwerve(
            // The Swerve subsystem
            s_Swerve,
            // getRawAxis() returns a value for the controller axis from -1 to 1
            () -> driver.getRawAxis(translationAxis),
            () -> driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis),
            // robotCentric and slowSpeed are both buttons on the joystick
            // The robotCentric button, when held down, enables axis behavior relative to
            // the field (and requires a working gyroscope). The default
            // is for movements to apply relative to the robot.
            () -> robotCentric.getAsBoolean(),
            // slowSpeed button, when held, causes translation and rotation to be performed
            // at a slower speed
            () -> slowSpeed.getAsBoolean(),
            () -> turbo.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    dPad_Down.whileTrue(new RunCommand(() -> s_Swerve.setX(), s_Swerve));

    // back_resetPosition.onTrue(new
    // InstantCommand(()->s_Swerve.resetToAbsoluteNorth()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new BusterAuto(this, this.chooser);
  }

  public void resetToAbsoluteNorth() {
    s_Swerve.resetToAbsoluteNorth();
  }

  /**
   * This is run constantly as soon as the robot is plugged in.
   */
  public void periodic() {
    s_Lifter.checkLimits();
    s_Intaker.periodic();
    SmartDashboard.putString("Choosen Auto", chooser.getSelected());
  }

  public void killTeleop() {
    s_Swerve.removeDefaultCommand();
  }

}
