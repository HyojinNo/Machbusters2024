package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;
  private BooleanSupplier turboSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier slowSpeedSup,
      BooleanSupplier turboSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.slowSpeedSup = slowSpeedSup;
    this.turboSup = turboSup;
  }

  @Override
  public void execute() {

    double defaultSpeedMultiplier = 0.7;
    double speedMultiplier = defaultSpeedMultiplier;
    if (slowSpeedSup.getAsBoolean()) {
      speedMultiplier = 0.2;
    } else if (turboSup.getAsBoolean()) {
      speedMultiplier = 1.0;
    }

    /* Get Values, Deadband */
    // translationLimiter, strafeLimiter, and rotationLimiter are all instances of
    // SlewRateLimiter.
    // This ensures that quick movements on the joystick do not result in too fast
    // of changes in the hardware.
    // The resulting values still range from -1 to 1, with 0 being the neutral (no
    // change) position.

    double translationVal = // forward/back
        translationLimiter.calculate(
            // speedMultiplier is set above and enables slow speed mode
            speedMultiplier *
            // applyDeadband() clamps values that are near 0 to 0.0. This ensures that the
            // joystick when at a neutral position will result in a halted robot, even if
            // the joystick leans slightly in one direction.
                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal = // right/left
        strafeLimiter.calculate(
            speedMultiplier *
                MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal = rotationLimiter.calculate(
        speedMultiplier *
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

    /* Drive */
    // Instruct the Swerve subsystem to apply the calculated values.
    s_Swerve.drive(
        // Pass a Translation2d containing X and Y values proportional to the drive and
        // strafe axes on the joystick.
        // The translation values will be from -maxSpeed to +maxSpeed
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        robotCentricSup.getAsBoolean(),
        // Pass isOpenLoop as true
        true);
  }
}
