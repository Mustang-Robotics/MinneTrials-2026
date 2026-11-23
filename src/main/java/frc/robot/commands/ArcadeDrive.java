package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends Command{
    DriveSubsystem m_drive;
    DoubleSupplier m_xSupplier;
    DoubleSupplier m_zSupplier;
    BooleanSupplier m_halfSpeedSupplier;
    private boolean halfSpeed = true;
    private boolean previousButtonState = false;
  
    public ArcadeDrive(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier, BooleanSupplier halfSpeedSupplier){
      m_drive = drive;
      m_xSupplier = xSupplier;
      m_zSupplier = zSupplier;
      m_halfSpeedSupplier = halfSpeedSupplier;

      addRequirements(m_drive);
    }

    @Override
    public void execute(){
      double x = MathUtil.applyDeadband(m_xSupplier.getAsDouble(), OperatorConstants.kDeadband);
      double z = MathUtil.applyDeadband(m_zSupplier.getAsDouble(), OperatorConstants.kDeadband);

      z = Math.copySign(Math.pow(z, 2), z) * DriveConstants.kMaxRotation; //squared inputs for more control at low speeds
      x = Math.copySign(Math.pow(x, 2), x); //squared inputs for more control at low speeds

      boolean buttonPressed = m_halfSpeedSupplier.getAsBoolean();
      if (buttonPressed && !previousButtonState) {
        halfSpeed = !halfSpeed; // Toggle the halfSpeed variable
      }
      previousButtonState = buttonPressed;
      double speedMultiplier;
      if(halfSpeed){
        speedMultiplier = 0.5;
      }else{
        speedMultiplier = 1.0;
      }
      SmartDashboard.putBoolean("Half Speed", halfSpeed);
      var speeds = DifferentialDrive.arcadeDriveIK(x, z, false);
      
      m_drive.runOpenLoop(
        speeds.left * DriveConstants.kMaxSpeedMetersPerSec * DriveConstants.kVoltPerSpeed * speedMultiplier, 
        speeds.right * DriveConstants.kMaxSpeedMetersPerSec * DriveConstants.kVoltPerSpeed * speedMultiplier);
    }
}
