
package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants.Setpoints;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    // Creates a SparkMax motor controller for the elevator & an encoder
    private final SparkMax m_elevator = new SparkMax(8, MotorType.kBrushless);
    private SparkClosedLoopController elevatorClosedLoopController =
        m_elevator.getClosedLoopController();

    public RelativeEncoder elevatorEncoder = m_elevator.getEncoder();
    private double elevatorTarget = Setpoints.kElevatorLow;
    public double SpeedAdjustment = 1;

    // Tracks the previous robot state
    private boolean wasDisabled = true;
    private boolean manualControl = false;

    public Elevator() {

        m_elevator.configure(
            Configs.Elevator.elevatorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        elevatorEncoder.setPosition(0);
    }

    public void changeSetpoint(double setpoint) {
        elevatorTarget = setpoint;
    }

    private void moveToSetpoint() {
        elevatorClosedLoopController.setSetpoint(
            elevatorTarget,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    private void ElevatorSpeedAdjustment() {
        SpeedAdjustment = 1 - (elevatorEncoder.getPosition() / 5000);
    }

    private void SetSpeed(double speed) {
        m_elevator.set(speed);
    }

    public void ManualDown() {
        if (!manualControl) {
            SetCurrentLimit(10);
            manualControl = true;
        }
        SetSpeed(-0.15);
    }

    public void ResetElevatorZero() {
        elevatorEncoder.setPosition(0);
    }

    public void StopElevator() {
        SetSpeed(0);
        SetCurrentLimit(50);
        manualControl = false;
        elevatorTarget = Setpoints.kElevatorLow;
    }

    private void SetCurrentLimit(int limit) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(limit);

    m_elevator.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters
    );
}

    // Changes idle mode when the robot changes between enabled and disabled
    private void updateIdleMode() {
        boolean isDisabled = DriverStation.isDisabled();

        if (isDisabled != wasDisabled) {

            SparkMaxConfig config = new SparkMaxConfig();

            if(isDisabled) {
                config.idleMode(IdleMode.kBrake);
            } else {
                config.idleMode(IdleMode.kCoast);
            }

            m_elevator.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            );

            wasDisabled = isDisabled;
        }
    }

    @Override
    public void periodic() {
        updateIdleMode();
        if(!manualControl){
          moveToSetpoint();
        }
        ElevatorSpeedAdjustment();
    }
}
