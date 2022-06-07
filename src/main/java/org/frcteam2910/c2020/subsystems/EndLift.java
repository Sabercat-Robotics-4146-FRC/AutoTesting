package org.frcteam2910.c2020.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;

public class EndLift implements Subsystem {

  private final CANSparkMax liftMotorLeader;
  private final CANSparkMax liftMotorFollower;

  private final Solenoid liftPin;

  private boolean pinToggle;

  public EndLift() {
    liftMotorLeader = new CANSparkMax(Constants.END_LIFT_LEFT, MotorType.kBrushless);
    liftMotorFollower = new CANSparkMax(Constants.END_LIFT_RIGHT, MotorType.kBrushless);

    liftMotorLeader.setInverted(true);
    liftMotorFollower.follow(liftMotorLeader, true);

    CANSparkMax[] sparkMaxs = {liftMotorLeader, liftMotorFollower};

    for (var sparkMax : sparkMaxs) {
      sparkMax.setSmartCurrentLimit(80); // current limit (amps)
      sparkMax.setOpenLoopRampRate(.5); // # seconds to reach peak throttle
    }

    liftPin = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.LIFT_PIN);

    pinToggle = false;
  }

  public void reverseSpool() {
    liftMotorLeader.setVoltage(-1.5);
  }

  public void SendSpool() {
    liftMotorLeader.setVoltage(12);
  }

  public void stopLift() {
    liftMotorLeader.stopMotor();
  }

  public void togglePin() {
    pinToggle = !pinToggle;
    liftPin.set(pinToggle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("pin toggle", pinToggle);
  }
}
