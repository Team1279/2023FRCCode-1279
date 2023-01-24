// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  DoubleSolenoid ArmSolenoidPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.ARM_EXTEND_ID, Constants.PneumaticIDs.ARM_RETRACT_ID);
  DoubleSolenoid ConeSolenoidPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.CONE_GRAB_ID, Constants.PneumaticIDs.CONE_RELEASE_ID);
  DoubleSolenoid SubeSolenoidPCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticIDs.CUBE_GRAB_ID, Constants.PneumaticIDs.CUBE_RELEASE_ID);
  /** Creates a new Pneumatics. */
  public Pneumatics() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
  public void ArmRetract()
  {
    ArmSolenoidPCM.set(DoubleSolenoid.Value.kReverse);
  }
  public void ArmExtend()
  {
    ArmSolenoidPCM.set(DoubleSolenoid.Value.kForward);
  }
  public void ConeRelease()
  {
    ConeSolenoidPCM.set(DoubleSolenoid.Value.kReverse);
  }
  public void ConeGrab()
  {
    ConeSolenoidPCM.set(DoubleSolenoid.Value.kForward);
  }
  public void CubeRelease()
  {
    SubeSolenoidPCM.set(DoubleSolenoid.Value.kForward);
  }
  public void CubeGrab()
  {
    SubeSolenoidPCM.set(DoubleSolenoid.Value.kReverse);
  }
}  