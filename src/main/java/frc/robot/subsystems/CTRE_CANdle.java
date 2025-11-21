package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*
  =NOTES=
  LEFT ELEVATOR SIDE: 8-124
  RIGHT ELEVATOR: 125-243
  MIDDLE BAR: 244-302

  ELEVATOR INCRIMENTS = 23-24
*/
public class CTRE_CANdle extends SubsystemBase {

  private CANdle m_lights = new CANdle(23);
  private CANdleConfiguration m_config = new CANdleConfiguration();

  //COLORS
  private static final RGBWColor kYellow = new RGBWColor(255, 125, 0);
  private static final RGBWColor kOrange = new RGBWColor(255, 50, 0);
  private static final RGBWColor kRed = new RGBWColor(225, 0, 0);
  private static final RGBWColor kBlue = new RGBWColor(0, 100, 255);
  private static final RGBWColor kGreen = new RGBWColor(0, 200, 0);
  private static final RGBWColor kPurple = new RGBWColor(150, 0, 200);
  private static final RGBWColor kLime = new RGBWColor(125, 255, 0);
  private static final RGBWColor kPink = new RGBWColor(255, 0, 150);
  private static final RGBWColor kCyan = new RGBWColor(0, 255, 200);

  public CTRE_CANdle() {
    m_config.LED.BrightnessScalar = 0.5;
    m_config.LED.StripType = StripTypeValue.GRB;
    m_lights.getConfigurator().apply(m_config);

    v_stopCurrentAnims();
    v_startupAnim();
  }

  public void v_startupAnim() {
    m_lights.setControl(new LarsonAnimation(8, 124).withColor(kLime).withFrameRate(240).withSlot(0).withSize(15).withBounceMode(LarsonBounceValue.Center));
    m_lights.setControl(new LarsonAnimation(125, 243).withColor(kCyan).withFrameRate(240).withSlot(1).withSize(15).withBounceMode(LarsonBounceValue.Center));
  }

  public void v_stopCurrentAnims() {
    m_lights.setControl(new EmptyAnimation(0));
    m_lights.setControl(new EmptyAnimation(1));
    m_lights.setControl(new EmptyAnimation(2));
    m_lights.setControl(new EmptyAnimation(3));
    m_lights.setControl(new EmptyAnimation(4));
    m_lights.setControl(new EmptyAnimation(5));
    m_lights.setControl(new EmptyAnimation(6));
    m_lights.setControl(new EmptyAnimation(7));
  }

  public void v_shootOut() {
    m_lights.setControl(new ColorFlowAnimation(244, 302).withColor(kRed).withFrameRate(700).withSlot(3));
  }

  public void v_intake() {
    m_lights.setControl(new ColorFlowAnimation(244, 302).withColor(kGreen).withFrameRate(700).withSlot(3));
  }

  public void v_stopI_O() {
    m_lights.setControl(new EmptyAnimation(3));
  }

  public void v_L1() {
    v_stopCurrentAnims();
    m_lights.setControl(new SingleFadeAnimation(8, 31).withColor(kLime).withFrameRate(120).withSlot(0));
    m_lights.setControl(new SingleFadeAnimation(125, 148).withColor(kLime).withFrameRate(120).withSlot(1));
  }

  public void v_L2() {
    v_stopCurrentAnims();
    m_lights.setControl(new SingleFadeAnimation(8, 54).withColor(kBlue).withFrameRate(120).withSlot(0));
    m_lights.setControl(new SingleFadeAnimation(125, 171).withColor(kBlue).withFrameRate(120).withSlot(1));
  }

  public void v_L3() {
    v_stopCurrentAnims();
    m_lights.setControl(new SingleFadeAnimation(8, 77).withColor(kPurple).withFrameRate(120).withSlot(0));
    m_lights.setControl(new SingleFadeAnimation(125, 194).withColor(kPurple).withFrameRate(120).withSlot(1));
  }

  public void v_L4() {
    v_stopCurrentAnims();
    m_lights.setControl(new SingleFadeAnimation(8, 100).withColor(kOrange).withFrameRate(120).withSlot(0));
    m_lights.setControl(new SingleFadeAnimation(125, 217).withColor(kOrange).withFrameRate(120).withSlot(1));
  }

  public void v_barge() {
    v_stopCurrentAnims();
    m_lights.setControl(new SingleFadeAnimation(8, 124).withColor(kCyan).withFrameRate(120).withSlot(0));
    m_lights.setControl(new SingleFadeAnimation(8, 243).withColor(kCyan).withFrameRate(120).withSlot(1));
    m_lights.setControl(new FireAnimation(244, 302).withFrameRate(60).withBrightness(0.5).withSparking(0.5).withCooling(0));
  }

  @Override
  public void periodic() {

  }
}
