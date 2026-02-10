package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RoboSingSubsystem extends SubsystemBase {
  private final Orchestra orchestra = new Orchestra();
  private String loadedSong = null;

  private final TalonFX motor1 = new TalonFX(0);
  private final TalonFX motor2 = new TalonFX(1);
  private final TalonFX motor3 = new TalonFX(2);
  private final TalonFX motor4 = new TalonFX(3);
  private final TalonFX motor5 = new TalonFX(4);
  private final TalonFX motor6 = new TalonFX(5);
  private final TalonFX motor7 = new TalonFX(6);
  private final TalonFX motor8 = new TalonFX(7);

  public RoboSingSubsystem() {
    orchestra.addInstrument(motor1);
    orchestra.addInstrument(motor2);
    orchestra.addInstrument(motor3);
    orchestra.addInstrument(motor4);
    orchestra.addInstrument(motor5);
    orchestra.addInstrument(motor6);
    orchestra.addInstrument(motor7);
    orchestra.addInstrument(motor8);

    loadSong("roborock_test.chrp");
  }

  public boolean loadSong(String chrpFileName) {
    StatusCode status = orchestra.loadMusic(chrpFileName);
    if (!status.isOK()) {
      DriverStation.reportError("Failed to load CHRP '" + chrpFileName + "': " + status, false);
      loadedSong = null;
      return false;
    }
    loadedSong = chrpFileName;
    DriverStation.reportWarning("Loaded CHRP: " + chrpFileName, false);
    return true;
  }

  public void play() {
    if (loadedSong == null) {
      DriverStation.reportWarning("No CHRP loaded, cannot play.", false);
      return;
    }
    orchestra.play();
  }

  public void pause() {
    orchestra.pause();
  }

  public void stop() {
    orchestra.stop();
  }

  public boolean isPlaying() {
    return orchestra.isPlaying();
  }
}
