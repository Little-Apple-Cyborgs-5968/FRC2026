package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RoboSingSubsystem extends SubsystemBase {
  private final Orchestra orchestra = new Orchestra();
  private String loadedSong = null;

  // private final TalonFX motor1 = new TalonFX(0);
  // private final TalonFX motor2 = new TalonFX(1);
  // private final TalonFX motor3 = new TalonFX(2);
  // private final TalonFX motor4 = new TalonFX(3);
  // private final TalonFX motor5 = new TalonFX(4);
  private final TalonFX diddyMotor1 = new TalonFX(12);
  private final TalonFX diddyMotor2 = new TalonFX(14);
  private final TalonFX diddyMotor3 = new TalonFX(15);

  public RoboSingSubsystem() {
    orchestra.addInstrument(diddyMotor1);
    orchestra.addInstrument(diddyMotor2);
    orchestra.addInstrument(diddyMotor3);

    loadSong("roborock_test.chrp");
  }

  public boolean loadSong(String chrpFileName) {
    StatusCode status = orchestra.loadMusic(chrpFileName);
    if (!status.isOK()) {
      System.out.println("Failed to load CHRP '" + chrpFileName + "': " + status);
      loadedSong = null;
      return false;
    }
    loadedSong = chrpFileName;
    System.out.println("Loaded CHRP: " + chrpFileName);
    return true;
  }

  public void play() {
    if (loadedSong == null) {
      System.out.println("No CHRP loaded, cannot play.");
      return;
    }
    System.out.println("Playing song: " + loadedSong);
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
