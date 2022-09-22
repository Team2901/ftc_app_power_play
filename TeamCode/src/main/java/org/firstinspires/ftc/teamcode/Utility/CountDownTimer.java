package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CountDownTimer {
  ElapsedTime elapsedTime;
  int targetTime;

  public CountDownTimer(ElapsedTime.Resolution resolution){
    elapsedTime = new ElapsedTime(resolution);
  }
  public void setTargetTime(int targetTime){
    elapsedTime.reset();
    this.targetTime = targetTime;
  }

  public double getRemainingTime(){
    double currentTime = elapsedTime.time();
    double remainingTime = targetTime - currentTime;
    if(remainingTime < 0){
      return 0;
    }
    return remainingTime;
  }

  public boolean hasRemainingTime(){
    if(getRemainingTime() > 0){
      return true;
    } else {
      return false;
    }
  }
}
