package org.firstinspires.ftc.teamcode.PowerPlay11588.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RI3W 11588 Red Lower", group = "11588")
public class RI3W11588RedLower extends RI3W11588BaseAutonomous{
    ElapsedTime runTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        waitForStart();
        runTime.reset();
        while(runTime.milliseconds() < 2000){}
        /*
        if(spot1){
            move to spot 1
        }else if(spot2) {
            move to spot 2
        }else if(spot3){
            move to spot 3
        }
         */
        //if spot 1
        moveXY(0, -24);
        moveXY(36, 0);

        //if spot 2
        moveXY(36, 0);

        //if spot 3
        moveXY(0, 24);
        moveXY(36, 0);
    }
}
