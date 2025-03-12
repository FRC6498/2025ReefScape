package frc.robot;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.PWM;

public class LinearActuator extends PWM {

    private double lengthmm;
    
    public LinearActuator(int channel, Distance length){
        super(channel);

        lengthmm = length.in(Millimeter);
    }
    
    public LinearActuator(int channel, double length){
        super(channel);

        lengthmm = length;
    }

    public void setLength(double setPos) {
        super.setSpeed( setPos / lengthmm * 2 - 1);
    }

    public void setLength(Distance setPos) {
        setLength(setPos.in(Millimeters));
    }

}
