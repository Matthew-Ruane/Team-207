package frc.utility;

import edu.wpi.first.wpilibj.PIDOutput;
 
/** A dummy PID output that does not output to any hardware
 *
 * @author joe
 */
public class DummyPIDOutput implements PIDOutput{
    double output;
    
    public DummyPIDOutput()
    {
        output = 0;
    }

    public void pidWrite(double output) {
        this.output = output;
    }

    public double getOutput() {
        return output;
    }
    
}