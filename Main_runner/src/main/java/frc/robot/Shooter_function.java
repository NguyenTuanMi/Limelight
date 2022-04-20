package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

public class Shooter_function {
    public static double[] calculate(double distance) throws IOException {
        int i = 0;
        
        double motor_rpm = 0;
        double angle = 0;
        double velocity = 0;
        double spin = 0;
        double time = 0;
        
        List<String> list = Files.readAllLines(Paths.get("data_raw.txt"));
        String line_1;
        String line_2;
        
        while (i < 306) {
            line_1 = list.get(i);
            line_2 = list.get(i+6);
            double k = Double.parseDouble(line_1);
            double q = Double.parseDouble(line_2);
            
            if (distance == k) {
                motor_rpm = Double.parseDouble(list.get(i+1)); 
                angle = Double.parseDouble(list.get(i+2));
                velocity = Double.parseDouble(list.get(i+3));
                spin = Double.parseDouble(list.get(i+4));
                time = Double.parseDouble(list.get(i+5));
                break;
            }
            
            else if (distance == q) {
                motor_rpm = Double.parseDouble(list.get(i+7)); 
                angle = Double.parseDouble(list.get(i+8));
                velocity = Double.parseDouble(list.get(i+9));
                spin = Double.parseDouble(list.get(i+10));
                time = Double.parseDouble(list.get(i+11));
                break;
            }
            
            else if (distance > k && distance < q) {
                motor_rpm = (Double.parseDouble(list.get(i+1)) + Double.parseDouble(list.get(i+7)))/2;
                angle = (Double.parseDouble(list.get(i+2)) + Double.parseDouble(list.get(i+8)))/2;
                velocity = (Double.parseDouble(list.get(i+3)) + Double.parseDouble(list.get(i+9)))/2;
                spin = (Double.parseDouble(list.get(i+4)) + Double.parseDouble(list.get(i+10)))/2;
                time = (Double.parseDouble(list.get(i+5)) + Double.parseDouble(list.get(i+11)))/2;
                break;
            }
            
            i += 6;
        }
        
        return new double[]{motor_rpm, angle, velocity, spin, time};
    }
}
