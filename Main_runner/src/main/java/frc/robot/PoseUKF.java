package frc.robot;

import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpiutil.math.numbers.*;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Pair;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.estimator.MerweScaledSigmaPoints;
import static frc.robot.Constants.ROBOT_DATA.*;

public class PoseUKF {
    ChassisSpeeds chassisSpeeds;
    
    Matrix<N3,N1> state = new Matrix<>(Nat.N3(), Nat.N1()); // for x, y, theta
    
    Matrix<N3,N3> p_0 = new Matrix<>(Nat.N3(), Nat.N3());
    Matrix<N2,N2> p_z = new Matrix<>(Nat.N2(), Nat.N2());
    Matrix<N3,N2> p_xz = new Matrix<>(Nat.N3(), Nat.N2());
    
    Matrix<N2,N1> measurement_matrix = new Matrix<>(Nat.N2(), Nat.N1());
    Matrix<N2,N1> h_function = new Matrix<>(Nat.N2(), Nat.N1());
    
    Matrix<N3,N1> state_standard_deviation = new Matrix<>(Nat.N3(), Nat.N1());
    Matrix<N2,N1> output_standard_deviation = new Matrix<>(Nat.N2(), Nat.N1());

    Matrix<N3,N3> m_Q;
    Matrix<N2,N2> m_r;

    MerweScaledSigmaPoints<N3> m_set = new MerweScaledSigmaPoints<>(Nat.N3(),0.5,2,0);

    MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    m_frontLeft, m_frontRight, m_backLeft, m_backRight);
    
    double dt;

    public PoseUKF(double t, double x_t, double y_t, double th_t, double r_t, double yaw_t) {
        dt = t;
        state_standard_deviation.set(0,0,x_t);
        state_standard_deviation.set(1,0,y_t);
        state_standard_deviation.set(2,0,th_t);

        output_standard_deviation.set(0,0,r_t);
        output_standard_deviation.set(1,0,yaw_t);

        h_function.set(0,0,1);
        h_function.set(1,1,1);
        h_function.set(2,2,1);

        m_Q = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), state_standard_deviation);
        m_r = StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), output_standard_deviation);
    }

    // Pre-calculation
    public void convertMotortoRobotframe(double v_1, double v_2, double v_3, double v_4) {
        chassisSpeeds = kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(v_1, v_2, v_3, v_4));
    }

    public double robotFrameVx() {
        return chassisSpeeds.vxMetersPerSecond;
    }

    public double robotFrameVy() {
        return chassisSpeeds.vyMetersPerSecond;
    }

    public double robotFrameThetarate() {
        return chassisSpeeds.omegaRadiansPerSecond;
    }

    public void collectMeasurement(double target_range, double target_yaw) {
        measurement_matrix.set(0,0,target_range);
        measurement_matrix.set(1,0,target_yaw);
    }

    // limit the range of angle in -pi to pi
    public double angle_normalize(double angle) { 
        angle = angle %(2*Math.PI);
        if (angle > Math.PI){
            angle -= Math.PI;
        }
        return angle;
    }

    // Calculate Sigma point, Wm, Wc
    public Matrix<N3, ?> calculateSigmaPoint() {
        return m_set.sigmaPoints(state, p_0);
    }

    public Matrix<?,N1> calculateWm() {
        return m_set.getWm();
    }

    public Matrix<?,N1> calculateWc() {
        return m_set.getWc();
    }

    public Matrix<N3,N1> extractSigmaPoint(int y) {
        return calculateSigmaPoint().extractColumnVector(y);
    }

    // Our system matrix
    public Matrix<N3,N3> CosMatrix(double theta) {
        Matrix<N3,N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        matrix.set(0,0,Math.cos(theta));
        matrix.set(0,1,-Math.sin(theta));
        matrix.set(1,0,Math.sin(theta));
        matrix.set(1,1,Math.cos(theta));
        matrix.set(2,2,1);
        return matrix;
    }

    public Matrix<N3,N3> CosTMatrix() {
        Matrix<N3,N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        if (robotFrameThetarate() == 0) {
            matrix.set(0,0,dt);
            matrix.set(1,1,dt);
            matrix.set(2,2,dt);
        }
        else{
            matrix.set(0,0,Math.sin(dt*robotFrameThetarate())/robotFrameThetarate());
            matrix.set(0,1,(Math.cos(dt*robotFrameThetarate())-1)/robotFrameThetarate());
            matrix.set(0,1,(1-Math.cos(dt*robotFrameThetarate()))/robotFrameThetarate());
            matrix.set(1,1,Math.sin(dt*robotFrameThetarate())/robotFrameThetarate());
        }
        return matrix;
    }

    public Matrix<N3,N1> ControlInput() {
        Matrix<N3,N1> control_input = new Matrix<>(Nat.N3(), Nat.N1());
        control_input.set(0,0,robotFrameVx());
        control_input.set(1,0,robotFrameVy());
        control_input.set(2,0,robotFrameThetarate());
        return control_input;
    }

    //The system matrix x = x + deltax 
    public Matrix<N3,N1> PredictY(int y) {
        Matrix<N3,N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
        Matrix<N3,N1> sigma_point = extractSigmaPoint(y);
        
        matrix = (CosMatrix(sigma_point.get(2, 0)).times(
        CosTMatrix()).times(ControlInput()));
        matrix = sigma_point.plus(matrix);
        return matrix;
    }

    public double[] measurement_calculator(double target_x, double target_y, double delta_0, double delta_1, double delta_2) {
        double p2_delta_x = Math.pow(target_x-delta_0,2);
        double p2_delta_y = Math.pow(target_y-delta_1,2);
        double range = Math.sqrt(p2_delta_x+p2_delta_y);
        double delta_theta = Math.atan((target_y-delta_0)/
        (target_x-delta_1))-delta_2;
        double[] x = new double[2];
        x[0] = range;
        x[1] = delta_theta;
        return x;
    }

    public void H_function(Matrix<N3,N1> y) {
        // double range;
        // double delta_theta;
        //if ( target x in the camera) {
        //   double[]x = measurement_calculator(target_x, target_y, y.get(0,0), y.get(1,0), y.get(2,0));
        //   range = x[0];
        //   delta_theta = x[1];
        //}
        //else if (target y in the camera) {
        //   double[]x = measurement_calculator(target_x, target_y, y.get(0,0), y.get(1,0), y.get(2,0));
        //   range = x[0];
        //   delta_theta = x[1];
        //}
        //else if (target z in the camera) {
        //    double[]x = measurement_calculator(target_x, target_y, y.get(0,0), y.get(1,0), y.get(2,0));
        //   range = x[0];
        //   delta_theta = x[1];
        //}
        //h_function.set(0,0,range);
        //h_function.set(1,0,delta_theta);
    }

    double numRow = calculateSigmaPoint().getNumRows();

    public Matrix<N3,N1> PredictX() {
        Matrix<N3,N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
        double sin_sum = 0;
        double cos_sum = 0;
        double fir_row = 0;
        double sec_row = 0;
        for(int i=0; i<=numRow; i++) {
            fir_row += PredictY(i).get(0,0)*(calculateWm().get(i,0));
            sec_row += PredictY(i).get(0,0)*(calculateWm().get(i,0));
            sin_sum += Math.sin(PredictY(i).get(2,0))*(calculateWm().get(i,0));
            cos_sum += Math.cos(PredictY(i).get(2,0))*(calculateWm().get(i,0));
        }
        double mean_theta = angle_normalize(Math.atan2(sin_sum, cos_sum));
        matrix.set(0,0,fir_row);
        matrix.set(1,0,sec_row);
        matrix.set(2,0,mean_theta);
        return matrix;
    }

    public void PredictP() {
        Matrix<N3,N3> zeroth = Matrix.eye(Nat.N3());
        for (int i=0; i<=numRow; i++) {
            var delta_0 = PredictY(i).minus(PredictX());
            var delta_1 = (PredictY(i).minus(PredictX())).transpose();
            zeroth = zeroth.plus(delta_0.times(delta_1).times(calculateWc().get(i,0)));
        }
        p_0 = zeroth.plus(m_Q);
    }

    // Update
    public Matrix<N2,N1> ZMatrix(int y) {
        H_function(PredictY(y));
        Matrix<N2,N1> matrix = new Matrix<>(Nat.N2(), Nat.N1());
        matrix = h_function;
        return matrix;
    }
    
    public Matrix<N2,N1> Meanz() {
        Matrix<N2,N1> matrix = new Matrix<>(Nat.N2(), Nat.N1());
        double range = 0;
        double sum_sin = 0;
        double sum_cos = 0;
        for (int i = 0; i<=numRow; i++) {
            range += ZMatrix(i).get(0,0)* calculateWm().get(i,0);
            sum_sin += Math.sin(ZMatrix(i).get(1,0))*calculateWm().get(i,0);
            sum_cos += Math.cos(ZMatrix(i).get(1,0))*calculateWm().get(i,0);
        }
        double theta = angle_normalize(Math.atan2(sum_sin, sum_cos)) ;
        matrix.set(0,0,range);
        matrix.set(1,0,theta);
        return matrix;
    }

    public void Pz() {
        for (int i=0; i<=numRow; i++) {
            var delta_0 = ZMatrix(i).minus(Meanz());
            var delta_1 = (ZMatrix(i).minus(Meanz())).transpose();
            p_z = p_z.plus(delta_0.times(delta_1).times(calculateWc().get(i,0)));
        }
        p_z = p_z.plus(m_r);
    }

    public void Pxz() {
        for (int i=0; i<=numRow; i++) {
            var delta_0 = PredictY(i).minus(PredictX());
            var delta_1 = 
            (ZMatrix(i).minus(Meanz())).transpose();
            p_xz = p_xz.plus(delta_0.times(delta_1).times(calculateWc().get(i,0)));
        }
    }

    public Pair<Matrix<N3,N1>, Matrix<N3,N3>> calculate() {
        PredictP();
        Pz();
        Pxz();

        //var k_gain = p_xz.times(p_z.inv());
        var k_gain = (p_z.transpose().solve(p_xz.transpose())).transpose();
        var y = measurement_matrix.minus(Meanz());
        y.set(2,0,angle_normalize(y.get(2,0)));
        
        state = state.plus(k_gain.times(y));
        p_0 = p_0.minus(k_gain.times(p_z).times(k_gain.transpose()));
        return new Pair<>(state, p_0);
    }
}
