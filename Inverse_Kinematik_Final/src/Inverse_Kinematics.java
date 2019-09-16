import Jama.Matrix;
import processing.core.PApplet;
import java.math.*;

public class Inverse_Kinematics {
	PApplet p;
	Jama.Matrix T01, T02, T03, T04, T05, T06, T0E, T6E, T0E7;
	double q1 = 0;
	double q2 = 0;
	double q3 = 0;
	double q4 = 0;
	double q5 = -Math.toRadians(0);
	double q6 = 0;
	double[][] qq = { { q1 }, { q2 }, { q3 }, { q4 }, { q5 }, { q6 } };
	double[][] null_vec6 = { { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 } };
	double speed_correction_factor = 1;

	Jama.Matrix q = new Jama.Matrix(qq);
	Jama.Matrix q_dot = new Jama.Matrix(null_vec6);
	Jama.Matrix error_int = new Jama.Matrix(null_vec6);

	Jama.Matrix Ep = new Jama.Matrix(null_vec6);
	Jama.Matrix Ei = new Jama.Matrix(null_vec6);

	double[][] toolframe = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	Jama.Matrix ToolFrame = new Jama.Matrix(toolframe);

	public Inverse_Kinematics(PApplet _p) {
		p = _p;
		forward_Kinematik(q);

		
	}

	public void differential_Kinematics(Quaternion Qd, Jama.Matrix Xd, double dt) {
		Jama.Matrix Re = getRotFromTrans(get_current_Trans());
		Jama.Matrix Pe = getPosFromTrans(get_current_Trans());
		Jama.Matrix Rd = Qd.getRotationmatrix();
		Jama.Matrix Pd = Qd.getPos();

		Jama.Matrix pe = Pd.minus(Pe);
		Jama.Matrix eRd = Re.transpose().times(Rd);
		Quaternion Qde = new Quaternion(eRd);
		Jama.Matrix ee_de = Qde.getVector();
		Jama.Matrix e_de = Re.times(ee_de);

		double kpp = 1;
		double kpo = 1;

		double[][] err = { { kpp * pe.get(0, 0) }, { kpp * pe.get(1, 0) }, { kpp * pe.get(2, 0) },
				{ kpo * e_de.get(0, 0) }, { kpo * e_de.get(1, 0) }, { kpo * e_de.get(2, 0) } };

		Jama.Matrix error = new Jama.Matrix(err);
		Jama.Matrix e_dot = Xd.minus(geometric_Jacobian(q).times(q_dot));
		error_int.plusEquals(error);
		double Kd = 20;
		double Ti = 250;
		double Td = 0.0001;
		Jama.Matrix U = error.plus(error_int.timesEquals(1/Ti).plus(e_dot.timesEquals(Td)));

		q_dot = correct_speed(pseudoInverse(geometric_Jacobian(q)).times(Xd.plus(U.timesEquals(Kd))));
		q = correct_angle(q);
		q_dot.timesEquals(speed_correction_factor);
		q = q.plus(q_dot.times(dt));
		//q_dot = correct_speed(q_dot);
		set_q(q);

	}
	public Jama.Matrix correct_angle(Jama.Matrix angle){
		if(angle.get(0, 0) > Parameter.q1M) {
			angle.set(0, 0, Parameter.q1M);
			q_dot.set(0, 0, 0);
			speed_correction_factor = 1;
		}
		if(angle.get(0, 0) < Parameter.q1m) {
			angle.set(0, 0, Parameter.q1m);
			q_dot.set(0, 0, 0);
			speed_correction_factor = 1;
		}
		
		if(angle.get(1, 0)> Parameter.q2M) {
			angle.set(1, 0, Parameter.q2M);
			q_dot.set(1, 0, 0);
			speed_correction_factor = 1;
		}
		if(angle.get(1, 0)<Parameter.q2m) {
			angle.set(1, 0, Parameter.q2m);
			q_dot.set(1, 0, 0);
			speed_correction_factor = 1;
		}
		
		if(angle.get(2, 0) > Parameter.q3M) {
			angle.set(2, 0, Parameter.q3M);
			q_dot.set(2, 0, 0);
			speed_correction_factor = 1;
		}
		if(angle.get(2, 0)<Parameter.q3m) {
			angle.set(2, 0, Parameter.q3m);
			q_dot.set(2, 0, 0);
			speed_correction_factor = 1;
		}
		
		if(angle.get(3, 0)> Parameter.q4M) {
			angle.set(3, 0, Parameter.q4M);
			q_dot.set(3, 0, 0);
			speed_correction_factor = 1;
			
		}
		if(angle.get(3, 0)<Parameter.q4m) {
			angle.set(3, 0, Parameter.q4m);
			q_dot.set(3, 0, 0);
			speed_correction_factor = 1;
		}
		
		if(angle.get(4, 0)> Parameter.q5M) {
			angle.set(4, 0, Parameter.q5M);
			q_dot.set(4, 0, 0);
			speed_correction_factor = 1;
		}
		if(angle.get(4, 0)<Parameter.q5m) {
			angle.set(4, 0, Parameter.q5m);
			q_dot.set(4, 0, 0);
			speed_correction_factor = 1;
		}
		
		if(angle.get(5, 0)> Parameter.q6M) {
			angle.set(5, 0, Parameter.q6M);
			q_dot.set(5, 0, 0);
			speed_correction_factor = 1;
		}
		if(angle.get(5, 0)<Parameter.q6m) {
			angle.set(5, 0, Parameter.q6m);
			q_dot.set(5, 0, 0);
			speed_correction_factor = 1;
		}
		return angle;
	}
	public Jama.Matrix  correct_speed(Jama.Matrix  speed){
		speed_correction_factor = 1;
		if(Math.abs(speed.get(0, 0))>Parameter.dq1M) {
			speed_correction_factor = Parameter.dq1M/Math.abs(speed.get(0, 0));
			//speed.timesEquals(speed_correction_factor);
		}
		if(Math.abs(speed.get(1, 0))>Parameter.dq2M) {
			speed_correction_factor = Parameter.dq2M/Math.abs(speed.get(1, 0));
			//speed.timesEquals(speed_correction_factor);
		}
		if(Math.abs(speed.get(2, 0))>Parameter.dq3M) {
			speed_correction_factor = Parameter.dq3M/Math.abs(speed.get(2, 0));
			//speed.timesEquals(speed_correction_factor);
		}
		if(Math.abs(speed.get(3, 0))>Parameter.dq4M) {
			speed_correction_factor = Parameter.dq4M/Math.abs(speed.get(3, 0));
			//speed.timesEquals(speed_correction_factor);
		}
		if(Math.abs(speed.get(4, 0))>Parameter.dq5M) {
			speed_correction_factor = Parameter.dq5M/Math.abs(speed.get(4, 0));
			//speed.timesEquals(speed_correction_factor);
		}
		if(Math.abs(speed.get(5, 0))>Parameter.dq6M) {
			speed_correction_factor = Parameter.dq6M/Math.abs(speed.get(5, 0));
			//speed.timesEquals(speed_correction_factor);
		}
		return speed;
	}
	
	public Jama.Matrix geometric_Jacobian(Jama.Matrix _q) {

		forward_Kinematik(_q);

		double[][] z = { { 0 }, { 0 }, { 1 } };
		Jama.Matrix jo1 = new Jama.Matrix(z);
		Jama.Matrix jo2 = getZFromTrans(T01);
		Jama.Matrix jo3 = getZFromTrans(T02);
		Jama.Matrix jo4 = getZFromTrans(T03);
		Jama.Matrix jo5 = getZFromTrans(T04);
		Jama.Matrix jo6 = getZFromTrans(T05);

		Jama.Matrix jp1 = cross(jo1, getPosFromTrans(T0E));
		Jama.Matrix jp2 = cross(jo2, getPosFromTrans(T0E).minusEquals(getPosFromTrans(T01)));
		Jama.Matrix jp3 = cross(jo3, getPosFromTrans(T0E).minusEquals(getPosFromTrans(T02)));
		Jama.Matrix jp4 = cross(jo4, getPosFromTrans(T0E).minusEquals(getPosFromTrans(T03)));
		Jama.Matrix jp5 = cross(jo5, getPosFromTrans(T0E).minusEquals(getPosFromTrans(T04)));
		Jama.Matrix jp6 = cross(jo6, getPosFromTrans(T0E).minusEquals(getPosFromTrans(T05)));

		double[][] jacobian = {
				{ jp1.get(0, 0), jp2.get(0, 0), jp3.get(0, 0), jp4.get(0, 0), jp5.get(0, 0), jp6.get(0, 0) },
				{ jp1.get(1, 0), jp2.get(1, 0), jp3.get(1, 0), jp4.get(1, 0), jp5.get(1, 0), jp6.get(1, 0) },
				{ jp1.get(2, 0), jp2.get(2, 0), jp3.get(2, 0), jp4.get(2, 0), jp5.get(2, 0), jp6.get(2, 0) },
				{ jo1.get(0, 0), jo2.get(0, 0), jo3.get(0, 0), jo4.get(0, 0), jo5.get(0, 0), jo6.get(0, 0) },
				{ jo1.get(1, 0), jo2.get(1, 0), jo3.get(1, 0), jo4.get(1, 0), jo5.get(1, 0), jo6.get(1, 0) },
				{ jo1.get(2, 0), jo2.get(2, 0), jo3.get(2, 0), jo4.get(2, 0), jo5.get(2, 0), jo6.get(2, 0) } };

		Jama.Matrix J = new Jama.Matrix(jacobian);

		return J;
	}



	public Jama.Matrix pseudoInverse(Jama.Matrix J) {

		Jama.Matrix I = Jama.Matrix.identity(6, 6);
		double xi = 0.01;
		Jama.Matrix W = I.times(xi);
		Jama.Matrix JT = J.transpose();
		Jama.Matrix JJT = J.times(JT);
		Jama.Matrix JJTW = JJT.plus(W);
		Jama.Matrix JJT_inv = JJTW.inverse();

		Jama.Matrix J_pseudo = JT.times(JJT_inv);
		return J_pseudo;
	}

	
	void set_q(Jama.Matrix _q) {

		q1 = (float) _q.get(0, 0);
		q2 = (float) _q.get(1, 0);
		q3 = (float) _q.get(2, 0);
		q4 = (float) _q.get(3, 0);
		q5 = (float) _q.get(4, 0);
		q6 = (float) _q.get(5, 0);
	}
	
	public Jama.Matrix q7toq6(Jama.Matrix _q7){
		return _q7.getMatrix(0,5,0,0);
	}


	public Jama.Matrix transformationMatrix(double d, double a, double alpha, double theta) {

		double[][] Trans = { { c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta) },
				{ s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), a * s(theta) }, { 0, s(alpha), c(alpha), d },
				{ 0, 0, 0, 1 } };
		Jama.Matrix T = new Jama.Matrix(Trans);
		return T;
	}
	
	public Jama.Matrix path_trough_Jointspace(Jama.Matrix qs, Jama.Matrix qe, double Time, double t) {
		double s = timeScaling_5p(1,Time,t);
		Jama.Matrix dq = qe.minus(qs);
		//p.println((float)dq.get(0,0),(float)dq.get(1,0),(float)dq.get(2,0),(float)dq.get(3,0),(float)dq.get(4,0),(float)dq.get(5,0)); 
		Jama.Matrix q_interpolated = qs.plus(dq.times(s));

		
		return q_interpolated;
	}
	public Quaternion circular_Path(Jama.Matrix Ts, Jama.Matrix Tm, Jama.Matrix Te, double Time, double t) {

		double s = timeScaling_5p(1, Time, t);

		Jama.Matrix Ps = getPosFromTrans(Ts);
		Jama.Matrix Pm = getPosFromTrans(Tm);
		Jama.Matrix Pe = getPosFromTrans(Te);

		Jama.Matrix rms = Pm.minus(Ps);
		Jama.Matrix res = Pe.minus(Ps);
		Jama.Matrix rem = Pe.minus(Pm);
		Jama.Matrix c = cross(rms, res);
		Jama.Matrix e_z = c.times(1 / c.norm2());
		Jama.Matrix rH1 = cross(e_z, rms);
		Jama.Matrix eH1 = rH1.times(1 / rH1.norm2());

		Jama.Matrix rH2 = cross(res, e_z);
		Jama.Matrix eH2 = rH2.times(1 / rH2.norm2());

		double h1_n1 = eH1.get(2, 0) * eH2.get(1, 0) - eH1.get(1, 0) * eH2.get(2, 0);
		double h1_n2 = eH1.get(2, 0) * eH2.get(0, 0) - eH1.get(0, 0) * eH2.get(2, 0);
		double h1_n3 = eH1.get(1, 0) * eH2.get(0, 0) - eH1.get(0, 0) * eH2.get(1, 0);

		double h1 = 0;
		if (Math.abs(h1_n1) >= 0.001) {
			h1 = 0.5 * (rem.get(2, 0) * eH2.get(1, 0) - rem.get(1, 0) * eH2.get(2, 0)) / h1_n1;
		} else {
			if (Math.abs(h1_n2) >= 0.001) {
				h1 = 0.5 * (rem.get(2, 0) * eH2.get(0, 0) - rem.get(1, 0) * eH2.get(2, 0)) / h1_n2;
			} else {
				h1 = 0.5 * (rem.get(1, 0) * eH2.get(0, 0) - rem.get(0, 0) * eH2.get(1, 0)) / h1_n3;
			}

		}
		Jama.Matrix Pc = Ps.plus(rms.times(0.5).plus(eH1.times(h1)));

		Jama.Matrix rsc = Ps.minus(Pc);
		Jama.Matrix e_x = rsc.times(1 / rsc.norm2());
		Jama.Matrix e_y = cross(e_z, e_x);

		double[][] Rc = { { e_x.get(0, 0), e_y.get(0, 0), e_z.get(0, 0) },
				{ e_x.get(1, 0), e_y.get(1, 0), e_z.get(1, 0) }, { e_x.get(2, 0), e_y.get(2, 0), e_z.get(2, 0) } };

		Jama.Matrix Rcc = new Jama.Matrix(Rc);

		// show center Koordinatensystem if needed

//		double size = 100;
//		p.strokeWeight(1);
//		p.stroke(255, 0, 0);
//		p.line((float) Pc.get(0, 0), (float) Pc.get(1, 0), (float) Pc.get(2, 0),
//				(float) (Pc.get(0, 0) + size * Rcc.get(0, 0)), (float) (Pc.get(1, 0) + size * Rcc.get(1, 0)),
//				(float) (Pc.get(2, 0) + size * Rcc.get(2, 0)));
//		p.stroke(0, 255, 0);
//		p.line((float) Pc.get(0, 0), (float) Pc.get(1, 0), (float) Pc.get(2, 0),
//				(float) (Pc.get(0, 0) + size * Rcc.get(0, 1)), (float) (Pc.get(1, 0) + size * Rcc.get(1, 1)),
//				(float) (Pc.get(2, 0) + size * Rcc.get(2, 1)));
//		p.stroke(0, 0, 255);
//		p.line((float) Pc.get(0, 0), (float) Pc.get(1, 0), (float) Pc.get(2, 0),
//				(float) (Pc.get(0, 0) + size * Rcc.get(0, 2)), (float) (Pc.get(1, 0) + size * Rcc.get(1, 2)),
//				(float) (Pc.get(2, 0) + size * Rcc.get(2, 2)));

		Jama.Matrix rec = Pe.minus(Pc);
		double radius = rsc.norm2();
		double dot = rsc.get(0, 0) * rec.get(0, 0) + rsc.get(1, 0) * rec.get(1, 0) + rsc.get(2, 0) * rec.get(2, 0);
		double det = rsc.get(0, 0) * rec.get(1, 0) * e_z.get(2, 0) + rec.get(0, 0) * e_z.get(1, 0) * rsc.get(2, 0)
				+ e_z.get(0, 0) * rsc.get(1, 0) * rec.get(2, 0) - rsc.get(2, 0) * rec.get(1, 0) * e_z.get(0, 0)
				- rec.get(2, 0) * e_z.get(1, 0) * rsc.get(0, 0) - e_z.get(2, 0) * rsc.get(1, 0) * rec.get(0, 0);
		Jama.Matrix check_Pe = Rcc.transpose().times(Pe);
		Jama.Matrix check_Pm = Rcc.transpose().times(Pc);
		check_Pe.minusEquals(check_Pm);

		double k;
		if (check_Pe.get(1, 0) < 0) {
			k = 2 * Parameter.pi + Math.atan2(det, dot);
		} else {
			k = Math.atan2(det, dot);
		}

		double[][] rp = { { radius * c(k * s) }, { radius * s(k * s) }, { 0 } };

		Jama.Matrix rpp = new Jama.Matrix(rp);
		Jama.Matrix P = Pc.plus(Rcc.times(rpp));

		Quaternion Qs = new Quaternion(Ts);
		Quaternion Qe = new Quaternion(Te);
		Jama.Matrix R = slerp(Qs, Qe, s).getRotationmatrix();

		double[][] trans = { 
				{ R.get(0, 0), R.get(0, 1), R.get(0, 2), P.get(0, 0) },
				{ R.get(1, 0), R.get(1, 1), R.get(1, 2), P.get(1, 0) },
				{ R.get(2, 0), R.get(2, 1), R.get(2, 2), P.get(2, 0) },
				{ 0, 0, 0, 1 } };

		Jama.Matrix Trans = new Jama.Matrix(trans);
		Quaternion Qres = new Quaternion(Trans);
		return Qres;
	}

	public Quaternion straight_Line_Path_dec(Jama.Matrix Ts, Jama.Matrix Te, double Time, double t) {

		double s = timeScaling_5p(1, Time, t);
		Jama.Matrix Pstart = getPosFromTrans(Ts);
		Jama.Matrix Pend = getPosFromTrans(Te);

		Quaternion Qs = new Quaternion(Ts);
		Quaternion Qe = new Quaternion(Te);
		Quaternion R = slerp(Qs, Qe, s);

		Jama.Matrix dp = Pend.minus(Pstart);
		// dp.print(3, 1);
		Jama.Matrix sdp = dp.times(s);
		Jama.Matrix P = Pstart.plus(sdp);

		Quaternion Res = new Quaternion(R.n, R.ex, R.ey, R.ez, P.get(0, 0), P.get(1, 0), P.get(2, 0));
		return Res;
	}

	public Quaternion slerp(Quaternion qa, Quaternion qb, double t) {
		Quaternion qm = new Quaternion(1, 0, 0, 0);
		double cosHalfTheta = qa.n * qb.n + qa.ex * qb.ex + qa.ey * qb.ey + qa.ez * qb.ez;
		if (cosHalfTheta < 0) {
			qb.n = -qb.n;
			qb.ex = -qb.ex;
			qb.ey = -qb.ey;
			qb.ez = -qb.ez;
			cosHalfTheta = -cosHalfTheta;
		}

		if (Math.abs(cosHalfTheta) >= 1.0) {
			qm.n = qa.n;
			qm.ex = qa.ex;
			qm.ey = qa.ey;
			qm.ez = qa.ez;
			return qm;
		}

		double halfTheta = Math.acos(cosHalfTheta);
		double sinHalfTheta = Math.sqrt(1.0 - cosHalfTheta * cosHalfTheta);

		if (Math.abs(sinHalfTheta) < 0.001) { // fabs is floating point absolute
			qm.n = (qa.n * 0.5 + qb.n * 0.5);
			qm.ex = (qa.ex * 0.5 + qb.ex * 0.5);
			qm.ey = (qa.ey * 0.5 + qb.ey * 0.5);
			qm.ez = (qa.ez * 0.5 + qb.ez * 0.5);
			return qm;
		}
		double ratioA = Math.sin((1 - t) * halfTheta) / sinHalfTheta;
		double ratioB = Math.sin(t * halfTheta) / sinHalfTheta;
		// calculate Quaternion.
		qm.n = (qa.n * ratioA + qb.n * ratioB);
		qm.ex = (qa.ex * ratioA + qb.ex * ratioB);
		qm.ey = (qa.ey * ratioA + qb.ey * ratioB);
		qm.ez = (qa.ez * ratioA + qb.ez * ratioB);
		return qm;
	}

	public double timeScaling_5p(float dist, double T, double t) {
		double f1 = 6 / (T * T * T * T * T);
		double f2 = -15 / (T * T * T * T);
		double f3 = 10 / (T * T * T);
		if (t < 0)
			t = 0;
		if (t > T)
			t = T;
		double t5 = t * t * t * t * t;
		double t4 = t * t * t * t;
		double t3 = t * t * t;

		double s = f1 * dist * t5 + f2 * dist * t4 + f3 * dist * t3;
		return s;
	}

	public Jama.Matrix cross(Jama.Matrix A, Jama.Matrix B) {
		double[][] res = { { A.get(1, 0) * B.get(2, 0) - A.get(2, 0) * B.get(1, 0) },
				{ A.get(2, 0) * B.get(0, 0) - A.get(0, 0) * B.get(2, 0) },
				{ A.get(0, 0) * B.get(1, 0) - A.get(1, 0) * B.get(0, 0) } };

		Jama.Matrix Res = new Jama.Matrix(res);

		return Res;
	}

	public Jama.Matrix getRotFromTrans(Jama.Matrix Trans) {
		Jama.Matrix R = Trans.getMatrix(0, 2, 0, 2);
		return R;
	}

	public Jama.Matrix getPosFromTrans(Jama.Matrix Trans) {
		Jama.Matrix P = Trans.getMatrix(0, 2, 3, 3);
		return P;
	}

	public Jama.Matrix getZFromTrans(Jama.Matrix Trans) {
		Jama.Matrix P = Trans.getMatrix(0, 2, 2, 2);
		return P;
	}

	private double s(double p) {
		return Math.sin(p);
	}

	private double c(double p) {
		return Math.cos(p);
	}

	public Jama.Matrix T_inv(Jama.Matrix T) {

		Jama.Matrix R = T.getMatrix(0, 2, 0, 2);
		Jama.Matrix RT = R.transpose();
		Jama.Matrix P = T.getMatrix(0, 2, 3, 3);
		Jama.Matrix RTP = RT.times(P.times(-1));

		double[][] ti = { { RT.get(0, 0), RT.get(0, 1), RT.get(0, 2), RTP.get(0, 0) },
							{ RT.get(1, 0), RT.get(1, 1), RT.get(1, 2), RTP.get(1, 0) },
							{ RT.get(2, 0), RT.get(2, 1), RT.get(2, 2), RTP.get(2, 0) },
							{ 0, 0, 0, 1 } };
		
		Jama.Matrix Ti = new Jama.Matrix(ti);

		return Ti;
	}

	public Jama.Matrix get_qVec() {
		double[][] qp = { { q1 }, { q2 }, { q3 }, { q4 }, { q5 }, { q6 } };
		Jama.Matrix Q = new Jama.Matrix(qp);

		return Q;
	}

	public Jama.Matrix get_current_Trans() {
		return T0E;
	}

	public void forward_Kinematik(Jama.Matrix _q) {
		T01 = transformationMatrix(Parameter.d1, Parameter.a1, Parameter.alpha1, Parameter.theta1 + _q.get(0, 0));
		Jama.Matrix T12 = transformationMatrix(Parameter.d2, Parameter.a2, Parameter.alpha2,
				Parameter.theta2 + _q.get(1, 0));
		Jama.Matrix T23 = transformationMatrix(Parameter.d3, Parameter.a3, Parameter.alpha3,
				Parameter.theta3 + _q.get(2, 0));
		Jama.Matrix T34 = transformationMatrix(Parameter.d4, Parameter.a4, Parameter.alpha4,
				Parameter.theta4 + _q.get(3, 0));
		Jama.Matrix T45 = transformationMatrix(Parameter.d5, Parameter.a5, Parameter.alpha5,
				Parameter.theta5 + _q.get(4, 0));
		Jama.Matrix T56 = transformationMatrix(Parameter.d6, Parameter.a6, Parameter.alpha6,
				Parameter.theta6 + _q.get(5, 0));
		T6E = new Jama.Matrix(toolframe);
		T02 = T01.times(T12);
		T03 = T02.times(T23);
		T04 = T03.times(T34);
		T05 = T04.times(T45);
		T06 = T05.times(T56);
		T0E = T06.times(T6E);
		
		//Jama.Matrix test = T06.times(T_inv(T06));
		//test.print(4, 4);
	}

//	public void forward_Kinematik(double q1, double q2, double q3, double q4, double q5, double q6) {
//		T01 = transformationMatrix(Parameter.d1, Parameter.a1, Parameter.alpha1, Parameter.theta1 + q1);
//		Jama.Matrix T12 = transformationMatrix(Parameter.d2, Parameter.a2, Parameter.alpha2, Parameter.theta2 + q2);
//		Jama.Matrix T23 = transformationMatrix(Parameter.d3, Parameter.a3, Parameter.alpha3, Parameter.theta3 + q3);
//		Jama.Matrix T34 = transformationMatrix(Parameter.d4, Parameter.a4, Parameter.alpha4, Parameter.theta4 + q4);
//		Jama.Matrix T45 = transformationMatrix(Parameter.d5, Parameter.a5, Parameter.alpha5, Parameter.theta5 + q5);
//		Jama.Matrix T56 = transformationMatrix(Parameter.d6, Parameter.a6, Parameter.alpha6, Parameter.theta6 + q6);
//
//		
//		T02 = T01.times(T12);
//		T03 = T02.times(T23);
//		T04 = T03.times(T34);
//		T05 = T04.times(T45);
//		T06 = T05.times(T56);
//		T0E = T06.times(T6E);
//	}

	void showKS(int jointNumber) {

		Jama.Matrix getKS = Jama.Matrix.identity(4, 4);

		switch (jointNumber) {

		case 0:
			break;

		case 1:
			getKS = T01;
			break;

		case 2:
			getKS = T02;
			break;

		case 3:
			getKS = T03;
			break;

		case 4:
			getKS = T04;
			break;

		case 5:
			getKS = T05;
			break;

		case 6:
			getKS = T06;
			break;

		case 7:
			getKS = T0E;
			break;
			
		case 8:
			getKS = T0E7;
			break;
		}

		float mx = (float) getKS.get(0, 0);
		float my = (float) getKS.get(1, 0);
		float mz = (float) getKS.get(2, 0);
		float lx = (float) getKS.get(0, 1);
		float ly = (float) getKS.get(1, 1);
		float lz = (float) getKS.get(2, 1);
		float nx = (float) getKS.get(0, 2);
		float ny = (float) getKS.get(1, 2);
		float nz = (float) getKS.get(2, 2);
		float x = (float) getKS.get(0, 3);
		float y = (float) getKS.get(1, 3);
		float z = (float) getKS.get(2, 3);
		// p.println(x,y,z);
		float size = 80;

		p.pushMatrix();
		p.strokeWeight(2);
		p.stroke(255, 0, 0);
		p.line(x, y, z, x + mx * size, y + my * size, z + mz * size);
		p.stroke(0, 255, 0);
		p.line(x, y, z, x + lx * size, y + ly * size, z + lz * size);
		p.stroke(0, 0, 255);
		p.line(x, y, z, x + nx * size, y + ny * size, z + nz * size);
		p.popMatrix();
	}

	public static class Parameter {

		static final double pi = 3.14159265359;
		static final double half_pi = 1.57079632679;

		static final double d1 = 160;
		static final double d2 = 0;
		static final double d3 = 0;
		static final double d4 = 285.8;
		static final double d5 = 0;
		static final double d6 = 51.5;

		static final double a1 = 90;
		static final double a2 = 300;
		static final double a3 = 56.5;
		static final double a4 = 0;
		static final double a5 = 0;
		static final double a6 = 0;

		static final double alpha1 = half_pi;
		static final double alpha2 = 0;
		static final double alpha3 = half_pi;
		static final double alpha4 = half_pi;
		static final double alpha5 = -half_pi;
		static final double alpha6 = 0;

		static final double theta1 = 0;
		static final double theta2 = half_pi;
		static final double theta3 = 0;
		static final double theta4 = 0;
		static final double theta5 = 0;
		static final double theta6 = -half_pi;
		
		static final double dq1M = pi;
		static final double dq2M = pi;
		static final double dq3M = pi;
		static final double dq4M = pi;
		static final double dq5M = pi;
		static final double dq6M = pi;

		static final double q1m = -pi;
		static final double q1M = pi;

		static final double q2m = -pi*0.5;
		static final double q2M = pi*0.5;

		static final double q3m = -pi;
		static final double q3M = pi*0.4;

		static final double q4m = -2*pi;
		static final double q4M = 2*pi;

		static final double q5m = -pi*0.7;
		static final double q5M = pi*0.7;

		static final double q6m = -4 * pi;
		static final double q6M = 4 * pi;
	}
}
