import processing.core.PApplet;
import processing.data.Table;
import processing.data.TableRow;

public class Trajectory {

	Table Traj;
	PApplet p;
	double dt_soll = 0.015;
	double v_soll = 50;
	double omega_max = 1;

	public Trajectory(PApplet _p) {
		p = _p;
		Traj = new Table();
		for (int i = 0; i < 13; i++) {
			Traj.addColumn();
		}
	}

	public Jama.Matrix g_code_interpolation(Table _list) {

		for (int i = 0; i < _list.getRowCount() - 1; i++) {
			Jama.Matrix T1 = getTransFromList(_list, i);
			Jama.Matrix T2 = getTransFromList(_list, i + i);
			Jama.Matrix P1 = T1.getMatrix(0, 2, 3, 3);
			Jama.Matrix P2 = T1.getMatrix(0, 2, 3, 3);
			Jama.Matrix R1 = T1.getMatrix(0, 2, 0, 2);
			Jama.Matrix R2 = T1.getMatrix(0, 2, 0, 2);
			double dx = P2.get(0, 0) - P1.get(0, 0);
			double dy = P2.get(1, 0) - P1.get(1, 0);
			double dz = P2.get(2, 0) - P1.get(2, 0);

			double l = Math.sqrt(dx * dx + dy * dy + dz * dz);

			double Time = l / v_soll;

		}
		return null;

	}

	public void createTraj(Table _list) {

		for (int i = 0; i < (_list.getRowCount() - 1); i++) {
			Jama.Matrix T1 = getTransFromList(_list, i);
			Jama.Matrix T2 = getTransFromList(_list, i + 1);
			Jama.Matrix P1 = T1.getMatrix(0, 2, 3, 3);
			Jama.Matrix P2 = T2.getMatrix(0, 2, 3, 3);
			Jama.Matrix R1 = T1.getMatrix(0, 2, 0, 2);
			Jama.Matrix R2 = T2.getMatrix(0, 2, 0, 2);

			Jama.Matrix Rres = R2.times(R1.transpose());
			Quaternion Qres = new Quaternion(Rres);
			double Qlen = Qres.getVector().norm2();
			double theta = Math.atan2(Qlen, Qres.n);

			double dx = P2.get(0, 0) - P1.get(0, 0);
			double dy = P2.get(1, 0) - P1.get(1, 0);
			double dz = P2.get(2, 0) - P1.get(2, 0);
			double l = Math.sqrt(dx * dx + dy * dy + dz * dz);

			double dt = l / v_soll;

			Quaternion Q1 = new Quaternion(R1);
			Quaternion Q2 = new Quaternion(R2);

			Quaternion Q_dot = Q2.minus(Q1);
			Q_dot.times(2 / dt);
			Quaternion w = Q_dot.times(Q1.inverse());
			Jama.Matrix W = w.getVector();
			double omega = W.norm2();
			
			if (omega > omega_max) {
				double dt_neu = theta / omega_max;
				if (dt_neu > dt)
					dt = dt_neu;
			}

			if (dt > dt_soll) {
				double inc =  (dt / dt_soll);
				// p.println(inc);
				for (int j = 1; j < inc+1; j++) {
					//p.println(dt, j * dt_soll);

					double s = s(dt, j * dt_soll);

					Jama.Matrix P_int = P1.plus((P2.minus(P1)).times(s));
					Quaternion Q_int = slerp(Q1, Q2, s);
					p.println(Q_int.n);
					Jama.Matrix R_int = Q_int.getRotationmatrix();
					Jama.Matrix dR = R1.transpose().times(R_int);
					Quaternion Q_res = new Quaternion(dR);
					double phi = 2*Math.atan2(Q_res.getVector().norm2(), Q_res.n);
//					if(Math.toDegrees(phi)>15)R_int = R1;
					TableRow row = Traj.addRow();

					row.setDouble(0, P_int.get(0, 0));
					row.setDouble(1, P_int.get(1, 0));
					row.setDouble(2, P_int.get(2, 0));
					row.setDouble(3, R_int.get(0, 0));
					row.setDouble(4, R_int.get(1, 0));
					row.setDouble(5, R_int.get(2, 0));
					row.setDouble(6, R_int.get(0, 1));
					row.setDouble(7, R_int.get(1, 1));
					row.setDouble(8, R_int.get(2, 1));
					row.setDouble(9, R_int.get(0, 2));
					row.setDouble(10, R_int.get(1, 2));
					row.setDouble(11, R_int.get(2, 2));
					row.setDouble(12, dt_soll);
				}
				//p.println(".......................");
			} else {

			TableRow row = Traj.addRow();

			row.setDouble(0, P1.get(0, 0));
			row.setDouble(1, P1.get(1, 0));
			row.setDouble(2, P1.get(2, 0));
			row.setDouble(3, R1.get(0, 0));
			row.setDouble(4, R1.get(1, 0));
			row.setDouble(5, R1.get(2, 0));
			row.setDouble(6, R1.get(0, 1));
			row.setDouble(7, R1.get(1, 1));
			row.setDouble(8, R1.get(2, 1));
			row.setDouble(9, R1.get(0, 2));
			row.setDouble(10, R1.get(1, 2));
			row.setDouble(11, R1.get(2, 2));
			row.setDouble(12, dt);
			}

		}

		p.saveTable(Traj, "traj_schnecke_xi1.csv");
	}

	public Jama.Matrix T_inv(Jama.Matrix T) {

		Jama.Matrix R = T.getMatrix(0, 2, 0, 2);
		Jama.Matrix RT = R.transpose();
		Jama.Matrix P = T.getMatrix(0, 2, 3, 3);
		Jama.Matrix RTP = RT.times(P.times(-1));

		double[][] ti = { { RT.get(0, 0), RT.get(0, 1), RT.get(0, 2), RTP.get(0, 0) },
				{ RT.get(1, 0), RT.get(1, 1), RT.get(1, 2), RTP.get(1, 0) },
				{ RT.get(2, 0), RT.get(2, 1), RT.get(2, 2), RTP.get(2, 0) }, { 0, 0, 0, 1 } };

		Jama.Matrix Ti = new Jama.Matrix(ti);

		return Ti;
	}

	public double s(double T, double t) {
		double s = t / T;
		if (t <= 0)
			s = 0;
		if (t >= T)
			s = 1;

		return s;

	}

	public Quaternion slerp(Quaternion qa, Quaternion qb, double t) {
		Quaternion qm = new Quaternion(1, 0, 0, 0);
		double cosHalfTheta = Math.abs(qa.n) * Math.abs(qb.n) + qa.ex * qb.ex + qa.ey * qb.ey + qa.ez * qb.ez;
		Jama.Matrix Re = qa.getRotationmatrix().transpose().times(qb.getRotationmatrix());
		
		Quaternion Qres = new Quaternion(Re);
	    cosHalfTheta = Math.cos(Math.atan2(Qres.getVector().norm2(), Qres.n));



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

	public void show(int step) {
		p.strokeWeight(2);
		p.stroke(0, 0, 255);

		double x = Traj.getDouble(step, 0);
		double y = Traj.getDouble(step, 1);
		double z = Traj.getDouble(step, 2);
		double xx = Traj.getDouble(step, 3);
		double xy = Traj.getDouble(step, 4);
		double xz = Traj.getDouble(step, 5);
		double yx = Traj.getDouble(step, 6);
		double yy = Traj.getDouble(step, 7);
		double yz = Traj.getDouble(step, 8);
		double zx = Traj.getDouble(step, 9);
		double zy = Traj.getDouble(step, 10);
		double zz = Traj.getDouble(step, 11);
		float size = 50;
		p.pushMatrix();
		p.strokeWeight(2);
		p.stroke(255, 0, 0);
		p.line((float) x, (float) y, (float) z, (float) (x + xx * size), (float) (y + xy * size),
				(float) (z + xz * size));
		p.stroke(0, 255, 0);
		p.line((float) x, (float) y, (float) z, (float) (x + yx * size), (float) (y + yy * size),
				(float) (z + yz * size));
		p.stroke(0, 0, 255);
		p.line((float) x, (float) y, (float) z, (float) (x + zx * size), (float) (y + zy * size),
				(float) (z + zz * size));

		p.popMatrix();

		for (int i = 0; i < step; i++) {
			x = Traj.getDouble(i, 0);
			y = Traj.getDouble(i, 1);
			z = Traj.getDouble(i, 2);

			p.point((float) x, (float) y, (float) z);
		}
	}

	public Table getTraj() {
		return Traj;
	}

	public Jama.Matrix getTransFromList(Table t, int i) {
		TableRow row = t.getRow(i);
		double x = row.getDouble(0);
		double y = row.getDouble(1);
		double z = row.getDouble(2);
		double xx = row.getDouble(3);
		double xy = row.getDouble(4);
		double xz = row.getDouble(5);
		double[][] xv = {{xx},{xy},{xz}};
		Jama.Matrix Xv = new Jama.Matrix(xv);
		double xl = Xv.norm2();
		Xv.timesEquals(1/xl);
		double yx = row.getDouble(6);
		double yy = row.getDouble(7);
		double yz = row.getDouble(8);
		double[][] yv = {{yx},{yy},{yz}};
		Jama.Matrix Yv = new Jama.Matrix(yv);
		double yl = Yv.norm2();
		Yv.timesEquals(1/yl);
		double zx = row.getDouble(9);
		double zy = row.getDouble(10);
		double zz = row.getDouble(11);
		double[][] zv = {{zx},{zy},{zz}};
		Jama.Matrix Zv = new Jama.Matrix(zv);
		double zl = Zv.norm2();
		//p.println(xl, zl, yl);
		Zv.timesEquals(1/zl);
		double dotxy = xx*yx+xy*yy+xz*yz;
		double dotxz = xx*zx+xy*zy+xz*zz;
		double dotyz = yx*zx+yy*zy+yz*zz;
	//	p.println(dotxy, dotxz, dotyz);
		double[][] transMat = { { Xv.get(0,0), Yv.get(0,0), Zv.get(0,0), x }, { Xv.get(1,0), Yv.get(1,0),  Zv.get(1,0), y }, { Xv.get(2,0), Yv.get(2,0), Zv.get(2,0), z }, { 0, 0, 0, 1 } };

		Jama.Matrix T = new Jama.Matrix(transMat);
		return T;

	}

}
