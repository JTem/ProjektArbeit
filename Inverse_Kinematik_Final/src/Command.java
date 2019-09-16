
import Jama.Matrix;
import processing.core.PApplet;
import processing.data.Table;
import processing.data.TableRow;

public class Command {

	PApplet p;
	Inverse_Kinematics ik;
	KoordinatenSystem ks;

	Main m;
	int counter_global = 1;
	Jama.Matrix Xs, Xm, Xe;
	boolean running = false;
	boolean finished = false;
	long current_time;
	long last_time = System.nanoTime();

	long current_time_save;
	long last_time_save = System.nanoTime();

	double t;
	double time = 0;
	Table TransList, SpeedList, AngleList, PosList;
	int step = 0;

	public Command(PApplet _p, Inverse_Kinematics _ik, Main _m) {
		p = _p;
		ik = _ik;
		m = _m;
		ks = new KoordinatenSystem(p, 0, 0, 0, 0, 0, 0);
		SpeedList = new Table();
		AngleList = new Table();
		PosList = new Table();

		for (int i = 0; i < 7; i++) {
			SpeedList.addColumn();
			AngleList.addColumn();
			PosList.addColumn();
		}

	}

	public void waitTime(double Time, int _count) {

		if (_count == counter_global) {
			if (!running) {
				t = 0;
				last_time = System.nanoTime()-10;
			}
			if (t <= Time) {
				current_time = System.nanoTime();
				double dt = 0.000000001 * (current_time - last_time);
				last_time = current_time;
				running = true;
				t = t + dt;
				ik.q_dot = new Jama.Matrix(ik.null_vec6);

			}
			if (t >= Time && running) {
				running = false;
				counter_global++;

			}
		}
	}

	public void move_Home(double Time, int _count) {
		if (_count == counter_global) {
			if (!running) {
				last_time = System.nanoTime()-10;
				Xs = ik.q;
				Xe = new Jama.Matrix(ik.null_vec6);

				
				t = 0;
			}

			if (t <= Time) {
				current_time = System.nanoTime();
				double dt = 0.000000001 * (current_time - last_time);
				last_time = current_time;
				running = true;
				t = t + dt;
	
				Jama.Matrix q1 = ik.path_trough_Jointspace(Xs, Xe, Time, t);
				Jama.Matrix q2 = ik.path_trough_Jointspace(Xs, Xe, Time, t+dt);
				Jama.Matrix dq = q2.minus(q1);
				Jama.Matrix q_dot = dq.times(1/dt);
				ik.q_dot = q_dot;
				ik.q = q1;
				
				ik.forward_Kinematik(q1);
				ik.set_q(q1);
			}
			if (t >= Time && running) {
				running = false;
				counter_global++;
				ik.q_dot = new Jama.Matrix(ik.null_vec6);
			}
		}
	}
	
	public void move_Rest(double Time, int _count) {
		if (_count == counter_global) {
			if (!running) {
				last_time = System.nanoTime()-10;
				Xs = ik.q;
				double q1 = Math.toRadians(0);
				double q2 = Math.toRadians(97);
				double q3 = Math.toRadians(-87);
				double q4 = Math.toRadians(0);
				double q5 = Math.toRadians(0);
				double q6 = Math.toRadians(0);
				double[][] qq = { { q1 }, { q2 }, { q3 }, { q4 }, { q5 }, { q6 } };
				Xe = new Jama.Matrix(qq);
				t = 0;
			}

			if (t <= Time) {
				current_time = System.nanoTime();
				double dt = 0.000000001 * (current_time - last_time);
				last_time = current_time;
				running = true;
				t = t + dt;
	
				Jama.Matrix q1 = ik.path_trough_Jointspace(Xs, Xe, Time, t);
				Jama.Matrix q2 = ik.path_trough_Jointspace(Xs, Xe, Time, t+dt);
				Jama.Matrix dq = q2.minus(q1);
				Jama.Matrix q_dot = dq.times(1/dt);
				ik.q_dot = q_dot;
				
				ik.q = q1;
				
				ik.forward_Kinematik(q1);
				ik.set_q(q1);
			}
			if (t >= Time && running) {
				running = false;
				counter_global++;
				ik.q_dot = new Jama.Matrix(ik.null_vec6);
			}
		}
	}

	public void linear_Path(KoordinatenSystem se, double Time, int _count) {

		if (_count == counter_global) {
			if (!running) {
				Xs = ik.get_current_Trans();
				Xe = se.getT();
				last_time = System.nanoTime()-10;
				t = 0;
			}

			if (t <= Time) {

				current_time = System.nanoTime();
				double dt = 0.000000001 * (current_time - last_time);
				last_time = current_time;

				t = t + dt * ik.speed_correction_factor;
				running = true;
				Quaternion Qd = ik.straight_Line_Path_dec(Xs, Xe, Time, t);
				// m.setQ(Qd);
				Quaternion Qd2 = ik.straight_Line_Path_dec(Xs, Xe, Time, t + dt);
				// Jama.Matrix Pd = Qd.getPos();
				Quaternion der_quat = Qd2.minus(Qd);
				double[] v = { der_quat.x / dt, der_quat.y / dt, der_quat.z / dt };
				der_quat.times(2 / dt);
				Quaternion w = der_quat.times(Qd.inverse());

				double[][] xd = { { v[0] }, { v[1] }, { v[2] }, { w.ex }, { w.ey }, { w.ez } };
				// double[][] xd = { { v[0] }, { v[1] }, { v[2] }, { 0 }, { 0 }, { 0 } };
				// double[][] xp = {{ v[0] }, { v[1] }, { v[2] }};

				Jama.Matrix Xd = new Jama.Matrix(xd);

				// ik.differential_Kinematics_z_Free(Qd, Xd, dt);
				// ik.analytical_IK(Qd.getTransformationmatrix());
				ik.differential_Kinematics(Qd, Xd, 1*dt);

			}
			if (t >= Time && running) {
				running = false;
				counter_global++;
				ik.q_dot = new Jama.Matrix(ik.null_vec6);
			}

		}

	}

	public void circle_Path(KoordinatenSystem sm, KoordinatenSystem se, double Time, int _count) {

		if (_count == counter_global) {
			if (!running) {
				Xs = ik.get_current_Trans();
				Xm = sm.getT();
				Xe = se.getT();
				t = 0;
				last_time = System.nanoTime()-10;
			}
			if (t <= Time) {

				current_time = System.nanoTime();
				double dt = 0.000000001 * (current_time - last_time);
				last_time = current_time;

				t = t + dt * ik.speed_correction_factor;
				running = true;

				Quaternion Qd = ik.circular_Path(Xs, Xm, Xe, Time, t);
				// m.setQ(Qd);

				Quaternion Qd2 = ik.circular_Path(Xs, Xm, Xe, Time, t + dt);

				Quaternion der_quat = Qd2.minus(Qd);
				double[] v = { der_quat.x / dt, der_quat.y / dt, der_quat.z / dt };
				der_quat.times(2 / dt);
				Quaternion w = der_quat.times(Qd.inverse());

				double[][] xd = { { v[0] }, { v[1] }, { v[2] }, { w.ex }, { w.ey }, { w.ez } };

				Jama.Matrix Xd = new Jama.Matrix(xd);

				// ik.differential_Kinematics_z_Free(Qd, Xd, dt);
				ik.differential_Kinematics(Qd, Xd, 1 * dt);

			}
			if (t >= Time && running) {
				running = false;
				counter_global++;
				ik.q_dot = new Jama.Matrix(ik.null_vec6);
			}

		}

	}

	public void moveToPrintStart(Table list, Jama.Matrix T_Nozzle, double Time, int _count) {
		if (_count == counter_global) {
			if (!running) {
				Jama.Matrix T = getTransFromList(list, 0);
				Xe = T_Nozzle.times(ik.T_inv(T));
				Xs = ik.get_current_Trans();
				t = 0;
			}
			if (t <= Time) {

				current_time = System.nanoTime();
				double dt = 0.000000001 * (current_time - last_time);
				last_time = current_time;

				t = t + dt;
				running = true;
				Quaternion Qd = ik.straight_Line_Path_dec(Xs, Xe, Time, t);
				Quaternion Qd2 = ik.straight_Line_Path_dec(Xs, Xe, Time, t + dt);

				Quaternion der_quat = Qd2.minus(Qd);
				double[] v = { der_quat.x / dt, der_quat.y / dt, der_quat.z / dt };
				der_quat.times(2 / dt);
				Quaternion w = der_quat.times(Qd.inverse());

				double[][] xd = { { v[0] }, { v[1] }, { v[2] }, { w.ex }, { w.ey }, { w.ez } };

				Jama.Matrix Xd = new Jama.Matrix(xd);

				ik.differential_Kinematics(Qd, Xd, 2 * dt);

			}
			if (t > Time && running) {
				running = false;
				counter_global++;
				ik.q_dot = new Jama.Matrix(ik.null_vec6);
			}
		}
	}

	public void doPrint2(Table list, Jama.Matrix T_Nozzle, int _count) {
		if (_count == counter_global) {
			if (!running) {
				t = 0;
				step = 0;
				running = true;

			}

			current_time = System.nanoTime();
			double ddt = 0.000000001 * (current_time - last_time);
			last_time = current_time;

			t += ddt;

			if (step < list.getRowCount() - 1) {
				Jama.Matrix T = getTransFromList(list, step);
				Jama.Matrix T2 = getTransFromList(list, step + 1);
				Jama.Matrix Td = T_Nozzle.times(ik.T_inv(T));
				Jama.Matrix Td2 = T_Nozzle.times(ik.T_inv(T2));
//				Jama.Matrix test = T.times(ik.T_inv(T));
//				Jama.Matrix R = T.getMatrix(0, 2, 0, 2);
				// Jama.Matrix Rt = R.transpose();
				// Jama.Matrix test2 = R.times(Rt);
				// //test2.print(3, 3);

				// test.print(4, 4);
				// Quaternion Q_Nozzle = new Quaternion(T_Nozzle);
				Quaternion Qd = new Quaternion(Td);
				Quaternion Qd2 = new Quaternion(Td2);
//				Quaternion Qd = Q_Nozzle.times(Q1.inverse());
//				Quaternion Qd2 = Q_Nozzle.times(Q2.inverse());
				m.setQ(Qd);
				Quaternion der_quat = Qd2.minus(Qd);
				if (ddt < 0.0000001) {
					ddt = 0.0001;
				}
				double[] v = { der_quat.x / ddt, der_quat.y / ddt, der_quat.z / ddt };
				der_quat.times(2 / ddt);
				Quaternion w = der_quat.times(Qd.inverse());

				double[][] xd = { { v[0] }, { v[1] }, { v[2] }, { w.ex }, { w.ey }, { w.ez } };
				Jama.Matrix Xd = new Jama.Matrix(xd);
				double dt = getDtFromList(list, step);
				ik.differential_Kinematics(Qd, Xd, ddt);

				if (t >= dt) {
					step++;
					m.setProgression(step);
					t = 0;
				}
			}

			if (step == list.getRowCount() - 1) {
				running = false;
				counter_global++;
			}
		}
	}

	public void saveList(int _count) {

		if (_count == counter_global) {
			p.saveTable(SpeedList, "/data/speed_schnecke_xi1.csv");
			p.saveTable(AngleList, "/data/angle_schnecke_xi1.csv");
			p.saveTable(PosList, "pos_Data_noPID.csv");
			counter_global++;
		}
	}

	public void addPosToList() {
		TableRow row = PosList.addRow();
		Jama.Matrix p = ik.getPosFromTrans(ik.get_current_Trans());
		row.setDouble(0, p.get(0, 0));
		row.setDouble(1, p.get(1, 0));
		row.setDouble(2, p.get(2, 0));
	}

	public void addStatesToList() {
		TableRow row_speed = SpeedList.addRow();
		TableRow row_angle = AngleList.addRow();

		current_time_save = System.nanoTime();
		double dt = 0.000000001 * (current_time_save - last_time_save);
		last_time_save = current_time_save;
		time = time + dt;
		Jama.Matrix current_speed = ik.q_dot;
		Jama.Matrix current_angle = ik.q;

		row_speed.setDouble(0, Math.toDegrees(current_speed.get(0, 0)));
		row_speed.setDouble(1, Math.toDegrees(current_speed.get(1, 0)));
		row_speed.setDouble(2, Math.toDegrees(current_speed.get(2, 0)));
		row_speed.setDouble(3, Math.toDegrees(current_speed.get(3, 0)));
		row_speed.setDouble(4, Math.toDegrees(current_speed.get(4, 0)));
		row_speed.setDouble(5, Math.toDegrees(current_speed.get(5, 0)));
		row_speed.setDouble(6, time);

		row_angle.setDouble(0, Math.toDegrees(current_angle.get(0, 0)));
		row_angle.setDouble(1, Math.toDegrees(current_angle.get(1, 0)));
		row_angle.setDouble(2, Math.toDegrees(current_angle.get(2, 0)));
		row_angle.setDouble(3, Math.toDegrees(current_angle.get(3, 0)));
		row_angle.setDouble(4, Math.toDegrees(current_angle.get(4, 0)));
		row_angle.setDouble(5, Math.toDegrees(current_angle.get(5, 0)));
		row_angle.setDouble(6, time);
	}

	public double getDtFromList(Table t, int i) {
		TableRow row = t.getRow(i);
		float dtime = row.getFloat(12);
		return dtime;
	}

	public Jama.Matrix getTransFromList(Table t, int i) {
		TableRow row = t.getRow(i);
		double x = row.getDouble(0);
		double y = row.getDouble(1);
		double z = row.getDouble(2);
		double xx = row.getDouble(3);
		double xy = row.getDouble(4);
		double xz = row.getDouble(5);
		double yx = row.getDouble(6);
		double yy = row.getDouble(7);
		double yz = row.getDouble(8);
		double zx = row.getDouble(9);
		double zy = row.getDouble(10);
		double zz = row.getDouble(11);

		double[][] transMat = { { xx, yx, zx, x }, { xy, yy, zy, y }, { xz, yz, zz, z }, { 0, 0, 0, 1 } };

		Jama.Matrix T = new Jama.Matrix(transMat);
		return T;

	}
}
