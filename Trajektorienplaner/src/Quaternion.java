import Jama.Matrix;

public class Quaternion {

	double n;
	double ex;
	double ey;
	double ez;
	double theta;
	double x = 0;
	double y = 0;
	double z = 0;

	public Quaternion(Jama.Matrix M) {

		if (M.getRowDimension() == 4 && M.getColumnDimension() == 4) {

			Jama.Matrix P = M.getMatrix(0, 2, 3, 3);
			Jama.Matrix R = M.getMatrix(0, 2, 0, 2);

			Rot2Quat2(R);
			theta = 2 * Math.atan2(Math.sqrt(ex * ex + ey * ey + ez * ez), n);
			x = P.get(0, 0);
			y = P.get(1, 0);
			z = P.get(2, 0);

		}

		if (M.getRowDimension() == 3 && M.getColumnDimension() == 3) {

			Rot2Quat2(M);

			theta = 2 * Math.atan2(Math.sqrt(ex * ex + ey * ey + ez * ez), n);
		}

	}

	public Quaternion(double _n, double _ex, double _ey, double _ez) {

		n = _n;
		ex = _ex;
		ey = _ey;
		ez = _ez;
		theta = 2 * Math.atan2(Math.sqrt(ex * ex + ey * ey + ez * ez), n);

	}

	public Quaternion(double _n, double _ex, double _ey, double _ez, double _x, double _y, double _z) {

		n = _n;
		ex = _ex;
		ey = _ey;
		ez = _ez;

		theta = 2 * Math.atan2(Math.sqrt(ex * ex + ey * ey + ez * ez), n);
		x = _x;
		y = _y;
		z = _z;

	}

	private void Rot2Quat(Jama.Matrix R) {

		n = 0.5 * Math.sqrt(R.get(0, 0) + R.get(1, 1) + R.get(2, 2) + 1);
		ex = 0.5 * Math.signum(R.get(2, 1) - R.get(1, 2)) * Math.sqrt(R.get(0, 0) - R.get(1, 1) - R.get(2, 2) + 1);
		ey = 0.5 * Math.signum(R.get(0, 2) - R.get(2, 0)) * Math.sqrt(R.get(1, 1) - R.get(2, 2) - R.get(0, 0) + 1);
		ez = 0.5 * Math.signum(R.get(1, 0) - R.get(0, 1)) * Math.sqrt(R.get(2, 2) - R.get(0, 0) - R.get(1, 1) + 1);
		if (Double.isNaN(n))
			n = 0;
		if (Double.isNaN(ex))
			ex = 0;
		if (Double.isNaN(ey))
			ey = 0;
		if (Double.isNaN(ez))
			ez = 0;
		
		if (n>1)
			n = 1;
		if (ex>1)
			ex = 1;
		if (ey>1)
			ey = 1;
		if (ez>1)
			ez = 1;

	}

	private void Rot2Quat2(Jama.Matrix R) {

		double r11 = R.get(0, 0);
		double r12 = R.get(0, 1);
		double r13 = R.get(0, 2);

		double r21 = R.get(1, 0);
		double r22 = R.get(1, 1);
		double r23 = R.get(1, 2);

		double r31 = R.get(2, 0);
		double r32 = R.get(2, 1);
		double r33 = R.get(2, 2);
		
		n = 0.25 * Math.sqrt(Math.pow((r11 + r22 + r33 + 1), 2) + Math.pow((r32 - r23), 2) + Math.pow((r13 - r31), 2)
				+ Math.pow((r21 - r12), 2));
		ex = 0.25 * Math.signum(r32 - r23) * Math.sqrt(Math.pow((r32 - r23), 2) + Math.pow((r11 - r22 - r33 + 1), 2)
				+ Math.pow((r21 + r12), 2) + Math.pow((r31 + r13), 2));
		ey = 0.25 * Math.signum(r13 - r31) * Math.sqrt(Math.pow((r13 - r31), 2) + Math.pow((r21 + r12), 2)
				+ Math.pow((r22 - r11 - r33 + 1), 2) + Math.pow((r32 + r23), 2));
		ez = 0.25 * Math.signum(r21 - r12) * Math.sqrt(Math.pow((r21 - r12), 2) + Math.pow((r31 + r13), 2)
				+ Math.pow((r32 + r23), 2) + Math.pow((r33 - r11 - r22 + 1), 2));
		if (Double.isNaN(n))
			n = 0;
		if (Double.isNaN(ex))
			ex = 0;
		if (Double.isNaN(ey))
			ey = 0;
		if (Double.isNaN(ez))
			ez = 0;
	}

	

	public Jama.Matrix getVector() {
		double[][] res = { { ex }, { ey }, { ez } };

		Jama.Matrix Res = new Jama.Matrix(res);
		return Res;
	}

	public Jama.Matrix getPos() {
		double[][] res = { { x }, { y }, { z } };

		Jama.Matrix Res = new Jama.Matrix(res);
		return Res;
	}

	public Quaternion minus(Quaternion q2) {
		Quaternion res = new Quaternion(n - q2.n, ex - q2.ex, ey - q2.ey, ez - q2.ez, x - q2.x, y - q2.y, z - q2.z);
		return res;
	}

	public void times(double d) {
		n = n * d;
		ex = ex * d;
		ey = ey * d;
		ez = ez * d;
	}

	public Quaternion times(Quaternion q2) {
		double n1 = n;
		double n2 = q2.n;

		double dx = q2.ex;
		double dy = q2.ey;
		double dz = q2.ez;

		double eTe = ex * dx + ey * dy + ez * dz;

		double[] n1e2 = { n1 * dx, n1 * dy, n1 * dz };
		double[] n2e1 = { n2 * ex, n2 * ey, n2 * ez };
		double[] e1xe2 = { ey * dz - ez * dy, ez * dx - ex * dz, ex * dy - ey * dx };
		double[] e_res = { n1e2[0] + n2e1[0] + e1xe2[0], n1e2[1] + n2e1[1] + e1xe2[1], n1e2[2] + n2e1[2] + e1xe2[2] };

		Quaternion res = new Quaternion(n1 * n2 - eTe, e_res[0], e_res[1], e_res[2]);
		return res;
	}

	public Quaternion inverse() {
		Quaternion res = new Quaternion(n, -ex, -ey, -ez);
		return res;
	}

	public Jama.Matrix getRotationmatrix() {
		double[][] r = { { 2 * (n * n + ex * ex) - 1, 2 * (ex * ey - n * ez), 2 * (ex * ez + n * ey) },
				{ 2 * (ex * ey + n * ez), 2 * (n * n + ey * ey) - 1, 2 * (ey * ez - n * ex) },
				{ 2 * (ex * ez - n * ey), 2 * (ey * ez + n * ex), 2 * (n * n + ez * ez) - 1 } };
		Jama.Matrix R = new Jama.Matrix(r);
		return R;
	}

	public Jama.Matrix getRotationmatrix2() {

		double[][] r2 = { { n * n + ex * ex - ey * ey - ez * ez, 2 * (ex * ey - n * ez), 2 * (ex * ez + n * ey) },
				{ 2 * (ex * ey + n * ez), n * n - ex * ex + ey * ey - ez * ez, 2 * (ey * ez - n * ex) },
				{ 2 * (ex * ez - n * ey), 2 * (ey * ez + n * ex), n * n - ex * ex - ey * ey + ez * ez } };
		Jama.Matrix R = new Jama.Matrix(r2);
		return R;
	}

	public Jama.Matrix getTransformationmatrix() {
		double[][] r = { { 2 * (n * n + ex * ex) - 1, 2 * (ex * ey - n * ez), 2 * (ex * ez + n * ey), x },
				{ 2 * (ex * ey + n * ez), 2 * (n * n + ey * ey) - 1, 2 * (ey * ez - n * ex), y },
				{ 2 * (ex * ez - n * ey), 2 * (ey * ez + n * ex), 2 * (n * n + ez * ez) - 1, z }, { 0, 0, 0, 1 } };
		Jama.Matrix R = new Jama.Matrix(r);
		return R;
	}
}
