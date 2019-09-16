import processing.core.PApplet;

public class KoordinatenSystem {

  Jama.Matrix eulerW;
  PApplet p;
  //Größe der KoordinatenSysteme
  int size=20;
  int strokeWeight=1;
  
  float x, y, z, alpha, beta, gamma;
  float mx, my, mz;
  float lx, ly, lz;
  float nx, ny, nz;
  
  public KoordinatenSystem(PApplet _p, double _x, double _y, double _z, double _alpha, double _beta, double _gamma) {
    p = _p;
	alpha=(float) _alpha;
    beta=(float) (_beta);
    gamma=(float) (_gamma);
    x=(float) _x;
    y=(float) _y;
    z=(float) _z;

    update();
  }

  void update() {
    double a=Math.toRadians(alpha);
    double b=Math.toRadians(beta);
    double c=Math.toRadians(gamma);

    double[][] EulerWinkel_StandardKonventionZYZ2= {
      {c(a)*c(b)*c(c)-s(a)*s(c), -c(a)*c(b)*s(c)-s(a)*c(c), c(a)*s(b)}, 
      {s(a)*c(b)*c(c)+c(a)*s(c), -s(a)*c(b)*s(c)+c(a)*c(c), s(a)*s(b)}, 
      {-s(b)*c(c), s(b)*s(c), c(b)}};


    eulerW= new Jama.Matrix(EulerWinkel_StandardKonventionZYZ2);
    //eulerW= new Jama.Matrix(Transformation_Yaw_Pitch_Roll);


    mx=(float)eulerW.get(0, 0);
    my=(float)eulerW.get(1, 0);
    mz=(float)eulerW.get(2, 0);
    lx=(float)eulerW.get(0, 1);
    ly=(float)eulerW.get(1, 1);
    lz=(float)eulerW.get(2, 1);
    nx=(float)eulerW.get(0, 2);
    ny=(float)eulerW.get(1, 2);
    nz=(float)eulerW.get(2, 2);
  }
  
  public void setTrans(Jama.Matrix T) {
	    mx=(float)T.get(0, 0);
	    my=(float)T.get(1, 0);
	    mz=(float)T.get(2, 0);
	    lx=(float)T.get(0, 1);
	    ly=(float)T.get(1, 1);
	    lz=(float)T.get(2, 1);
	    nx=(float)T.get(0, 2);
	    ny=(float)T.get(1, 2);
	    nz=(float)T.get(2, 2);
	    x =(float)T.get(0, 3);
	    y =(float)T.get(1, 3);
	    z =(float)T.get(2, 3);
  }

  public void show() {

    p.pushMatrix();
    p.strokeWeight(strokeWeight);
    p.stroke(255, 0, 0);
    p.line(x, y, z, x+mx*size, y+my*size, z+ mz*size);   
    p.stroke(0, 255, 0); 
    p.line(x, y, z, x+lx*size, y+ly*size, z+lz*size);
    p.stroke(0, 0, 255);
    p.line(x, y, z, x+nx*size, y+ny*size, z+nz*size);

    p.popMatrix();
  }

  public Jama.Matrix getT() {

    double[][] t = {
      {mx, lx, nx, x}, 
      {my, ly, ny, y}, 
      {mz, lz, nz, z}, 
      {0, 0, 0, 1}};
      
      Jama.Matrix T = new Jama.Matrix(t);
      
    return T;
  }
  
	private double s(double p) {
		return Math.sin(p);
	}

	private double c(double p) {
		return Math.cos(p);
	}

  void setStrokeWeight(int _strokeWeight) {
    strokeWeight = _strokeWeight;
  }

  void setSize(int _size) {
    size = _size;
  }

  void setX(int _x) {
    x=_x;
    update();
  }
  void setY(int _y) {
    y=_y;
    update();
  }
  void setZ(int _z) {
    z=_z;
    update();
  }
  void setAlpha(int _a) {
    alpha=_a;
    update();
  }
  void setBeta(int _b) {
    beta=_b;
    update();
  }
  void setGamma(int _g) {
    gamma=_g;
    update();
  }
}
