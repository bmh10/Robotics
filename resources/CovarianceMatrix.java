import java.awt.geom.Point2D;


public class CovarianceMatrix {

	private static final int N = 10;
	
	public static void main(String[] args) {
		//Assume start point is (5,5)
		//ALL UNITS ARE CM
		Point2D[] in = {p(5.3f,5.7f),  
				        p(5.4f,5.4f),
				        p(5.2f,5.3f),
				        p(5.1f,5.2f),
				        p(5.6f,5.1f),
				        p(5.0f,5.0f),
				        p(4.8f,5.1f),
				        p(4.9f,4.9f),
				        p(5.2f,5.3f),
				        p(5.3f,4.8f)
					   };
		
		double xdash = 0;
		double ydash = 0;
		for (int i=0; i < N; i++) {
			xdash += in[i].getX();
			ydash += in[i].getY();
		}
		xdash /= N;
		ydash /= N;
		
		/*Covariance Matrix P
		 *NB Indexing of matrix is as follows:
		 *  ( 0 , 1 )
		 *  ( 2 , 3 )
		 */
		double[] P = {0.0, 0.0, 0.0, 0.0};
		double xi, yi, tmp;
		
		for (int i=0; i < N; i++) {
			xi = in[i].getX();
			yi = in[i].getY();
			
			P[0] += sqr(xi - xdash);
			
			tmp = (xi - xdash)*(yi - ydash);
			P[1] += tmp;
			P[2] += tmp;
			
			P[3] += sqr(yi - ydash);
		}
		
		for (int i=0; i < P.length; i++) {
			P[i] /= N;
		}
		
		System.out.println("COVARIANCE MATRIX");
		System.out.println("-----------------------------------------------------");
		System.out.println("P =  ( "+P[0]+" , "+P[1]+" )");
		System.out.println("     ( "+P[2]+" , "+P[3]+" )\n\n");
		
		System.out.println("SANITY CHECK (expected to be a few cms)");
		System.out.println("-----------------------------------------------------");
		System.out.println("Std deviation of x = "+ Math.sqrt(P[0]));
		System.out.println("Std deviation of y = "+ Math.sqrt(P[3]));
		
	}
	
	private static Point2D p(double x, double y) {
		return new Point2D.Double(x, y);
	}
	
	private static double sqr(double d) {
		return d*d;
	}

}
