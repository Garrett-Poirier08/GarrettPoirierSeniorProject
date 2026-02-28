package org.firstinspires.ftc.teamcode;

public class FK {

    private double lengthOne = 5;

    private double lengthTwo = 5;
    private double originX = 0;
    private double originY = 0;
    private double originZ = 0;

    private double endEffectorX = 0;
    private double endEffectorY = 0;
    private double endEffectorZ = 0;

    private double targetX = 0;
    private double targetY = 0;
    private double targetZ = 0;

    private double theta = 0;

    private double[][] matrixTOne = new double [3][3];
    private double[][] matrixTTwo = new double [3][3];
    private double[][] matrixTThree = new double [3][3];
    private double[][] matrixT = new double[3][3];
    private double[][] x = {{0},{0},{1}};


    public void ini(double endX, double endY,double targetX,double targetY,double thetaOne,double thetaTwo, double thetaThree){
        endEffectorX = endX;
        endEffectorY = endY;
        this.targetX = targetX;
        this.targetY = targetY;
        buildMatrixR(matrixTOne,thetaOne,0);
        buildMatrixR(matrixTTwo,thetaTwo,lengthOne);
        buildMatrixR(matrixTThree,thetaThree,lengthTwo);
    }
    public void solve(){
        matrixT = multiplyMatrices3x3(multiplyMatrices3x3(matrixTOne,matrixTTwo),matrixTThree);
        multiplyMatrices3x1(x,matrixT);
    }
    public double[][] multiplyMatrices3x1(double[][] A, double[][] B){

            // Result will be 1x3
            double[][] result = new double[1][3];
            // Perform multiplication: (1x3) * (3x3) = (1x3)
            for (int i = 0; i < 1; i++) { // Rows of A
                for (int j = 0; j < 3; j++) { // Columns of B
                    for (int k = 0; k < 3; k++) { // Inner dimension
                        result[i][j] += A[i][k] * B[k][j];
                    }
                }
            }
            return result;
        }
    public static double[][] multiplyMatrices3x3(double[][] m1, double[][] m2) {
        double[][] result = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[i][j] += m1[i][k] * m2[k][j];
                }
            }
        }
        return result;
    }
    public void buildMatrixR(double[][] matrix,double theta, double length){
        matrix[0][0] = Math.cos(theta);
        matrix[0][1] = -Math.sin(theta);
        matrix[0][2] = -Math.sin(theta);
        matrix[1][0] = Math.sin(theta);;
        matrix[1][1] = Math.cos(theta);
        matrix[1][0] = Math.cos(theta);
        matrix[2][0] = length;
        matrix[2][1] = 0;
        matrix[2][2] = 1;
    }

}
