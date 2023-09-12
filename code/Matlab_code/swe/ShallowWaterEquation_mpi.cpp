////////////////////////////////////////////////
//    
//          Title : Assienment 1
//         Shallow Water Equation
//      
//       NAME:Li Lindong,Zheng Bujingda
//            ID:795501,838092
//              version:MPI
//
///////////////////////////////////////////////

//#include"stdafx.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>
#include<sstream>
#include <mpi.h>

using namespace std;

// Global variables
const int       N_D     = 2;   //num of dimension
const int       X       = 0;
const int       Y       = 1;
const double	x_min   = 0.00;
const double	x_max   = 100.00;
const double	y_min   = 0.00;
const double	y_max   = 100.00;
const int		myN_x   = 20;
const int		myN_y   = 20;
const double	t_min   = 0.00;
const double	t_max   = 100;
const double	Delta_t = 0.1;
const int		N_t     = (t_max - t_min) / Delta_t + 1;
const double    a       = 1.0 / 6;//coe for RK4 
const double    b       = 1.0 / 3;
const double 	g       = 9.81;
double			Delta_x;
double			Delta_y;

//MPI initialization
MPI_Status		status;
MPI_Datatype	strideType;
MPI_Comm		Comm2D;

// Function declarations
void    rk4(double ***k, double ***h_vx_vy);
void	exchange(double*** h_vx_vy, int myID, int N_Procs, int* neighbor);
void	write(fstream& file, double*** h_vx_vy, char myFileName[64]);
template<class T> T*** allocate3D(const int& M, const int& N, const int& O);
template<class T> void deallocate3D(T*** A);

int		main(int argc, char** argv)
{

	int		myID;
	int		N_Procs;
	MPI_Init(&argc, &argv);
	MPI_Comm_size(MPI_COMM_WORLD, &N_Procs); //return the size 
	MPI_Comm_rank(MPI_COMM_WORLD, &myID); // return the rank

	// Simulation parameters
	int				N               = sqrt(N_Procs);
	int				dimensions[N_D] = { N, N };             
	int				isPeriodic[N_D] = { 1, 1 };
	int				myCoords[N_D]   = { 0, 0 };
	int				neighbor[4]     = { 0, 0, 0, 0 };   //left,right,bottom,top
	int				N_x             = myN_x*dimensions[X];
	int				N_y             = myN_y*dimensions[Y];
	double			wtime           = 0.0;
	fstream			file;
	char        	myFileName[64];
	double			t               = 0;
	int 			x               = 0;
	int 			y               = 0;
	int             count           = 0;
	Delta_x = (x_max - x_min) / (N_x - 1);
	Delta_y = (y_max - y_min) / (N_y - 1);

	// Allocate arrays
	double***    h_vx_vy      = allocate3D<double>(myN_x + 6, myN_y + 6, 3);
	double***    temp_h_vx_vy = allocate3D<double>(myN_x + 6, myN_y + 6, 3);
	double***    k1           = allocate3D<double>(myN_x + 6, myN_y + 6, 3);
	double***    k2           = allocate3D<double>(myN_x + 6, myN_y + 6, 3);
	double***    k3           = allocate3D<double>(myN_x + 6, myN_y + 6, 3);
	double***    k4           = allocate3D<double>(myN_x + 6, myN_y + 6, 3);
	memset(neighbor, 0, 4 * sizeof(int));

	// Map the processors to the N x N grid, determine the new rank and determine the neighboring processes
	MPI_Cart_create(MPI_COMM_WORLD, N_D, dimensions, isPeriodic, 1, &Comm2D);
	MPI_Comm_rank(Comm2D, &myID);
	MPI_Cart_coords(Comm2D, myID, N_D, myCoords);
	MPI_Cart_shift(Comm2D, X, 1, &neighbor[0], &neighbor[1]);
	MPI_Cart_shift(Comm2D, Y, 1, &neighbor[2], &neighbor[3]);

	MPI_Barrier(Comm2D);

	if (myID == 0)
	{
		wtime = MPI_Wtime();
	}

	// Set initial condition
	for (int i = 3; i<myN_x + 3; i++)
	{
		x = x_min + (myCoords[X] * myN_x + i - 3)*Delta_x;
		for (int j = 3; j<myN_y + 3; j++)
		{
			y = y_min + (myCoords[Y] * myN_y + j - 3)*Delta_y;

			h_vx_vy[i][j][0] = 0.5*exp(-(pow(x - 30.0, 2) + pow(y - 30.0, 2)) / 25) + 1.0;//h
			h_vx_vy[i][j][1] = 0.0;//vx
			h_vx_vy[i][j][2] = 0.0;//vy
		}
	}

	// Create a new datatype to store values on bottom and Top boundary
	MPI_Type_vector(myN_x, 9, 3*(myN_y+6), MPI_DOUBLE, &strideType);
	MPI_Type_commit(&strideType);

	t = t_min;
	// Time marching loop
	for (int tm = 1; tm < N_t; tm++)//N_t-1
	{
		//K1
		exchange(h_vx_vy, myID, N_Procs, neighbor);
		rk4(k1, h_vx_vy);
		for (int j = 3; j < myN_x + 3; j++){
			for (int k = 3; k < myN_y + 3; k++){
				for (int l = 0; l < 3; l++){
					temp_h_vx_vy[j][k][l] = h_vx_vy[j][k][l] + 0.5*Delta_t*k1[j][k][l];//temp for getting k2
				}
			}
		}

		//K2
		exchange(temp_h_vx_vy, myID, N_Procs, neighbor);
		rk4(k2, temp_h_vx_vy);
		for (int j = 3; j < myN_x + 3; j++){
			for (int k = 3; k < myN_y + 3; k++){
				for (int l = 0; l < 3; l++){
					temp_h_vx_vy[j][k][l] = h_vx_vy[j][k][l] + 0.5*Delta_t*k2[j][k][l];//temp for getting k2
				}
			}
		}

		//K3
		exchange(temp_h_vx_vy, myID, N_Procs, neighbor);
		rk4(k3, temp_h_vx_vy);
		for (int j = 3; j < myN_x + 3; j++){
			for (int k = 3; k < myN_y + 3; k++){
				for (int l = 0; l < 3; l++){
					temp_h_vx_vy[j][k][l] = h_vx_vy[j][k][l] + Delta_t*k3[j][k][l];//temp for getting k2
				}
			}
		}

		//K4
		exchange(temp_h_vx_vy, myID, N_Procs, neighbor);
		rk4(k4, temp_h_vx_vy);
		for (int j = 3; j < myN_x + 3; j++){
			for (int k = 3; k < myN_y + 3; k++){
				for (int l = 0; l < 3; l++){
					h_vx_vy[j][k][l] = h_vx_vy[j][k][l] + Delta_t*(a*k1[j][k][l] + b*k2[j][k][l] + b*k3[j][k][l] + a*k4[j][k][l]);
				}
			}
		}
                //tagging the export file name with process ID and time index
			    sprintf_s(myFileName, "Process_%d_%d_%d.csv", myCoords[X], myCoords[Y], tm);
				write(file, h_vx_vy, myFileName);
				cout << "t= " << t << endl;
				t += Delta_t;
	}
	

	//memory de-allocation
	deallocate3D<double>(h_vx_vy);
	deallocate3D<double>(temp_h_vx_vy);
	deallocate3D<double>(k1);
	deallocate3D<double>(k2);
	deallocate3D<double>(k3);
	deallocate3D<double>(k4);

	if (myID == 0)
	{
		wtime = MPI_Wtime() - wtime;	// Record the end time and calculate elapsed time
		cout << "Simulation took " << wtime / N_t << " seconds per time step with " << N_Procs << " processes" <<endl;
		cout<< "total time= "<<wtime <<" myN_x: "<<myN_x<<"    myN_y: "<<myN_y<<endl;
		cout<<"N_t= "<<N_t<<"  Delta_x: "<<Delta_x<<"  Delta_y: "<<Delta_y<<endl;
	}

	MPI_Finalize();
	return 0;
}

void rk4(double ***k, double ***h_vx_vy){
// With the help of exchange function grid points at any position can 
// satisfy the boundary condition as the interior grid	
	for (int i = 3; i < myN_x + 3; i++){
		int ibc[6] = { i - 3, i - 2, i - 1, i + 1, i + 2, i + 3 };
		for (int j = 3; j < myN_y + 3; j++){
			int jbc[6] = { j - 3, j - 2, j - 1, j + 1, j + 2, j + 3 };

			//k of vx                                                     
			k[i][j][1] = -h_vx_vy[i][j][1] / Delta_x*(-1.0 / 60 * h_vx_vy[ibc[0]][j][1] + 3.0 / 20 * h_vx_vy[ibc[1]][j][1] + (-3.0 / 4)*h_vx_vy[ibc[2]][j][1] + 3.0 / 4 * h_vx_vy[ibc[3]][j][1] + (-3.0 / 20)*h_vx_vy[ibc[4]][j][1] + 1.0 / 60 * h_vx_vy[ibc[5]][j][1])
				        - h_vx_vy[i][j][2] / Delta_y*(-1.0 / 60 * h_vx_vy[i][jbc[0]][1] + 3.0 / 20 * h_vx_vy[i][jbc[1]][1] + (-3.0 / 4)*h_vx_vy[i][jbc[2]][1] + 3.0 / 4 * h_vx_vy[i][jbc[3]][1] + (-3.0 / 20)*h_vx_vy[i][jbc[4]][1] + 1.0 / 60 * h_vx_vy[i][jbc[5]][1])
				                       - g / Delta_x*(-1.0 / 60 * h_vx_vy[ibc[0]][j][0] + 3.0 / 20 * h_vx_vy[ibc[1]][j][0] + (-3.0 / 4)*h_vx_vy[ibc[2]][j][0] + 3.0 / 4 * h_vx_vy[ibc[3]][j][0] + (-3.0 / 20)*h_vx_vy[ibc[4]][j][0] + 1.0 / 60 * h_vx_vy[ibc[5]][j][0]);
			//cout << Delta_x << " " << Delta_y << " " << h_vx_vy[i][j][2] << endl;
			//k of vy
			k[i][j][2] = -h_vx_vy[i][j][1] / Delta_x*(-1.0 / 60 * h_vx_vy[ibc[0]][j][2] + 3.0 / 20 * h_vx_vy[ibc[1]][j][2] + (-3.0 / 4)*h_vx_vy[ibc[2]][j][2] + 3.0 / 4 * h_vx_vy[ibc[3]][j][2] + (-3.0 / 20)*h_vx_vy[ibc[4]][j][2] + 1.0 / 60 * h_vx_vy[ibc[5]][j][2])
				        - h_vx_vy[i][j][2] / Delta_y*(-1.0 / 60 * h_vx_vy[i][jbc[0]][2] + 3.0 / 20 * h_vx_vy[i][jbc[1]][2] + (-3.0 / 4)*h_vx_vy[i][jbc[2]][2] + 3.0 / 4 * h_vx_vy[i][jbc[3]][2] + (-3.0 / 20)*h_vx_vy[i][jbc[4]][2] + 1.0 / 60 * h_vx_vy[i][jbc[5]][2])
				                       - g / Delta_y*(-1.0 / 60 * h_vx_vy[i][jbc[0]][0] + 3.0 / 20 * h_vx_vy[i][jbc[1]][0] + (-3.0 / 4)*h_vx_vy[i][jbc[2]][0] + 3.0 / 4 * h_vx_vy[i][jbc[3]][0] + (-3.0 / 20)*h_vx_vy[i][jbc[4]][0] + 1.0 / 60 * h_vx_vy[i][jbc[5]][0]);
			//k of h
			k[i][j][0] = -1 / Delta_x*(-1.0 / 60 * h_vx_vy[ibc[0]][j][0] * h_vx_vy[ibc[0]][j][1] + 3.0 / 20 * h_vx_vy[ibc[1]][j][0] * h_vx_vy[ibc[1]][j][1] + (-3.0 / 4)*h_vx_vy[ibc[2]][j][0] * h_vx_vy[ibc[2]][j][1] + 3.0 / 4 * h_vx_vy[ibc[3]][j][0] * h_vx_vy[ibc[3]][j][1] + (-3.0 / 20)*h_vx_vy[ibc[4]][j][0] * h_vx_vy[ibc[4]][j][1] + 1.0 / 60 * h_vx_vy[ibc[5]][j][0] * h_vx_vy[ibc[5]][j][1])
				        - 1 / Delta_y*(-1.0 / 60 * h_vx_vy[i][jbc[0]][0] * h_vx_vy[i][jbc[0]][2] + 3.0 / 20 * h_vx_vy[i][jbc[1]][0] * h_vx_vy[i][jbc[1]][2] + (-3.0 / 4)*h_vx_vy[i][jbc[2]][0] * h_vx_vy[i][jbc[2]][2] + 3.0 / 4 * h_vx_vy[i][jbc[3]][0] * h_vx_vy[i][jbc[3]][2] + (-3.0 / 20)*h_vx_vy[i][jbc[4]][0] * h_vx_vy[i][jbc[4]][2] + 1.0 / 60 * h_vx_vy[i][jbc[5]][0] * h_vx_vy[i][jbc[5]][2]);
		}
	}
	return;
}

//MPI_Type_vector(myN_x, 3*3, 3 * (myN_y + 6), MPI_DOUBLE, &strideType);
//MPI_Type_commit(&strideType);

void	exchange(double*** h_vx_vy, int myID, int N_Procs, int* neighbor)
{
		//left: Sending a block of memory with dimension of 3 * 3 * (myN_y + 6) from Left edges of the grid to the Right
		MPI_Sendrecv(&(h_vx_vy[3][0][0]),         3 * 3 * (myN_y + 6), MPI_DOUBLE, neighbor[0], 0,
			         &(h_vx_vy[myN_x + 3][0][0]), 3 * 3 * (myN_y + 6), MPI_DOUBLE, neighbor[1], 0, Comm2D, &status);
		//right: Sending a block of memory with dimension of 3 * 3 * (myN_y + 6) from Right edges of the grid to the Left
		MPI_Sendrecv(&(h_vx_vy[myN_x][0][0]),     3 * 3 * (myN_y + 6), MPI_DOUBLE, neighbor[1], 0,
			         &(h_vx_vy[0][0][0]),         3 * 3 * (myN_y + 6), MPI_DOUBLE, neighbor[0], 0, Comm2D, &status);
		
		//bottom: Sending a block of memory with storing type of Stride from Bottom to the Top			
		MPI_Sendrecv(&(h_vx_vy[3][3][0]),         1,       strideType, neighbor[2], 0,
			         &(h_vx_vy[3][myN_y+3][0]),   1,       strideType, neighbor[3], 0, Comm2D, &status);
		//top:    Sending a block of memory with storing type of Stride from Top to the Bottom
		MPI_Sendrecv(&(h_vx_vy[3][myN_y][0]),     1,       strideType, neighbor[3], 0,
			         &(h_vx_vy[3][0][0]),         1,       strideType, neighbor[2], 0, Comm2D, &status);
	
	return;
}

template<class T>
T*** allocate3D(const int& M, const int& N, const int& O)
{
	T*** A = new T**[M];
	A[0] = new T*[M * N];
	A[0][0] = new T[M * N * O];

	for (int m = 1, mm = N; m<M; m++, mm += N)
	{
		A[m] = &A[0][mm];
	}

	for (int mn = 1, nn = O; mn<(M*N); mn++, nn += O)
	{
		A[0][mn] = &A[0][0][nn];
	}

	return A;
}

template<class T>
void deallocate3D(T*** M)
{
	if (M[0][0] != nullptr){
		delete[] M[0][0];
	}
	if (M[0] != nullptr){
		delete[] M[0];
	}
	if (M != nullptr){
		delete[] M;
	}
	return;
}

void write(fstream& file, double*** h_vx_vy, char myFileName[64]){

		file.open(myFileName, ios::out);
		for (int j = 3; j<myN_y + 3; j++)
		{
			int index_x, index_y;
			for (int i = 3; i<myN_x + 3; i++){

				index_x = i - 3;
				index_y = j - 3;

				file << index_x << ",\t" << index_y << ",\t" << h_vx_vy[i][j][0] << endl;
			}
		}
		file << "\n";
		file.close();
	
	return;
}