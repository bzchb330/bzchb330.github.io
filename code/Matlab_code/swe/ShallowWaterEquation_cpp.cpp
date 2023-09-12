
////////////////////////////////////////////////
//    
//          Title : Assienment 1
//         Shallow Water Equation
//      
//     NAME:Li Lingdong,Zheng Bujingda
//           ID:795501,838092
//             
//
///////////////////////////////////////////////

//#include "stdafx.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <time.h>//lib imported for recording running time

using namespace std;

const double x_min   = 0.0;
const double x_max   = 100.0;
const double y_min   = 0.0;
const double y_max   = 100.0;
const double t_min   = 0.0;
const double t_max   = 100;
const double Delta_x = 1;
const double Delta_y = 1;
const double Delta_t = 0.1;
const double pre_exp = -1.0 / 25;
const double g       = 9.81;
const double a       = 1.0 / 6;//coefficients for for RK4 
const double b       = 1.0 / 3;
const int N_x        = (x_max - x_min) / Delta_x + 1;
const int N_y        = (y_max - y_min) / Delta_y + 1;
const int N_t        = (t_max - t_min) / Delta_t + 1;

//function declaration 
void rk4(double ***k, double ***h_vx_vy);
void write(fstream& file, double*** h_vx_vy);
template<class T> T*** allocate3D(const int& M, const int& N, const int& O);
template<class T> void deallocate3D(T*** A);


int _tmain(int argc, _TCHAR* argv[])
{
	clock_t t_start = clock();//return start time 
	fstream	file;
	double  t  = 0;
	double  x, y;

	//3D array allocation 
	double*** h_vx_vy       = allocate3D<double>(N_x, N_y , 3);
	double*** temp_h_vx_vy  = allocate3D<double>(N_x , N_y , 3);
	double*** k1            = allocate3D<double>(N_x , N_y , 3);
	double*** k2            = allocate3D<double>(N_x , N_y , 3);
	double*** k3            = allocate3D<double>(N_x , N_y , 3);
	double*** k4            = allocate3D<double>(N_x , N_y , 3);

	for (int i = 0; i <N_x; i++){
		for (int j = 0; j < N_y; j++){
			x                = x_min + Delta_x*i;
			y                = y_min + Delta_y*j;
			h_vx_vy[i][j][1] = 0.0;//vx
			h_vx_vy[i][j][2] = 0.0;//vy
			//initial values for h
			h_vx_vy[i][j][0] = 1 + 0.5 * exp(-0.04*(pow((x - 30), 2) + pow((y - 30), 2)));
		}
	}

	//write files within the time marching loop
	file.open("shallow_water_equation_cpp.data", ios::out);
	
	for (int i = 0; i < N_t; i++){//<time marching loop>
		t = t + Delta_t;

		//k1
		rk4(k1, h_vx_vy);
		for (int j = 0; j < N_x; j++){
			for (int k = 0; k < N_y; k++){
				for (int l = 0; l < 3; l++){
					temp_h_vx_vy[j][k][l] = h_vx_vy[j][k][l] + 0.5*Delta_t*k1[j][k][l];//temp for getting k2
				}
			}
		}

		//k2
		rk4(k2, temp_h_vx_vy);
		for (int j = 0; j < N_x; j++){
			for (int k = 0; k < N_y; k++){
				for (int l = 0; l < 3; l++){
					temp_h_vx_vy[j][k][l] = h_vx_vy[j][k][l] + 0.5*Delta_t*k2[j][k][l];//temp for getting k3
				}
			}
		}

		//k3
		rk4(k3, temp_h_vx_vy);
		for (int j = 0; j < N_x; j++){
			for (int k = 0; k < N_y; k++){
				for (int l = 0; l < 3; l++){
					temp_h_vx_vy[j][k][l] = h_vx_vy[j][k][l] + Delta_t*k3[j][k][l];//temp for getting k4
				}
			}
		}

		//k4
		rk4(k4, temp_h_vx_vy);
		for (int j = 0; j < N_x; j++){
			for (int k = 0; k < N_y; k++){
				for (int l = 0; l < 3; l++){
					//assembling answer at one time
					h_vx_vy[j][k][l] = h_vx_vy[j][k][l] + Delta_t*(a*k1[j][k][l] + b*k2[j][k][l] + b*k3[j][k][l] + a*k4[j][k][l]);
				}
			}
		}

		write(file, h_vx_vy);
		cout << "t= " << t << endl;
	}//</time marching loop>
	file.close();

	clock_t t_end = clock();//return the end time 
	printf("Total Time = % d\n", t_end - t_start);

	//memory de-allocation
	deallocate3D<double>(h_vx_vy);
	deallocate3D<double>(temp_h_vx_vy);
	deallocate3D<double>(k1);
	deallocate3D<double>(k2);
	deallocate3D<double>(k3);
	deallocate3D<double>(k4);

	return 0;
}


void rk4(double ***k, double ***h_vx_vy){
// ibc and jbc are two coefficient arrays that helps to satisfy the boundary condition
// at different position of the grid
	for (int i = 0; i < N_x; i++){
		int ibc[6] = { i - 3, i - 2, i - 1, i + 1, i + 2, i + 3 };
		if (i == 0)
			ibc[0] = N_x - 3, ibc[1] = N_x - 2, ibc[2] = N_x - 1, ibc[3] = 1, ibc[4] = 2, ibc[5] = 3;
		else if (i == 1)
			ibc[0] = N_x - 2, ibc[1] = N_x - 1, ibc[2] = 0, ibc[3] = 2, ibc[4] = 3, ibc[5] = 4;
		else if (i == 2)
			ibc[0] = N_x - 1, ibc[1] = 0, ibc[2] = 1, ibc[3] = 3, ibc[4] = 4, ibc[5] = 5;
		else if (i == N_x - 1)
			ibc[0] = N_x - 4, ibc[1] = N_x - 3, ibc[2] = N_x - 2, ibc[3] = 0, ibc[4] = 1, ibc[5] = 2;
		else if (i == N_x - 2)
			ibc[0] = N_x - 5, ibc[1] = N_x - 4, ibc[2] = N_x - 3, ibc[3] = N_x - 1, ibc[4] = 0, ibc[5] = 1;
		else if (i == N_x - 3)
			ibc[0] = N_x - 6, ibc[1] = N_x - 5, ibc[2] = N_x - 4, ibc[3] = N_x - 2, ibc[4] = N_x - 1, ibc[5] = 0;

		for (int j = 0; j < N_y; j++){
			int jbc[6] = { j - 3, j - 2 , j - 1 , j + 1 , j + 2 , j + 3  };
			if (j == 0)//bc1
				jbc[0] = N_y - 3, jbc[1] = N_y - 2, jbc[2] = N_y - 1, jbc[3] = 1, jbc[4] = 2, jbc[5] = 3;
			else if (j == 1)//bc2
				jbc[0] = N_y - 2, jbc[1] = N_y - 1, jbc[2] = 0, jbc[3] = 2, jbc[4] = 3, jbc[5] = 4;
			else if (j == 2)//bc3
				jbc[0] = N_y - 1, jbc[1] = 0, jbc[2] = 1, jbc[3] = 3, jbc[4] = 4, jbc[5] = 5;
			else if (j == N_y - 1)//bc4
				jbc[0] = N_y - 4, jbc[1] = N_y - 3, jbc[2] = N_y - 2, jbc[3] = 0, jbc[4] = 1, jbc[5] = 2;
			else if (j == N_y - 2)//bc5
				jbc[0] = N_y - 5, jbc[1] = N_y - 4, jbc[2] = N_y - 3, jbc[3] = N_y - 1, jbc[4] = 0, jbc[5] = 1;
			else if (j == N_y - 3)//bc6
				jbc[0] = N_y - 6, jbc[1] = N_y - 5, jbc[2] = N_y - 4, jbc[3] = N_y - 2, jbc[4] = N_y - 1, jbc[5] = 0;


			//k of vx                                                     
			k[i][j][1] = -h_vx_vy[i][j][1] / Delta_x*(-1.0 / 60 * h_vx_vy[ibc[0]][j][1] + 3.0 / 20 * h_vx_vy[ibc[1]][j][1] + (-3.0 / 4)*h_vx_vy[ibc[2]][j][1] + 3.0 / 4 * h_vx_vy[ibc[3]][j][1] + (-3.0 / 20)*h_vx_vy[ibc[4]][j][1] + 1.0 / 60 * h_vx_vy[ibc[5]][j][1])
				        - h_vx_vy[i][j][2] / Delta_y*(-1.0 / 60 * h_vx_vy[i][jbc[0]][1] + 3.0 / 20 * h_vx_vy[i][jbc[1]][1] + (-3.0 / 4)*h_vx_vy[i][jbc[2]][1] + 3.0 / 4 * h_vx_vy[i][jbc[3]][1] + (-3.0 / 20)*h_vx_vy[i][jbc[4]][1] + 1.0 / 60 * h_vx_vy[i][jbc[5]][1])
				                       - g / Delta_x*(-1.0 / 60 * h_vx_vy[ibc[0]][j][0] + 3.0 / 20 * h_vx_vy[ibc[1]][j][0] + (-3.0 / 4)*h_vx_vy[ibc[2]][j][0] + 3.0 / 4 * h_vx_vy[ibc[3]][j][0] + (-3.0 / 20)*h_vx_vy[ibc[4]][j][0] + 1.0 / 60 * h_vx_vy[ibc[5]][j][0]);
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

void	write(fstream& file, double*** h_vx_vy)
{
	for (int i = 0; i<N_x; i++)
	{
		for (int j = 0; j<N_y; j++){
			file << h_vx_vy[i][j][0] << "\t";
		}
		file << endl;
	}
	
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
		A[m]    = &A[0][mm];
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