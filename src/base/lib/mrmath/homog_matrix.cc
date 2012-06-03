#include <cmath>
#include <ostream>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "base/lib/mrmath/mrmath.h"

namespace mrrocpp {
namespace lib {

// ******************************************************************************************
//                                     definicje skladowych klasy Homog_matrix
// ******************************************************************************************

const double Homog_matrix::ALPHA_SENSITIVITY = 0.000001;

Homog_matrix::Homog_matrix()
{
	setIdentity();
}

Homog_matrix::Homog_matrix(const K_vector & versor_x, const K_vector & versor_y, const K_vector & versor_z, const K_vector & angles)
{
	matrix_m[0][0] = 1;
	matrix_m[1][0] = versor_x[2] * angles[0] + versor_y[2] * angles[1] + versor_z[2] * angles[2];
	matrix_m[2][0] = -1 * (versor_x[1] * angles[0] + versor_y[1] * angles[1] + versor_z[1] * angles[2]);

	matrix_m[0][1] = -1 * (versor_x[2] * angles[0] + versor_y[2] * angles[1] + versor_z[2] * angles[2]);
	matrix_m[1][1] = 1;
	matrix_m[2][1] = versor_x[0] * angles[0] + versor_y[0] * angles[1] + versor_z[0] * angles[2];

	matrix_m[0][2] = versor_x[1] * angles[0] + versor_y[1] * angles[1] + versor_z[1] * angles[2];
	matrix_m[1][2] = -1 * (versor_x[0] * angles[0] + versor_y[0] * angles[1] + versor_z[0] * angles[2]);
	matrix_m[2][2] = 1;

	matrix_m[0][3] = 0.0;
	matrix_m[1][3] = 0.0;
	matrix_m[2][3] = 0.0;
}

Homog_matrix::Homog_matrix(const Xyz_Euler_Zyz_vector & l_vector)
{
	set_from_xyz_euler_zyz(l_vector);
}

Homog_matrix::Homog_matrix(const Xyz_Rpy_vector & l_vector)
{
	set_from_xyz_rpy(l_vector);
}

Homog_matrix::Homog_matrix(const Xyz_Angle_Axis_vector & l_vector)
{
	set_from_xyz_angle_axis(l_vector);
}

Homog_matrix::Homog_matrix(const K_vector & angles)
{
	matrix_m[0][0] = 1;
	matrix_m[0][0] = angles[2];
	matrix_m[0][0] = -angles[1];

	matrix_m[0][1] = -angles[2];
	matrix_m[1][1] = 1;
	matrix_m[2][1] = angles[0];

	matrix_m[0][2] = angles[1];
	matrix_m[1][2] = -angles[0];
	matrix_m[2][2] = 1;

	matrix_m[0][3] = 0.0;
	matrix_m[1][3] = 0.0;
	matrix_m[2][3] = 0.0;
}

Homog_matrix::Homog_matrix(const double r[3][3], const double t[3])
{
	// utworznie macierzy jednorodnej na podstawie
	// - macierzy rotacji r
	// - wektora przesuniecia t

	// i - i-ta kolumna
	// j - j-ty wiersz

	// uzupelnianie macierzy przeksztalcenia wierszami
	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++)
			matrix_m[i][j] = r[i][j];

		matrix_m[j][3] = t[j];
	}
}

Homog_matrix::Homog_matrix(double x, double y, double z)
{
	// Tworzy macierz jednorodna
	// 			| 1 0 0 x |
	// 			| 0 1 0 y |
	// 			| 0 0 1 z |

	remove_rotation();

	matrix_m[0][3] = x;
	matrix_m[1][3] = y;
	matrix_m[2][3] = z;
}

Homog_matrix::Homog_matrix(double r11, double r12, double r13, double t1, double r21, double r22, double r23, double t2, double r31, double r32, double r33, double t3)
{
	matrix_m[0][0] = r11;
	matrix_m[0][1] = r12;
	matrix_m[0][2] = r13;
	matrix_m[0][3] = t1;
	matrix_m[1][0] = r21;
	matrix_m[1][1] = r22;
	matrix_m[1][2] = r23;
	matrix_m[1][3] = t2;
	matrix_m[2][0] = r31;
	matrix_m[2][1] = r32;
	matrix_m[2][2] = r33;
	matrix_m[2][3] = t3;
}

Homog_matrix::Homog_matrix(const std::string & str)
{
	set(str);
}

Homog_matrix::Homog_matrix(const Eigen::Matrix <double, 3, 4>& eigen_matrix)
{
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix_m[i][j] = eigen_matrix(i, j);
		}
	}
}

Homog_matrix::Homog_matrix(const Eigen::Matrix<double, 4 , 4>& eigen_matrix)
{
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 4; ++j) {
			matrix_m[i][j] = eigen_matrix(i, j);
		}
	}
}


void Homog_matrix::setIdentity()
{
	// Tworzy macierz jednostkowa
	// 			| 1 0 0 0 |
	// 			| 0 1 0 0 |
	// 			| 0 0 1 0 |

	// i - i-ta kolumna
	// j - j-ty wiersz

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == j)
				matrix_m[j][i] = 1;
			else
				matrix_m[j][i] = 0;
		}
	}
}

void Homog_matrix::set(const std::string & str)
{
	// Prepare char-separated tokenizer
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

	// Setup skipped skip-delimiters and kept-delimiters
	boost::char_separator<char> sep(" \t\r\n", "[;]");

	// Instantiate tokenizer
	tokenizer tokens(str, sep);

	// Setup token iterator
	tokenizer::iterator tok_iter = tokens.begin();

	// Parse matrix string
	if(*tok_iter++ != "[") throw std::runtime_error("Opening bracket expected");
	matrix_m[0][0] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[0][1] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[0][2] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[0][3] = boost::lexical_cast<double>(*tok_iter++);
	if(*tok_iter++ != ";") throw std::runtime_error("1st semicolon expected");
	matrix_m[1][0] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[1][1] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[1][2] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[1][3] = boost::lexical_cast<double>(*tok_iter++);
	if(*tok_iter++ != ";") throw std::runtime_error("2nd semicolon expected");
	matrix_m[2][0] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[2][1] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[2][2] = boost::lexical_cast<double>(*tok_iter++);
	matrix_m[2][3] = boost::lexical_cast<double>(*tok_iter++);
	if(*tok_iter++ != ";") throw std::runtime_error("3rd semicolon expected");
	if(boost::lexical_cast<double>(*tok_iter++) != 0) throw std::runtime_error("1st zero expected");
	if(boost::lexical_cast<double>(*tok_iter++) != 0) throw std::runtime_error("2nd zero expected");
	if(boost::lexical_cast<double>(*tok_iter++) != 0) throw std::runtime_error("3rd zero expected");
	if(boost::lexical_cast<double>(*tok_iter++) != 1) throw std::runtime_error("1 zero expected");
	if(*tok_iter++ != "]") throw std::runtime_error("Closing bracket expected");

	// Check if all tokens has been parsed
	if(tok_iter != tokens.end()) throw std::runtime_error("End-of-string expected");
}

// Przeksztalcenie do formy XYZ_EULER_ZYZ i zwrocenie w tablicy.
/*
 void Homog_matrix::get_xyz_euler_zyz(double t[6]) const
 {
 double alfa, beta, gamma; // Katy Euler'a Z-Y-Z
 //const double EPS=1.0E-10;
 const double EPS=0.1;

 // Sprawdzenie czy cos(beta) == 1.
 if (matrix[2][2] < 1+EPS && matrix[2][2] > 1-EPS)
 {
 // Osie pierwszego oraz trzeciego obrotu pokrywaja sie.
 // Obrot o kat (alfa + gamma).
 beta = 0.0;
 // Przyjecie, ze jeden jest zero, a drugi zostaje obliczony
 gamma = 0.0;
 alfa = atan2(-matrix[1][0],matrix[0][0]);
 }
 // Sprawdzenie czy cos(beta) == -1.
 else if(matrix[2][2] < -1+EPS && matrix[2][2] > -1-EPS)
 {
 // Osie pierwszego oraz trzeciego obrotu pokrywaja sie, lecz sa skierowanie przeciwnie.
 // Obrot o kat  (alfa - gamma)
 beta = M_PI;
 // Przyjecie, ze jeden jest zero, a drugi zostaje obliczony.
 gamma = 0.0;
 alfa = atan2(-matrix[0][1], matrix[1][1]);
 }
 else
 {
 // Normalne rozwiazanie.
 double sb = sqrt(matrix[0][2]*matrix[0][2] + matrix[1][2]*matrix[1][2]);
 beta = atan2(sb, matrix[2][2]);
 //		beta = acos(matrix[2][2]);

 alfa = atan2(matrix[2][1], matrix[2][0]);
 gamma = atan2(matrix[1][2], -matrix[0][2]);

 // Sinus beta.
 // Jesli alfa !=0 oraz |alfa| != 1 (alfa != pi && alfa != -pi)
 //		if (alfa > EPS && alfa < -EPS && fabs(alfa) > M_PI +EPS && fabs(alfa) < M_PI - EPS)
 //		{
 //printf(" (alfa > EPS && alfa < -EPS && fabs(alfa) > M_PI +EPS && fabs(alfa) < M_PI - EPS)\n");
 //			sb = matrix[2][1] / sin(alfa);
 //		}
 //		else
 //		{
 //printf(" else\n");
 //			sb = matrix[2][0] / cos(alfa);
 //		}
 };//: else

 // Zredukowanie katow.
 alfa = reduce(alfa, -M_PI, M_PI, 2*M_PI);
 beta = reduce(beta, 0, M_PI, M_PI);
 gamma = reduce(gamma, -M_PI, M_PI, 2*M_PI);

 // Przepisanie wyniku do tablicy
 for(int i = 0;i<3;i++)
 t[i] = matrix[3][i];
 t[3] = alfa;
 t[4] = beta;
 t[5] = gamma;
 }
 */


void Homog_matrix::get_xyz_euler_zyz(Xyz_Euler_Zyz_vector & l_vector) const
{
	double alfa, beta, gamma; // Katy Euler'a Z-Y-Z
	//const double EPS=1.0E-10;
	const double EPS = 0.000001;
	// Sprawdzenie czy cos(beta) == 1.
	if (matrix_m[2][2] < 1 + EPS && matrix_m[2][2] > 1 - EPS) {
		// Osie pierwszego oraz trzeciego obrotu pokrywaja sie.
		// Obrot o kat (alfa + gamma).
		beta = 0.0;
		// Przyjecie, ze jeden jest zero, a drugi zostaje obliczony
		gamma = 0.0;
		alfa = atan2(-matrix_m[0][1], matrix_m[0][0]);
	}
	// Sprawdzenie czy cos(beta) == -1.
	else if (matrix_m[2][2] < -1 + EPS && matrix_m[2][2] > -1 - EPS) {
		// Osie pierwszego oraz trzeciego obrotu pokrywaja sie, lecz sa skierowanie przeciwnie.
		// Obrot o kat  (alfa - gamma)
		beta = M_PI;
		// Przyjecie, ze jeden jest zero, a drugi zostaje obliczony.
		gamma = 0.0;
		alfa = atan2(-matrix_m[1][0], matrix_m[1][1]);
	} else {
		// Normalne rozwiazanie.
		double sb = hypot(matrix_m[2][0], matrix_m[2][1]);
		beta = atan2(sb, matrix_m[2][2]);
		//		beta = acos(matrix[2][2]);

		alfa = atan2(matrix_m[1][2], matrix_m[0][2]);
		gamma = atan2(matrix_m[2][1], -matrix_m[2][0]);

		// Sinus beta.
	}

	// Przepisanie wyniku do tablicy
	for (int i = 0; i < 3; i++) {
		l_vector[i] = matrix_m[i][3];
	}
	l_vector[3] = alfa;
	l_vector[4] = beta;
	l_vector[5] = gamma;
}


void Homog_matrix::set_from_xyz_euler_zyz(const Xyz_Euler_Zyz_vector & l_vector)
{
	// Reduction of alpha and beta to <-PI, PI) makes no sense, because sin and cos are periodical with period equal to 2PI.
	//	const double alfa = reduce(l_vector[3], -M_PI, M_PI, 2 * M_PI);
	//	const double gamma = reduce(l_vector[5], -M_PI, M_PI, 2 * M_PI);

	// The beta angle is reduced to <0,PI).
	//	const double beta = reduce(l_vector[4], 0, M_PI, M_PI);
	Xyz_Euler_Zyz_vector
			l_reduced(l_vector[0], l_vector[1], l_vector[2], l_vector[3], reduce(l_vector[4], 0, M_PI, M_PI), l_vector[5]);

	// Compute the homogenous matrix coefficients.
	set_from_xyz_euler_zyz_without_limits(l_reduced);
}


void Homog_matrix::get_xyz_euler_zyz_without_limits(Xyz_Euler_Zyz_vector & l_vector, const double alpha_, const double beta_, const double gamma_) const
{
	double phi, theta, psi;
	const double EPS = 1.0E-10;

#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"u33 = "<< matrix_m[2][2] << endl;
#endif

	if ((matrix_m[2][2] < (1 + EPS)) && (matrix_m[2][2] > (1 - EPS))) {
		// If u33 = 1 then theta is 0.
		theta = 0;
		// Infinite number of solutions: only the phi + psi value can be computed, thus we assume, that phi will equal to the previous one.
		phi = alpha_;
		psi = atan2(matrix_m[1][0], matrix_m[0][0]) - phi;
		// atan2(r(2,1), r(1,1)) - phi
#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"CASE I: u33=1 => ["<<phi<<", "<<theta<<", "<<psi<<"]\n";
#endif
		l_vector << matrix_m[0][3], matrix_m[1][3], matrix_m[2][3], phi, theta, psi;
	} else if ((matrix_m[2][2] < (-1 + EPS)) && (matrix_m[2][2] > (-1 - EPS))) {
		// If u33 = -1 then theta is equal to pi.
		theta = M_PI;
		// Infinite number of solutions: only the phi - psi value can be computed, thus we assume, that phi will equal to the previous one.
		phi = alpha_;
		psi = - atan2(-matrix_m[0][1], -matrix_m[0][0]) + phi;
#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"CASE II: u33=-1 => ["<<phi<<", "<<theta<<", "<<psi<<"]\n";
#endif
		l_vector << matrix_m[0][3], matrix_m[1][3], matrix_m[2][3], phi, theta, psi;
	} else {
		// Two possible solutions.
//		double sb = hypot(matrix_m[2][0], matrix_m[2][1]);

		// First solution.
		theta = atan2(sqrt(1 - matrix_m[2][2]*matrix_m[2][2]), matrix_m[2][2]);
//		theta = atan2(sb, matrix_m[2][2]);

		phi = atan2(matrix_m[1][2], matrix_m[0][2]);
		psi = atan2(matrix_m[2][1], -matrix_m[2][0]);
#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"CASE III: atan(u33, sqrt(1-u33^3)) => ["<<phi<<", "<<theta<<", "<<psi<<"]\n";
#endif
		// Compute maximal delta.
		const double dist = std::max(std::max(fabs(phi - alpha_), fabs(theta - beta_)), fabs(psi - gamma_));

		// Second solution.
		const double theta2 = atan2(-sqrt(1 - matrix_m[2][2]*matrix_m[2][2]), matrix_m[2][2]);
//		theta = atan2(-sb, matrix_m[2,2]);

		const double phi2 = atan2(-matrix_m[1][2], -matrix_m[0][2]);
		const double psi2 = atan2(-matrix_m[2][1], matrix_m[2][0]);
#if(DEBUG_KINEMATICS)
		std::cout.precision(15);
		std::cout<<"CASE IV: atan(u33, -sqrt(1-u33^3)) => ["<<phi2<<", "<<theta2<<", "<<psi2<<"]\n";
#endif
		// Compute maximal delta.
		const double dist2 = std::max(std::max(fabs(phi2 - alpha_), fabs(theta2 - beta_)), fabs(psi2 - gamma_));

		// Select best solution.
		if (dist < dist2)
			l_vector << matrix_m[0][3], matrix_m[1][3], matrix_m[2][3], phi, theta, psi;
		else
			l_vector << matrix_m[0][3], matrix_m[1][3], matrix_m[2][3], phi2, theta2, psi2;
	}
}

void Homog_matrix::set_from_xyz_euler_zyz_without_limits(const Xyz_Euler_Zyz_vector & l_vector)
{
	// Compute the sines and cosines of all angles.
	const double c_alfa = cos(l_vector[3]);
	const double s_alfa = sin(l_vector[3]);
	const double c_beta = cos(l_vector[4]);
	const double s_beta = sin(l_vector[4]);
	const double c_gamma = cos(l_vector[5]);
	const double s_gamma = sin(l_vector[5]);

	// Compute the rotation matrix coefficients.
	matrix_m[0][0] = c_alfa * c_beta * c_gamma - s_alfa * s_gamma;
	matrix_m[1][0] = s_alfa * c_beta * c_gamma + c_alfa * s_gamma;
	matrix_m[2][0] = -s_beta * c_gamma;

	matrix_m[0][1] = -c_alfa * c_beta * s_gamma - s_alfa * c_gamma;
	matrix_m[1][1] = -s_alfa * c_beta * s_gamma + c_alfa * c_gamma;
	matrix_m[2][1] = s_beta * s_gamma;

	matrix_m[0][2] = c_alfa * s_beta;
	matrix_m[1][2] = s_alfa * s_beta;
	matrix_m[2][2] = c_beta;

	// Copy position variables.
	matrix_m[0][3] = l_vector[0];
	matrix_m[1][3] = l_vector[1];
	matrix_m[2][3] = l_vector[2];
}



// UWAGA ponizsze dwie funckje nie byly testowane - po pozytywnych  testach usunac komentarz
// Przeksztalcenie do formy XYZ_RPY (rool pitch yaw) i zwrocenie w tablicy.
void Homog_matrix::get_xyz_rpy(Xyz_Rpy_vector & l_vector) const
{
	// x, y, z
	l_vector[0] = matrix_m[0][3];
	l_vector[1] = matrix_m[1][3];
	l_vector[2] = matrix_m[2][3];

	// alfa (wokol z) , beta (wokol y), gamma (wokol x)
	l_vector[3] = atan2(matrix_m[2][1], matrix_m[2][2]);
	l_vector[4] = atan2(-matrix_m[2][0], hypot(matrix_m[0][0], matrix_m[1][0]));
	l_vector[5] = atan2(matrix_m[1][0], matrix_m[0][0]);

	// TODO: właściwa implementacja!! str. 63 craig.
}

// Wypelnienie wspolczynnikow macierzy na podstawie danych w formie XYZ_RPY.
void Homog_matrix::set_from_xyz_rpy(const Xyz_Rpy_vector & l_vector)
{
	// alfa (wokol z) , beta (wokol y), gamma (wokol x)
	const double c_alfa = cos(l_vector[3]);
	const double s_alfa = sin(l_vector[3]);
	const double c_beta = cos(l_vector[4]);
	const double s_beta = sin(l_vector[4]);
	const double c_gamma = cos(l_vector[5]);
	const double s_gamma = sin(l_vector[5]);

	// Obliczenie macierzy rotacji.
	matrix_m[0][0] = c_alfa * c_beta;
	matrix_m[1][0] = s_alfa * c_beta;
	matrix_m[2][0] = -s_beta;

	matrix_m[0][1] = c_alfa * s_beta * s_gamma - s_alfa * c_gamma;
	matrix_m[1][1] = s_alfa * s_beta * s_gamma + c_alfa * c_gamma;
	matrix_m[2][1] = c_beta * s_gamma;

	matrix_m[0][2] = c_alfa * s_beta * c_gamma + s_alfa * s_gamma;
	matrix_m[1][2] = s_alfa * s_beta * c_gamma - c_alfa * s_gamma;
	matrix_m[2][2] = c_beta * c_gamma;

	// Przepisanie polozenia.
	set_translation_vector(l_vector[0], l_vector[1], l_vector[2]);
}

//Wypelnienie macierzy na podstawie parametrow  wejsciowych w postaci kwaternionu
void Homog_matrix::set_from_xyz_quaternion(double eta, double eps1, double eps2, double eps3, double x, double y, double z)
{
	// Macierz rotacji
	matrix_m[0][0] = 2 * (eta * eta + eps1 * eps1) - 1;
	matrix_m[1][0] = 2 * (eps1 * eps2 + eta * eps3);
	matrix_m[2][0] = 2 * (eps1 * eps3 - eta * eps2);

	matrix_m[0][1] = 2 * (eps1 * eps2 - eta * eps3);
	matrix_m[1][1] = 2 * (eta * eta + eps2 * eps2) - 1;
	matrix_m[2][1] = 2 * (eps2 * eps3 + eta * eps1);

	matrix_m[0][2] = 2 * (eps1 * eps3 + eta * eps2);
	matrix_m[1][2] = 2 * (eps2 * eps3 - eta * eps1);
	matrix_m[2][2] = 2 * (eta * eta + eps3 * eps3) - 1;

	// Uzupelnienie macierzy wspolrzednymi polozenia
	set_translation_vector(x, y, z);
}

void Homog_matrix::set_from_xyz_angle_axis_gamma(const Xyz_Angle_Axis_Gamma_vector & xyz_aa_gamma)
{
	// Transform into the xyz_aa representation.
	Xyz_Angle_Axis_vector xyz_aa;
	for (int i = 0; i < 3; i++) {
		// The translation.
		xyz_aa[i] = xyz_aa_gamma[i];
		// The rotation in the form of vector multiplied by angle.
		xyz_aa[3 + i] = xyz_aa_gamma[3 + i] * xyz_aa_gamma[6];
	}

	// Set matrix on the base of x,y,z, vx,vy,vz.
	set_from_xyz_angle_axis(xyz_aa);
}

void Homog_matrix::set_from_xyz_angle_axis(const Xyz_Angle_Axis_vector & xyz_aa)
{
	const double alfa = sqrt(xyz_aa[3] * xyz_aa[3] + xyz_aa[4] * xyz_aa[4] + xyz_aa[5] * xyz_aa[5]);

	double kx, ky, kz;

	if (alfa > ALPHA_SENSITIVITY) {
		kx = xyz_aa[3] / alfa;
		ky = xyz_aa[4] / alfa;
		kz = xyz_aa[5] / alfa;
	} else {
		kx = ky = kz = 0.0;
	}

	// funkcja ta dokonuje zmiany macierzy jednorodnej na macierz okresona poleceniem
	// w formie XYZ_ANGLE_AXIS
	// Utworznie macierzy jednorodnej na podstawie rozkazu w formie XYZ_ANGLE_AXIS

	// c_alfa - kosinus kata alfa
	// s_alfa - sinus kata alfa
	// v_alfa = 1 - c_alfa;

	// wartosci poszczegolnych funkcji trygonometrycznych dla kata obrotu
	const double c_alfa = cos(alfa);
	const double s_alfa = sin(alfa);
	const double v_alfa = 1 - c_alfa;

	// macierz rotacji na podstawie wzoru 2.80 ze strony 68
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	matrix_m[0][0] = kx * kx * v_alfa + c_alfa;
	matrix_m[1][0] = kx * ky * v_alfa + kz * s_alfa;
	matrix_m[2][0] = kx * kz * v_alfa - ky * s_alfa;

	matrix_m[0][1] = kx * ky * v_alfa - kz * s_alfa;
	matrix_m[1][1] = ky * ky * v_alfa + c_alfa;
	matrix_m[2][1] = ky * kz * v_alfa + kx * s_alfa;

	matrix_m[0][2] = kx * kz * v_alfa + ky * s_alfa;
	matrix_m[1][2] = ky * kz * v_alfa - kx * s_alfa;
	matrix_m[2][2] = kz * kz * v_alfa + c_alfa;

	// uzupelnienie macierzy
	set_translation_vector(xyz_aa[0], xyz_aa[1], xyz_aa[2]);
}

void Homog_matrix::get_xyz_angle_axis_gamma(Xyz_Angle_Axis_Gamma_vector & xyz_aa_gamma) const
{
	// przeksztalcenie macierzy jednorodnej do rozkazu w formie XYZ_ANGLE_AXIS
	static const double EPS = 1.0E-6;
	static const double delta = (M_PI - 3.14158);

	double Kd[3]; // Kd - K z "daszkiem" - wersor kierunkowy

	// obliczenia zgodne ze wzorami 2.81 i 2.82 ze strony 68
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	double value = (matrix_m[0][0] + matrix_m[1][1] + matrix_m[2][2] - 1) / 2;

	// wyeliminowanie niedokladnosci obliczeniowej lub bledu programisty
	if (value < -1)
		value = -1;
	else if (value > 1)
		value = 1;

	// kat obrotu
	double gamma = acos(value);

	if ((gamma < M_PI + delta) && (gamma > M_PI - delta)) // kat obrotu 180 stopni = Pi radianow
	{

		Kd[0] = sqrt((matrix_m[0][0] + 1) / (double) 2);
		Kd[1] = sqrt((matrix_m[1][1] + 1) / (double) 2);
		Kd[2] = sqrt((matrix_m[2][2] + 1) / (double) 2);

		// ustalenie znakow paramertow wersora

		if (((Kd[0] < -EPS) || (Kd[0] > EPS)) && ((Kd[1] < -EPS) || (Kd[1] > EPS)) && ((Kd[2] < -EPS) || (Kd[2] > EPS))) {
			if ((matrix_m[0][1] < 0) && (matrix_m[0][2] < 0)) {
				Kd[1] = Kd[1] * (-1);
				Kd[2] = Kd[2] * (-1);
			} else if ((matrix_m[0][1] < 0) && (matrix_m[0][2] > 0))
				Kd[1] = Kd[1] * (-1);
			else if ((matrix_m[0][1] > 0) && (matrix_m[0][2] < 0))
				Kd[2] = Kd[2] * (-1);
		} else if (((Kd[0] > -EPS) && (Kd[0] < EPS)) && ((Kd[1] < -EPS) || (Kd[1] > EPS)) && ((Kd[2] < -EPS) || (Kd[2]
				> EPS))) // kx==0, ky!=0, kz!=0
		{
			if (matrix_m[1][2] < 0)
				Kd[2] = Kd[2] * (-1);
		} else if (((Kd[0] < -EPS) || (Kd[0] > EPS)) && ((Kd[1] > -EPS) && (Kd[1] < EPS)) && ((Kd[2] < -EPS) || (Kd[2]
				> EPS))) // kx!=0, ky==0, kz!=0
		{
			if (matrix_m[0][2] < 0)
				Kd[2] = Kd[2] * (-1);
		} else if (((Kd[0] < -EPS) || (Kd[0] > EPS)) && ((Kd[1] < -EPS) || (Kd[1] > EPS)) && ((Kd[2] > -EPS) && (Kd[2]
				< EPS))) // kx!=0 ky!=0 kz==0
		{
			if (matrix_m[0][1] < 0)
				Kd[1] = Kd[1] * (-1);
		}

	}// end kat obrotu 180 stopni
	else if ((gamma < ALPHA_SENSITIVITY) && (gamma > -ALPHA_SENSITIVITY)) // kat obrotu 0 stopni
	{

		for (int i = 0; i < 3; i++)
			Kd[i] = 0;

		gamma = 0;

	} else // standardowe obliczenia
	{
		// sinus kata obrotu alfa
		const double s_alfa = sin(gamma);

		Kd[0] = (1 / (2 * s_alfa)) * (matrix_m[2][1] - matrix_m[1][2]);
		Kd[1] = (1 / (2 * s_alfa)) * (matrix_m[0][2] - matrix_m[2][0]);
		Kd[2] = (1 / (2 * s_alfa)) * (matrix_m[1][0] - matrix_m[0][1]);
	}

	// Write the computed values in output parameter.
	xyz_aa_gamma << matrix_m[0][3], matrix_m[1][3], matrix_m[2][3], Kd[0], Kd[1], Kd[2], gamma;
}

void Homog_matrix::get_xyz_angle_axis(Xyz_Angle_Axis_vector & xyz_aa) const
{
	Xyz_Angle_Axis_Gamma_vector xyz_aa_gamma;
	// Compute x,y,z, vx,vy,vz, gamma.
	get_xyz_angle_axis_gamma(xyz_aa_gamma);

	// Copy results to table.
	for (int i = 0; i < 3; i++) {
		// The translation.
		xyz_aa[i] = xyz_aa_gamma[i];
		// The rotation in the form of vector multiplied by angle.
		xyz_aa[3 + i] = xyz_aa_gamma[3 + i] * xyz_aa_gamma[6];
	}
}

// Obliczenie kwaternionu na podstawie macierzy rotacji
void Homog_matrix::get_xyz_quaternion(double t[7]) const
{
	const double EPSILON = 1.0E-10; // dokladnosc

	double eps1, eps2, eps3;

	// slad macierzy
	const double tr = matrix_m[0][0] + matrix_m[1][1] + matrix_m[2][2];

	double eta = 0.5 * sqrt(tr + 1);

	if (eta > EPSILON) {
		eps1 = (matrix_m[2][1] - matrix_m[1][2]) / (4 * eta);
		eps2 = (matrix_m[0][2] - matrix_m[2][0]) / (4 * eta);
		eps3 = (matrix_m[1][0] - matrix_m[0][1]) / (4 * eta);
	} else {
		const int s_iNext[3] = { 2, 3, 1 };

		double *tmp[3] = { &eps1, &eps2, &eps3 };

		int i = 0;
		if (matrix_m[1][1] > matrix_m[0][0])
			i = 1;
		if (matrix_m[2][2] > matrix_m[1][1])
			i = 2;

		const int j = s_iNext[i - 1];
		const int k = s_iNext[j - 1];

		double fRoot = sqrt(matrix_m[i - 1][i - 1] - matrix_m[j - 1][j - 1] - matrix_m[k - 1][k - 1] + 1.0);

		*tmp[i - 1] = 0.5 * fRoot;
		fRoot = 0.5 / fRoot;
		eta = (matrix_m[k - 1][j - 1] - matrix_m[j - 1][k - 1]) * fRoot;
		*tmp[j - 1] = (matrix_m[j - 1][i - 1] + matrix_m[i - 1][j - 1]) * fRoot;
		*tmp[k - 1] = (matrix_m[k - 1][i - 1] + matrix_m[i - 1][k - 1]) * fRoot;
	}

	// Przepisanie wyniku do tablicy

	for (int i = 0; i < 3; i++)
		t[i] = matrix_m[3][i];
	t[3] = eta;
	// BLAD cppcheck - niezainijowano eps1 eps2 eps3

	//	t[4]=eps1;
	//	t[5]=eps2;
	//	t[6]=eps3;
	// Koniec Blad cppcheck

}

Homog_matrix Homog_matrix::operator*(const Homog_matrix & m) const
{
	// mnozenie macierzy
	// mnozenie realizowane jest zgodnie ze wzorem 2.41 ze strony 54
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	Homog_matrix zwracana;

	// i - i-ta kolumna
	// j - j-ty wiersz

	// macierz rotacji
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			for (int a = 0; a < 3; a++) {
				if (a == 0)
					zwracana.matrix_m[j][i] = matrix_m[j][a] * m.matrix_m[a][i];
				else
					zwracana.matrix_m[j][i] += matrix_m[j][a] * m.matrix_m[a][i];

			}
		}

	// wektor przesuniecia
	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++)
			zwracana.matrix_m[j][3] += matrix_m[j][i] * m.matrix_m[i][3];
		zwracana.matrix_m[j][3] += matrix_m[j][3];
	}

	return zwracana;
}

void Homog_matrix::operator *=(const Homog_matrix & m)
{
	// mnozenie macierzy
	this->operator=(this->operator *(m));
}

K_vector Homog_matrix::operator*(const K_vector & w) const
{
	// operator mnozenia
	// umozliwia otrzymanie wektora w ukladzie odniesienia reprezentowanym przez macierz jednorodna
	// argument: obiekt klasy K_vector

	K_vector zwracany;

	// i - i-ta kolumna
	// j - j-ty wiersz

	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 3; i++)
			zwracany[j] += matrix_m[j][i] * w[i];

	for (int i = 0; i < 3; i++)
		zwracany[i] += matrix_m[i][3];

	return zwracany;
}

K_vector Homog_matrix::operator*(const double tablica[3]) const
{
	return this->operator *(K_vector(tablica));
}

Homog_matrix Homog_matrix::operator!() const
{
	// przeksztalcenie odwrotne
	// odwrocenie macierzy zgodnie ze wzorem 2.45 ze strony 55
	// ksiazki: "Wprowadzenie do robotyki" John J. Craig

	// i - i-ta kolumna
	// j - j-ty wiersz

	Homog_matrix zwracana;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			zwracana.matrix_m[i][j] = matrix_m[j][i];

	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 3; i++)
			zwracana.matrix_m[j][3] += -1 * zwracana.matrix_m[j][i] * matrix_m[i][3];

	return zwracana;
}

bool Homog_matrix::operator==(const Homog_matrix & comp) const
{
	// opeartor porownania - sprawdza czy dwa obiekty Homog_matrix sa takie same
	// warunki:
	// obie macierze musza spelniac warunek jednorodnosci
	// 	lub obie go nie spelniac
	// suma kwadratow elementow macierzy musi byc mniejsza od eps

	// zwraca:
	// true - macierze rowne
	// false - macierzy rozne

	// i - i-ta kolumna
	// j - j-ty wiersz

	const double eps = 1e-5;
	double val = 0;

	if (this->is_valid() != comp.is_valid())
		return false;

	Homog_matrix A(*this);
	Homog_matrix B(comp);

	Homog_matrix T(A * !B);

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 3; j++) {
			if (i == j)
				val += ((T.matrix_m[i][i] - 1) * (T.matrix_m[i][i] - 1));
			else
				val += (T.matrix_m[j][i] * T.matrix_m[j][i]);
		}

	// przekroczony eps => macierze sa rozne
	if (val > eps)
		return false;

	return true;
}

bool Homog_matrix::operator!=(const Homog_matrix & comp) const
{
	// operator porownania - sprawdza czy porownywane obiekty klasy Homog_matrix sa rozne od siebie
	return (!this->operator==(comp));
}

std::ostream & operator<<(std::ostream & stream, const Homog_matrix & m)
{
	// operator wypisania
	// przedstawia macierz jednorodna w przyjaznej dla czlowieka formie

	stream << "[\n";
	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 4; i++) {
			stream.width(8);
			stream.setf(std::ios::showpos | std::ios::left);
			stream << "\t" << m.matrix_m[j][i];
		}
		stream << ";\n";
	}
	stream << "\t" << 0 << "\t" << 0 << "\t" << 0 << "\t" << 1 << "\n]\n";

	return stream;
}

bool Homog_matrix::is_valid() const
{
	// funkcja sprawdza czy macierz jest macierza jednorodna
	// jesli tak jest to funkcja zwraca 'true'
	// w przeciwnym wypadku funkcja zwraca 'false'

	const double eps = 1e-5;

	// obliczenia zgodne ze wzorem 6.12 z punktu 6.3.1 pracy:
	// Biblioteka funkcji matematycznych wspomagajacych sterowanie robotami
	for (int i = 0; i < 3; i++) {
		double value = 0;
		for (int k = 0; k < 3; k++)
			value += (matrix_m[i][k] * matrix_m[i][k]);
		if ((value > 1 + eps) || (value < 1 - eps))
			return false;
	}

	// obliczenia zgodne ze wzorem 6.13 z punktu 6.3.1 pracy:
	// Biblioteka funkcji matematycznych wspomagajacych sterowanie robotami
	for (int i = 0; i < 3; i++) {
		int j = ((i + 1) % 3);
		double value = 0;
		for (int k = 0; k < 3; k++)
			value += (matrix_m[i][k] * matrix_m[j][k]);
		if ((value < -eps) || (value > eps))
			return false;
	}

	return true;
}

// Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
void Homog_matrix::set_translation_vector(double x, double y, double z)
{
	set_translation_vector(K_vector(x, y, z));
}

// Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
void Homog_matrix::set_translation_vector(const K_vector & xyz)
{
	matrix_m[0][3] = xyz(0, 0);
	matrix_m[1][3] = xyz(1, 0);
	matrix_m[2][3] = xyz(2, 0);
}

// Ustawienie wektora translacji. Macierz rotacji pozostaje niezmieniona.
void Homog_matrix::set_translation_vector(const double t[3])
{
	set_translation_vector(K_vector(t));
}

void Homog_matrix::set_translation_vector(const Homog_matrix &wzor)
{
	matrix_m[0][3] = wzor.matrix_m[0][3];
	matrix_m[1][3] = wzor.matrix_m[1][3];
	matrix_m[2][3] = wzor.matrix_m[2][3];
}

// wyzerowanie wektora translacji.
void Homog_matrix::remove_translation()
{
	matrix_m[0][3] = 0.0;
	matrix_m[1][3] = 0.0;
	matrix_m[2][3] = 0.0;
}

// wstawienie jedynek na diagonalii rotacji
void Homog_matrix::remove_rotation()
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == j)
				matrix_m[i][j] = 1;
			else
				matrix_m[i][j] = 0;
		}
	}
}

Homog_matrix Homog_matrix::return_with_with_removed_translation() const
{
	Homog_matrix ret(*this);

	ret.remove_translation();

	return ret;
}

Homog_matrix Homog_matrix::return_with_with_removed_rotation() const
{
	Homog_matrix ret(*this);

	ret.remove_rotation();

	return ret;
}

// Zwraca obecny wektor translacji.
void Homog_matrix::get_translation_vector(double t[3]) const
{
	t[0] = matrix_m[0][3];
	t[1] = matrix_m[1][3];
	t[2] = matrix_m[2][3];
}

// Ustawienie macierzy rotacji. Wektor translacji pozostaje niezmieniony.
void Homog_matrix::set_rotation_matrix(const double r[3][3])
{
	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 3; i++)
			matrix_m[j][i] = r[j][i];
}

void Homog_matrix::set_rotation_matrix(const Homog_matrix & wzor)
{
	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 3; i++)
			matrix_m[j][i] = wzor.matrix_m[j][i];
}

// Pobranie macierzy rotacji.
void Homog_matrix::get_rotation_matrix(double r[3][3]) const
{
	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 3; i++)
			r[j][i] = matrix_m[j][i];
}

void Homog_matrix::get_rotation_matrix(Eigen::Matrix3d& rot) const
{
	for(int i=0; i<3; ++i){
		for(int j=0; j<3; ++j){
			rot(i,j) = this->operator ()(i, j);
		}
	}
}

void Homog_matrix::set_rotation_matrix(const Eigen::Matrix3d& rot)
{
	for(int i=0; i<3; ++i){
		for(int j=0; j<3; ++j){
			this->operator ()(i, j) = rot(i,j);
		}
	}
}

Homog_matrix Homog_matrix::interpolate(double t, const Homog_matrix& other)
{
	Eigen::Matrix3d rot0, rot1;

	this->get_rotation_matrix(rot0);
	other.get_rotation_matrix(rot1);

	Eigen::Quaterniond q0(rot0), q1(rot1);

	Eigen::Quaterniond q_p = q0.slerp(t, q1);

	Homog_matrix hm_p;

	double x0 = operator()(0, 3), y0 = operator()(1, 3), z0 = operator()(2, 3);
	double x1 = other(0, 3), y1 = other(1, 3), z1 = other(2, 3);
	double x, y, z;
	x = x0 + t * (x1 - x0);
	y = y0 + t * (y1 - y0);
	z = z0 + t * (z1 - z0);

	hm_p.set_translation_vector(x, y, z);
	hm_p.set_rotation_matrix(q_p.toRotationMatrix());
	return hm_p;
}

} // namespace lib
} // namespace mrrocpp
