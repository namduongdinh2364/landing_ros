#include "generate_curve.h"
#include "landing.h"

#define DEGREE2RADIAN(x)	x*3.14159265/180

namespace generate_curve {
	void getIntersectionPoints(vector<Eigen::Vector3d> *points, const Eigen::Vector3f& marker_pos, const Eigen::Vector3f& uav_pos) {
		Eigen::Vector3f pointa(marker_pos(0), marker_pos(1), uav_pos(2));
		Eigen::Vector3d point10, point20, point25, point0, point4;
		double distance, distance10 , distance20, distance25;
		distance = (uav_pos - pointa).norm();
		distance25 = tan(DEGREE2RADIAN(25)) * 9.0;
		distance20 = tan(DEGREE2RADIAN(20)) * 8.0;
		distance10 = tan(DEGREE2RADIAN(10)) * 6.0;
		/*
		*							C
		*						/	|
		*					B		|
		*				/	|		|
		*			A		|		|
		*		/	|		|		|
		*	O-------F-------E-------D
		*/
		double varCD_, varOD_, varAF_, varsinCOD_, varOF_;
		varsinCOD_ = abs(uav_pos(1) - marker_pos(1)) /distance;

		varAF_ = distance10 * varsinCOD_;
		varOF_ = sqrt(pow(distance10,2) - pow(varAF_,2));

		varCD_ = distance20 * varsinCOD_;
		varOD_ = sqrt(pow(distance20,2) - pow(varCD_,2));
		if (marker_pos(0) > uav_pos(0)) {
			point10(0) = marker_pos(0) - varOF_;
			point20(0) = marker_pos(0) - varOD_;
		}
		else {
			point10(0) = marker_pos(0) + varOF_;
			point20(0) = marker_pos(0) + varOD_;
		}

		if (marker_pos(1) > uav_pos(1)) {
			point10(1) = marker_pos(1) - varAF_;
			point20(1) = marker_pos(1) - varCD_;
		}
		else {
			point10(1) = marker_pos(1) + varAF_;
			point20(1) = marker_pos(1) + varCD_;
		}
		point10(2) = 6.0;
		point20(2) = 8.0;

		// if (uav_pos(2) > 8.2 && (distance > distance20)) {
		// 	points->push_back(point20);
		// 	std::cout << "point20: " << point20 << std::endl;
		// }
		// if (uav_pos(2) > 6.5 && (distance > distance10)) {
		// 	points->push_back(point10);
		// 	std::cout << "point10: " << point10 << std::endl;
		// }
		if (uav_pos(2) > 8.5)
		{
			point4 << marker_pos(0), marker_pos(1), 8.0;
			points->push_back(point4);
		}
		else {
			point0 << marker_pos(0), marker_pos(1), ALTITUDE_CHANGE_METHOD - 1.0;
			points->push_back(point0);
		}

		// std::cout << "distance 20: " << distance20 << std::endl;
		// std::cout << "distance 10: " << distance10 << std::endl;
		// std::cout << "distance: " << distance << std::endl;
	}
};