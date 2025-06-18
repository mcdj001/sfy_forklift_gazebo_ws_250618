/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-04 13:56:34
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-09-04 13:43:47
 */

#include "mytools.h"
using namespace SfyMotionController;

double MyTools::cal_K(vector<waypoint> waypoints, int index){
	double res;
	//差分法求一阶导和二阶导
	double dx, dy, ddx, ddy;
	if (index == 0) {
		dx = waypoints[1].x - waypoints[0].x;
		dy = waypoints[1].y - waypoints[0].y;
		ddx = waypoints[2].x + waypoints[0].x - 2 * waypoints[1].x;
		ddy = waypoints[2].y + waypoints[0].y - 2 * waypoints[1].y;
	}
	else if (index == (waypoints.size() - 1)) {
		dx = waypoints[index].x - waypoints[index - 1].x;
		dy = waypoints[index].y - waypoints[index - 1].y;
		ddx = waypoints[index].x + waypoints[index - 2].x - 2 * waypoints[index].x;
		ddy = waypoints[index].y + waypoints[index - 2].y - 2 * waypoints[index].y;
	}
	else {
		dx = waypoints[index + 1].x - waypoints[index].x;
		dy = waypoints[index + 1].y - waypoints[index].y;
		ddx = waypoints[index + 1].x + waypoints[index - 1].x - 2 * waypoints[index].x;
		ddy = waypoints[index + 1].y + waypoints[index - 1].y - 2 * waypoints[index].y;
	}
	//res.yaw = atan2(dy, dx);//yaw
	//计算曲率：设曲线r(t) =(x(t),y(t)),则曲率k=(x'y" - x"y')/((x')^2 + (y')^2)^(3/2).
	res = (ddy * dx - ddx * dy) / (sqrt(pow((pow(dx, 2.0) + pow(dy, 2.0)), 3.0)));
	if(dx == 0 &&  dy == 0)  res = 0;
	return res;
}

double MyTools::normalizeAngle(double angle) {
    while(angle >= M_PI){
        angle -= 2.0*M_PI;
    }
    while(angle <= - M_PI){
        angle += 2.0*M_PI;
    }
    return angle;
}

double MyTools::factorial(int n) {
    if(n<=1)return 1;
    return factorial(n-1)*n;
}

Vector2d MyTools::bezierCommon(vector<Vector2d> Ps, double t) {

    if(Ps.size()==1)return Ps[0];

    Vector2d p_t(0.,0.);
    int n = Ps.size()-1;
    for(int i=0;i<Ps.size();i++){
        double C_n_i = factorial(n)/ (factorial(i)* factorial(n-i));
        p_t +=  C_n_i * pow((1.0-t),(n-i)) * pow(t,i)*Ps[i];
        //cout<<t<<","<<1-t<<","<<n-i<<","<<pow((1-t),(n-i))<<endl;
    }
    return p_t;
}
