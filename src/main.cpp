#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
		int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

int main() {
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {
					double maxTargetVelocity = 90;
					// Extract location data and guide points,
					// converting from global to vehicle coordinates.
					vector<double> ptsx = j[1]["ptsx"]; // global coordinates
					vector<double> ptsy = j[1]["ptsy"];
					const int n = ptsx.size();
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					
					//Transformation of global coordinates in simulator to vehicle coordinate space
					Eigen::VectorXd pts_x(n);  // vehicle coordinates
					Eigen::VectorXd pts_y(n);
					float totalCIS = 0;
					float prev_x_vspace = 0;
					float prev_y_vspace = 0;
					float prev_slope = 0;
					
					for (int i=0; i < n; i++) {
						float dx = ptsx[i] - px;
						float dy = ptsy[i] - py;
						float r = sqrt(dx*dx + dy*dy); // distance r to the point from vehicle
						float beta = atan2(-dy, dx);  // angle of vector in global space
						float alpha = beta+psi;       // angle of vector in vehicle space
						
						float x_vspace = r*cos(alpha);
						float y_vspace = -r*sin(alpha);
						
						//The section below calculates the upcoming turn complexity for rule based throttle adjustment
						//Based on an increasing change in clope between vehincle space points over points
						//The further the points the stronger the change in slope is factored in. 
						if (i>0)
						{
							//Real slope between each two data points
							float slope = (y_vspace-prev_y_vspace)/(x_vspace-prev_x_vspace);
							//Slope change between this point and last point
							float slopeChange = slope-prev_slope;
							//CIS = change in slope squared to neglect direction + power by the index of the points
							//to make further points factor in stronger. This ensures slowing before the curves
							//and accelarating out of the turn. 
							float cis = pow(slopeChange,2) * pow(i,2);
							totalCIS = totalCIS + cis;
							prev_slope = slope;
						}
						prev_x_vspace = x_vspace;
						prev_y_vspace = y_vspace;
						pts_x[i] = x_vspace;     // x, vehicle space
						pts_y[i] = y_vspace;   // y, vehicle space
					}

					// Extract dynamic information and create our state
					// vector.  Account for latency by advancing the car along the current
					// speed and turning angle.  C
					
					//compute cross-track error and our
					// desired turning angle.
					Eigen::VectorXd state(6);
					double v = j[1]["speed"];
					double rho = j[1]["steering_angle"];
					auto coeffs = polyfit(pts_x, pts_y, 2);   // Fit a quadratic guide wire

					// Account for latency
					const double latency = 0.11;  // 100 ms
					const double Lf = 2.67;
					px = v * latency;
					psi = - v * rho / Lf * latency ;

					// Compute cross track error as the y where we're supposed to be,
					// in local coordinates, minus our current y, 0, or the origin.
					double cte = polyeval(coeffs, px);

					// Estimate our desired steering angle as the angle of a line
					// tangent to the start of our guideware curve.
					double epsi = -atan(coeffs[1]);

					// Store everything in an initial state, which we'll then
					// use a model to project forward in time.
					state << px, 0, psi, v, cte, epsi;

					// Choose our throttle and steering angle by finding an optimal
					// number of settings over the next few seconds, based on simulating
					// our vehicle and how it will likely move forward kinematically.
					if (totalCIS > 20)
					{
						totalCIS = 20;
					}
					double targetV;
					targetV = 80 - (totalCIS * 2);
					auto vars = mpc.Solve(state, coeffs, targetV);

					// Store our actuators in JSON to fire them off via a websocket
					// back into the simulator engine.
					json msgJson;
					
					msgJson["throttle"] = vars[0];
					double angle = -vars[1]/deg2rad(25.0);
					msgJson["steering_angle"] = angle;
					
					std::cout << std::setprecision(2) << std::fixed;
					std::cout
					<< "Turn_Complexity: "
					<< totalCIS 
					<< " Target velocity: "
					<< targetV 
					<< " Throttle: "
					<< vars[0] 
					<< " Steer angle: "
					<< angle 
					<<std::endl;
					
					// Add our predicted trajectory, in green
					const int npts = (int) vars[2];
					vector<double> mpc_x_vals(npts);
					vector<double> mpc_y_vals(npts);

					// Points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Green line
					for (int i=0; i < npts; i++) {
						mpc_x_vals[i] = vars[i*2+3];
						mpc_y_vals[i] = vars[i*2+4];
						//
						// Uncomment this line to see our guidewire instead of our
						// predicted path forward.
						//
						//mpc_y_vals[i] = polyeval(coeffs, pts_x[i]);
					}

					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;

					// Display the waypoints/reference line in yellow
					vector<double> next_x_vals(n);
					vector<double> next_y_vals(n);
					for (int i=0; i < n; i++) {
						double x = pts_x[i];
						next_x_vals[i] = x;
						next_y_vals[i] = pts_y[i];
					}
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;


					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << "Sending " << msg << std::endl;
					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.
					//
					// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
					// SUBMITTING.
					this_thread::sleep_for(chrono::milliseconds(100));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
			size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}