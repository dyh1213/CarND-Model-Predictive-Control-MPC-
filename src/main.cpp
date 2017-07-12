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
#include <iomanip>

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
	}
	else if (b1 != string::npos && b2 != string::npos) {
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

Eigen::MatrixXd transformGlobalToLocal(double x, double y, double psi, const vector<double> & ptsx, const vector<double> & ptsy) {

	assert(ptsx.size() == ptsy.size());
	unsigned len = ptsx.size();

	auto waypoints = Eigen::MatrixXd(2, len);

	for (auto i = 0; i < len; ++i) {
		waypoints(0, i) = cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
		waypoints(1, i) = -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);
	}

	return waypoints;
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
					std::vector<double> points_xs = j[1]["ptsx"];
					std::vector<double> points_ys = j[1]["ptsy"];

					const double px = j[1]["x"];
					const double py = j[1]["y"];
					const double psi = j[1]["psi"];
					const double speed_mph = j[1]["speed"]; // 
					const double speed_mps = speed_mph * 0.44704; //-> m/s
					const double delta = j[1]["steering_angle"]; 
					const double throttle = j[1]["throttle"];
					const double a = throttle; 

					const double delay = 0.1; //ms
					const double Lf = 2.67;

					//Using the kinematic model
					const double delayed_px = px + speed_mps * cos(psi) * delay;
					const double delayed_py = py + speed_mps * sin(psi) * delay;
					const double delayed_psi = psi + (speed_mps * tan(-delta) / Lf) * delay + ((a * tan(-delta) / (2 * Lf)) * pow(delay, 2));
					//const double delayed_psi = psi + (speed_mps / Lf) * delta * delay;
					const double delayed_v = speed_mps + a * delay;

					Eigen::MatrixXd waypoints = transformGlobalToLocal(delayed_px, delayed_py, delayed_psi, points_xs, points_ys);
					Eigen::VectorXd waypoints_xs = waypoints.row(0);
					Eigen::VectorXd waypoints_ys = waypoints.row(1);

					const int n = points_xs.size();
					// Extract dynamic information and create our state
					// vector.  Account for latency by advancing the car along the current
					// speed and turning angle.  C

					//compute cross-track error and our
					// desired turning angle.
					Eigen::VectorXd state(6);

					auto coeffs = polyfit(waypoints_xs, waypoints_ys, 2);   // Fit a quadratic guide wire

					double cte = polyeval(coeffs, delayed_px);

					double epsi = -atan(coeffs[1]);

					state << 0, 0, 0, delayed_v, cte, epsi;

					auto vars = mpc.Solve(state, coeffs, 30);

					// Store our actuators in JSON to fire them off via a websocket
					// back into the simulator engine.
					json msgJson;

					msgJson["throttle"] = vars[0];
					double angle = -vars[1] / deg2rad(25.0);
					msgJson["steering_angle"] = angle;

					std::cout << std::setprecision(2) << std::fixed;
					std::cout
						<< "Turn_Complexity: "
						<< 80
						<< " Target velocity: "
						<< 80
						<< " Throttle: "
						<< vars[0]
						<< " Steer angle: "
						<< angle
						<< std::endl;

					// Add our predicted trajectory, in green
					const int npts = (int)vars[2];
					vector<double> mpc_x_vals(npts);
					vector<double> mpc_y_vals(npts);

					// Points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Green line
					for (int i = 0; i < npts; i++) {
						mpc_x_vals[i] = vars[i * 2 + 3];
						mpc_y_vals[i] = vars[i * 2 + 4];
						//
						// Uncomment this line to see our guidewire instead of our
						// predicted path forward.
						//
						//mpc_y_vals[i] = polyeval(coeffs, pts_x[i]);
					}

					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;

				  //Display the waypoints/reference line
				  vector<double> next_x_vals;
				  vector<double> next_y_vals;

				  //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
				  // the points in the simulator are connected by a Yellow line

				  for (double i = 3; i < 50; i += 3){
					next_x_vals.push_back(i);
					next_y_vals.push_back(polyeval(coeffs, i));
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
			}
			else {
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
		}
		else {
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
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}