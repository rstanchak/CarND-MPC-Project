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

#ifdef CONFIG_MYBUILD
typedef uWS::WebSocket<uWS::SERVER> * WsServer;
#else
typedef uWS::WebSocket<uWS::SERVER> WsServer;
#endif

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double normalize_angle(double theta) {
    return atan2( sin(theta), cos(theta) );
}

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

struct MapToVehicleTransform {
    double cos_theta, sin_theta, tx, ty;
    MapToVehicleTransform(double px, double py, double theta):
        cos_theta(cos(theta)),
        sin_theta(sin(theta)),
        tx(-px),
        ty(-py) { }
    void operator()(double x, double y, double & retarg_x, double & retarg_y) {
        retarg_x = (x+tx)*cos_theta + (y+ty)*sin_theta;
        retarg_y = -(x+tx)*sin_theta + (y+ty)*cos_theta;
    }
};

struct VehicleToMapTransform {
    double cos_theta, sin_theta, tx, ty;
    VehicleToMapTransform(double px, double py, double theta):
        cos_theta(cos(-theta)),
        sin_theta(sin(-theta)),
        tx(px),
        ty(py) { }
    void operator()(double x, double y, double & retarg_x, double & retarg_y) {
        retarg_x = x*cos_theta + y*sin_theta + tx;
        retarg_y = -x*sin_theta + y*cos_theta + ty;
    }
};

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> * ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        std::cout << "---------------------------------------------------------" << std::endl;
        std::cout << s << std::endl;
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = normalize_angle(j[1]["psi"]);
          double v = j[1]["speed"];

          Eigen::VectorXd xvals(ptsx.size()), yvals(ptsy.size());
          for(size_t i=0; i<ptsx.size(); ++i) {
              xvals[i] = ptsx[i];
              yvals[i] = ptsy[i];
          }

		  Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);
		  Eigen::VectorXd dcoeffs(3);
          dcoeffs << coeffs[1], 2*coeffs[2], 3*coeffs[3];

          double cte =  py - polyeval(coeffs, px);
          // \psi_des_t can be calculated as the tangential angle of the polynomial f evaluated at x_t
          // arctan(f'(x_t))
          // f'(x_t) = 3*coeffs[3]*x^2 + 2*coeffs[2]*x + coefs[1]
          // well this is a mess.  Not sure how to get proper direction with this.
          // double psi_des = atan2(polyeval(dcoeffs,px), 1.);
          // instead, approximate psi_des by fitting a line between the first two waypoints
          double psi_des = atan2( ptsy[1]-ptsy[0], ptsx[1]-ptsx[0]);
          double epsi = normalize_angle(psi - psi_des);
          std::cout<<"psi: "<<psi<<"\tpsi_des:"<<psi_des<<"\tepsi:" << epsi;
          std::cout<<"\tx,y: "<<px<<","<<py<<"\tf(x)="<<polyeval(coeffs, px)<<"\tcte: "<<cte;
          std::cout<<"\tv: "<<v<<std::endl;

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
          //std::cout<<"state:"<<state<<std::endl;
          //std::cout<<"coeffs:"<<coeffs<<std::endl;

          // solution is x, y, psi, v, cte, epsi, delta, a
          auto solution = mpc.Solve(state, coeffs);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = -solution[6];///deg2rad(25);
          double throttle_value = solution[7];
          std::cout<<"steer:"<<steer_value<<"\tthrottle:"<<throttle_value<<std::endl;
          

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals ;
          vector<double> mpc_y_vals;

          MapToVehicleTransform map2vehicle(px, py, psi);

          mpc_x_vals.push_back(0);
          mpc_y_vals.push_back(0);
          {
              double x,y;
              map2vehicle(solution[0], solution[1], x, y);
              mpc_x_vals.push_back(x);
              mpc_y_vals.push_back(y);
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for(size_t i=0; i<ptsx.size(); ++i)
          {
              double x, y;
              map2vehicle(ptsx[i], ptsy[i], x, y);
              next_x_vals.push_back(x);
              next_y_vals.push_back(y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

#if 0
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

#endif
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> * ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> * ws, int code,
                         char *message, size_t length) {
    ws->close();
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
