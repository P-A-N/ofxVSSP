#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "urg3d_sensor.h"

class ofxVSSP : public ofThread
{
public:

	ofxVSSP(const string _IP, const int _port, const int _horiz_interace = 2, const int _vert_interace = 4) :
		IP(_IP), port(_port), state(State::none),
		last_connect_try_time(0.0),
		horiz_interace(_horiz_interace), vert_interace(_vert_interace)
	{
		msg = "";
		connect_btn.addListener(this, &ofxVSSP::connect);
		start_dataflow_btn.addListener(this, &ofxVSSP::start_dataflow);
		stop_dataflow_btn.addListener(this, &ofxVSSP::stop_dataflow);
		disconnect_btn.addListener(this, &ofxVSSP::disconnect);
		restart_btn.addListener(this, &ofxVSSP::restart);

		gui.setup("op gui");
		gui.add(msg.setup("msg", ""));
		//gui.add(connect_btn.setup("connect"));
		//gui.add(start_dataflow_btn.setup("start_dataflow"));
		//gui.add(stop_dataflow_btn.setup("stop_dataflow"));
		//gui.add(disconnect_btn.setup("disconnect"));
		//gui.add(restart_btn.setup("restart"));

		connect();
		start_dataflow();

		startThread();
	}

	~ofxVSSP()
	{
		stop_dataflow();
		waitForThread(true);
		disconnect();
	}

	void update()
	{
		if (state == State::none)
		{
			if (ofGetElapsedTimef() - last_connect_try_time > 2.0)
			{
				connect();
				if (state == State::connected)
					start_dataflow();
				last_connect_try_time = ofGetElapsedTimef();
			}
		}
	}

	void draw_points()
	{
		if (state != State::dataflowing) return;
		if (data.empty()) return;

		for (auto d : data)
		{
			for (auto spot = 0; spot < d.spot_count; spot++)
			{
				for (auto echo = 0; echo < d.spots[spot].echo_count; echo++)
				{
					glm::vec3 v(
						d.spots[spot].point[echo].x_m,
						d.spots[spot].point[echo].y_m,
						d.spots[spot].point[echo].z_m);
					v *= 100;
					ofPushStyle();
					ofSetColor(ofColor::magenta);
					ofDrawBox(v, 2);
					ofPopStyle();
				}
			}
		}
	}

	void draw_gui() { gui.draw(); }
	deque<urg3d_measurement_data_t> get_raw_data() { return data; }

private:

	void threadedFunction()
	{
		while (isThreadRunning())
		{
			if (state != State::dataflowing) continue;

			while (true)
			{
				if (urg3d_next_receive_ready(&urg))
				{
					urg3d_measurement_data_t d;
					if (urg3d_high_get_measurement_data(&urg, &d) > 0)
					{
						data.push_back(d);
						if (data.size() > 200)
							data.pop_front();

						if ((string)msg == "wait for a while")
							msg = "getting data...";
					}
					else
					{
						ofLogError() << "fail to get data";
						urg3d_vssp_header_t header;
						char data[URG3D_MAX_RX_LENGTH];
						int length_data;
						if (urg3d_low_get_binary(&urg, &header, data, &length_data) > 0)
						{
							if (strncmp(header.type, "ERR", 3) == 0 || strncmp(header.type, "_er", 3) == 0)
							{
								ofLog() << "error " << (int)header.status[0] << " " << (int)header.status[1] << " " << (int)header.status[2] << " " << data;
								if (header.status[0] != '0') 
									break;
							}
						}
					}
				}
				else
				{
					auto sleep_msec = 10;
					ofLog() << "sleep " << sleep_msec << " msec";
					msg = "wait for a while";
					ofSleepMillis(sleep_msec);
					break;
				}
			}
		}
	}

	void connect()
	{
		if (state != State::none)
		{
			ofLogError() << "its already connected.";
			return;
		}

		ret = urg3d_open(&urg, IP.c_str(), port);
		if (ret < 0)
		{
			msg = "error urg3d_open " + ofToString(ret);
			ofLogError() << (string)msg;
			return;
		}
		else
			ofLog() << "open ok";

		urg3d_high_set_blocking_timeout_ms(&urg, 2000);

		urg3d_sensor_version_t version;
		ret = urg3d_high_blocking_get_sensor_version(&urg, &version);
		if (ret < 0)
		{
			msg = "error urg3d_high_blocking_get_sensor_version " + ofToString(ret);
			ofLogError() << (string)msg;
		}
		else
		{
			ofLog() << "version.vendor = " << version.vendor;
			ofLog() << "version.product = " << version.product;
			ofLog() << "version.serial = " << version.serial;
			ofLog() << "version.firmware = " << version.firmware;
			ofLog() << "version.protocol = " << version.protocol;
		}

		ret = urg3d_high_blocking_init(&urg);
		if (ret < 0)
		{
			msg = "error urg3d_high_blocking_init " + ofToString(ret);
			ofLogError() << (string)msg;
			disconnect();
			return;
		}
		ofLog() << "wait initialize... (about 30-60 seconds after power-on)";

		ret = urg3d_high_blocking_wait_finished_initialize(&urg);
		if (ret < 0)
		{
			msg = "error urg3d_high_blocking_wait_finished_initialize " + ofToString(ret);
			ofLogError() << (string)msg;
			disconnect();
			return;
		}

		ret = urg3d_high_blocking_set_horizontal_interlace_count(&urg, horiz_interace);
		if (ret < 0)
		{
			msg = "error urg3d_high_blocking_set_horizontal_interlace_count " + ofToString(ret);
			ofLogError() << (string)msg;
			disconnect();
			return;
		}

		ret = urg3d_high_blocking_set_vertical_interlace_count(&urg, vert_interace);
		if (ret < 0)
		{
			msg = "error urg3d_high_blocking_set_vertical_interlace_count " + ofToString(ret);
			ofLogError() << (string)msg;
			disconnect();
			return;
		}

		msg = "successfully connect";
		ofLog() << (string)msg;
		state = State::connected;
	}

	void start_dataflow()
	{
		if (state == State::none)
		{
			ofLogError() << "need to connect device.";
			return;
		}
		else if (state == State::dataflowing)
		{
			ofLogError() << "its getting data already";
			return;
		}

		ret = urg3d_high_start_data(&urg, URG3D_DISTANCE_INTENSITY);
		if (ret < 0)
		{
			msg = "error urg3d_high_start_data " + ofToString(ret);
			ofLogError() << (string)msg;
			disconnect();
			return;
		}
		else
		{
			data.clear();
			msg = "start dataflow";
			ofLog() << (string)msg;
			state = State::dataflowing;
		}
	}

	void stop_dataflow()
	{
		if (state == State::none)
		{
			ofLogError() << "need to connect device.";
			return;
		}
		else if (state == State::connected)
		{
			ofLogError() << "not start getting data yet.";
			return;
		}

		ret = urg3d_high_stop_data(&urg, URG3D_DISTANCE_INTENSITY);
		if (ret < 0)
		{
			msg = "error urg3d_high_stop_data " + ofToString(ret);
			ofLogError() << (string)msg;
			disconnect();
			return;
		}
		else
		{
			data.clear();
			msg = "stop dataflow";
			ofLog() << (string)msg;
			state = State::connected;
		}
	}

	void disconnect()
	{
		if (state == State::dataflowing)
			stop_dataflow();

		ret = urg3d_close(&urg);
		if (ret < 0)
		{
			msg = "error urg3d_close " + ofToString(ret);
			ofLogError() << (string)msg;
			return;
		}
		else
		{
			msg = "successfully close." + ofToString(ret);
			ofLog() << (string)msg;
			state = State::none;
		}
	}

	void restart()
	{
		if (state == State::dataflowing)
		{
			ofLogError() << "couldnt reboot device. stop data first.";
			return;
		}
		else if (state == State::none)
		{
			ofLogError() << "couldnt reboot device. connect to device first.";
			return;
		}

		ret = urg3d_high_blocking_restart(&urg);
		if (ret < 0)
		{
			msg = "couldnt restart device " + ofToString(ret);
			ofLogError() << (string)msg;
		}
		else
		{
			disconnect();
			msg = "restart device " + ofToString(ret);
			ofLog() << (string)msg;
			state = State::none;
		}
	}

	string IP; int port;
	enum struct State { none, connected, dataflowing }; State state;

	float last_connect_try_time;

	urg3d_t urg;
	deque<urg3d_measurement_data_t> data;
	int ret = 0; /* operation return */
	int horiz_interace, vert_interace;

	// gui
	ofxPanel gui;
	ofxLabel msg;
	ofxButton connect_btn, start_dataflow_btn, stop_dataflow_btn, disconnect_btn, restart_btn;
};