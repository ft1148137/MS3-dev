#ifndef DATA_FUSION_
#define DATA_FUSION_

#define Nsta 3   
#define Mobs 3

#include "iostream"
#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"

#include <TinyEKF.h>

class data_fusion : public TinyEKF{
	public:
	struct gryo_data{
		std::string time_stamp;
		std::string angle[3];
		std::string angle_vel[3];
		std::string linear_acc[3];
	};
	data_fusion(std::string Port,int baud_rate,size_t timeout,double Freq_read);
	
	void data_fusion_core();
	void serial_read();
	void freq_serial_read();
	bool reset = false;

	private:

	void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]);
	void read_complete(const boost::system::error_code& error,size_t bytes_transferred);
	void time_out(const boost::system::error_code& error);
	bool read_char(char& val);
	
	
	boost::asio::io_service io;
	boost::asio::io_service io_freq;
	boost::asio::serial_port port_server;
	size_t timeout;
	char c;
	double Freq_read;
	double Freq_arduino;
	boost::asio::deadline_timer timer_read;
	boost::asio::deadline_timer timer_program_freq;
	bool read_error;

	std::vector<gryo_data> gryo_data_set;
	};
#endif //DATA_FUSION_


