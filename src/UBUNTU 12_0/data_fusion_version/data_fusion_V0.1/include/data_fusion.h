#ifndef DATA_FUSION_
#define DATA_FUSION_
#include "iostream"
#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/bind.hpp"

class data_fusion{
	public:
	struct gryo_data{
		std::string time_stamp;
		std::string angle[3];
		std::string angle_vel[3];
		std::string linear_acc[3];
	};
	data_fusion(std::string Port,int baud_rate,size_t timeout,int Freq);
	//void serial_read(const boost::system::error_code& error);
	void serial_read();
	void freq_serial_read();
	////serial connect////

	private:
	void read_complete(const boost::system::error_code& error,size_t bytes_transferred);
	void time_out(const boost::system::error_code& error);
	bool read_char(char& val);
	
	
	boost::asio::io_service io;
	boost::asio::io_service io_freq;
	boost::asio::serial_port port_server;
	size_t timeout;
	char c;
	double Freq_;
	boost::asio::deadline_timer timer_read;
	boost::asio::deadline_timer timer_program_freq;
	bool read_error;
	std::vector<gryo_data> gryo_data_set;
	};
#endif //DATA_FUSION_


